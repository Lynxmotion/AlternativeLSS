#include "LssPosixChannel.h"
#include "../../LssChannel.h"

#include <termios.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <fcntl.h>
#include <sys/signal.h>
#include <sys/types.h>
#include <sys/select.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/serial.h>
#include <pthread.h>

#include <functional>
#include <poll.h>
#include <asm/ioctls.h>

#define BAUDRATE B230400
#define _POSIX_SOURCE 1 /* POSIX compliant source */


#if 0
#define IFLOG(x) x
#else
#define IFLOG(x)
#endif

//#define LOG_ACTIONS

class posix_serial_private {
public:
    // processing loop variables
    int fd;
    pthread_t loop;
    ChannelState state; // current loop task/state

    union {
        int client_service[2];
        struct {
            int client;
            int service;
        };
    } notify;

    inline posix_serial_private()
        : fd(0), loop(0), state(ChannelStopped)
    {
        notify.client = 0;
        notify.service = 0;
    }
};


LssPosixChannel::LssPosixChannel(LssChannel* channel)
        : LssChannelDriver(channel), priv(nullptr), devname(nullptr), baudrate(115200)
{}

LssPosixChannel::~LssPosixChannel() {
    if(priv != nullptr && priv->state <= ChannelStarting) {

        // stop and join the processing loop
        void* rv = nullptr;
        pthread_cancel(priv->loop);
        pthread_join(priv->loop, &rv);

        // remove the private block
        delete priv;
        priv = nullptr;
    }
}

ChannelDriverError LssPosixChannel::begin(const char* _devname, int _baudrate)
{
    if(priv !=nullptr)
        return DriverAlreadyInitialized;

    if(devname) ::free((void*)devname);
    devname = strdup(_devname);
    baudrate = _baudrate;

    statistics = Statistics();

    priv = new posix_serial_private();

    // start the processing loop
    priv->state = ChannelStarting;
    if(pthread_create( &priv->loop, NULL, s_run, (void*) this) !=0) {
        // todo: should probably print some kind of error here
        printf("failed to start serial processing thread\n");
        priv = nullptr;
    }

    // wait for channel processing loop to startup
    int timeout = 5000;
    while(--timeout && priv->state == ChannelStarting)
        usleep(1000);   // 1ms

    if(priv->state <= ChannelStarting) {
        // processing loop failed to start in time
        void* rv = nullptr;
        pthread_cancel(priv->loop);
        pthread_join(priv->loop, &rv);
        delete priv;
        priv = nullptr;
        return DriverOpenFailed;
    }
    return DriverSuccess;
}

void* LssPosixChannel::s_run(void* inst) {
    return ((LssPosixChannel*)inst)->run();
}

void* LssPosixChannel::run()
{
    if(priv == nullptr) return nullptr;
    priv->state = ChannelStarting;

    int res;
    int errors = 0;
    int consecutive_errors = 0;
    int sigs = 0;

    // file descriptor of serial port
    int fd;
    struct pollfd ufds[2];
    timespec time = {0, 100000};

    // setup our signal masks for ppoll()
    sigset_t sigmask;
    sigfillset(&sigmask);
    sigdelset(&sigmask, SIGALRM);   // break only for SIGALRM signal

    // our port settings, and saved settings to restore when closing
    struct termios oldtio, newtio;
    struct serial_struct serial;
    bool restore_tio = false;

    // todo: just move this into the processling loop
    char buffer[256];
    char* pbuffer = buffer;

    //LssTransaction* ctx = nullptr;

reopen:
    /* open the device to be non-blocking (read will return immediately) */
    fd = ::open(devname, O_RDWR | O_NOCTTY | O_NDELAY | O_NONBLOCK);
    if (fd <0) {
        perror(devname);
        errors++;
        consecutive_errors++;
        if(consecutive_errors > 300) {
            printf("too many consecutive errors, aborting.\n");
            goto exit_serial_processing;
        }
        sleep(1);
        goto reopen;
    }

    // todo: generate a callback or promise here so the app can do some startup init anytime the port is reopened

    // setup the client/service notification domain socket
    if(socketpair(AF_LOCAL, SOCK_STREAM | SOCK_NONBLOCK, 0, priv->notify.client_service) == -1) {
        perror("failed to create domain socket for posix serial processing");
        goto exit_serial_processing;
    }

    /* Make the file descriptor asynchronous (the manual page says only
       O_APPEND and O_NONBLOCK, will work with F_SETFL...) */
    //fcntl(fd, F_SETFL, FASYNC);

    ioctl(fd, TIOCGSERIAL, &serial);
    serial.flags |= ASYNC_LOW_LATENCY;
    res = ioctl(fd, TIOCSSERIAL, &serial);

    tcgetattr(fd,&oldtio); /* save current port settings */
    newtio.c_cflag = BAUDRATE | CS8 | CLOCAL | CREAD; // | CRTSCTS
    newtio.c_cflag &= ~(PARENB | PARODD); // No parity
    newtio.c_cflag &= ~CRTSCTS; // No hardware handshake
    newtio.c_cflag &= ~CSTOPB; // 1 stopbit

    newtio.c_iflag = IGNBRK;
    newtio.c_iflag &= ~(IXON | IXOFF | IXANY); // No software handshake
    newtio.c_iflag = IGNPAR;

    newtio.c_oflag = 0;
    newtio.c_lflag = 0;
    newtio.c_lflag &= ~(ICANON|ECHO); /* Clear ICANON and ECHO. */

    newtio.c_cc[VMIN]=1;
    newtio.c_cc[VTIME]=0;
    tcflush(fd, TCIFLUSH);
    tcsetattr(fd,TCSANOW,&newtio);
    restore_tio = true; // remember to restore old settings

    priv->state = ChannelIdle;
    priv->fd = fd;

    ufds[0].fd = fd;
    ufds[0].events = POLLIN;
    ufds[1].fd = priv->notify.service;
    ufds[1].events = POLLIN;

    /* allow the process to receive SIGIO */
    //fcntl(fd, F_SETOWN, getpid());

    /* after receiving SIGIO, wait_flag = FALSE, input is available
       and can be read */
    while (priv->state >= ChannelIdle && consecutive_errors < 15) {
        priv->state = ChannelIdle;

        int res = ppoll(ufds, 2, &time, &sigmask);
        if(res<0) {
            // alarm condition
            IFLOG(perror("polling error"));
            perror("polling error");
            goto exit_serial_processing;
        } else if(res == 0) {
            // check for new transaction to send
            channel->driverIdle();
            continue;
        }

        // check if poll() is telling us this port is closed or missing
        if(ufds[0].revents & (POLLNVAL|POLLERR|POLLHUP)) {
            priv->state = ChannelError;
            // port is not longer available
            ::close(fd);
            priv->fd = fd = 0;
            goto reopen;
        } else if(ufds[1].revents & POLLIN) {
            char msg[32];
            int bytes_read = read(priv->notify.service, msg, sizeof(msg));
            sigs++;
            channel->driverIdle();
            continue;
        }

        // process data, read from serial
        priv->state = ChannelProcessing;

        //IFLOG(if(pbuffer > buffer)  printf("[+= %s]\n", buffer));
        int bytes_read = read(fd, pbuffer, sizeof(buffer) - (pbuffer - buffer));
        char *pb = buffer;
        pbuffer += bytes_read;
        *pbuffer = 0;  // null terminate
        statistics.bytes_received += bytes_read;

        char* p = pb;
        while(*p) {
            if(*p == '*') {
                pb = p;
            }
            else if(*p == '\r') {
                IFLOG(printf("<=%s\n", pb));

                // dispatch packet to destination servo (if we have it)
                LynxPacket packet(pb + 1);
                channel->driverDispatch(packet);
                *p = 0;
                pb = p + 1;
            }
            p++;
        }

        if(pb > buffer) {
            // move buffer to remove processed packets
            int n = pbuffer - pb;
            if(n > 0) {
                memmove(buffer, pb, n+1);
                pbuffer = buffer + n;
            } else
                pbuffer = buffer;
        }
    }

exit_serial_processing:
    priv->state = ChannelStopping;
    if(fd >0) {
        ::close(fd);
        fd = 0;
    }
    priv->fd = 0;

    // restore old port settings
    if(restore_tio)
        tcsetattr(fd, TCSANOW, &oldtio);

    if(priv->notify.service)
        ::close(priv->notify.service);
    if(priv->notify.client)
        ::close(priv->notify.client);
    priv->notify.client = priv->notify.service = 0;

    priv->state = ChannelError;
    return nullptr;
}

intptr_t LssPosixChannel::signal(ChannelDriverSignal signal, unsigned long a, const void* ptr)
{
    // we send a simple character to awaken the processing thread
    switch(signal) {
        case OpenSignal:
            begin( (const char*)ptr, a);
            break;

        case TransactionSignal:
        case DataSignal:
            write(priv->notify.client, "*", 1);
            break;

        case TransmitSignal:
            if(ptr && a>0)
                transmit((const char*)ptr, a);
            break;
    }
    return 0;
}

void LssPosixChannel::transmit(const char* pkt_bytes, int count) {
    if(priv->state >= ChannelIdle) {
        if(count<0)
            count = strlen(pkt_bytes);
        IFLOG(printf("%lld=>%s", tt, pkt_bytes));

        statistics.bytes_sent += count;
        write(priv->fd, pkt_bytes, count);
#ifdef LOG_ACTIONS
        printf("=> %s\n", pkt_bytes);
#endif
    }
}

