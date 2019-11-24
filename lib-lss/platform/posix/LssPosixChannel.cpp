#include "LssPosixChannel.h"
#include "../../LssServo.h"

#include <termios.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/signal.h>
#include <sys/types.h>
#include <sys/select.h>
#include <sys/ioctl.h>
#include <linux/serial.h>
#include <pthread.h>

#include <functional>
#include <poll.h>
#include <asm/ioctls.h>

#define BAUDRATE B115200
#define _POSIX_SOURCE 1 /* POSIX compliant source */


#if 0
#define IFLOG(x) x
#else
#define IFLOG(x)
#endif

class posix_serial_private {
public:
    // processing loop variables
    int fd;
    pthread_t loop;
    ChannelState state; // current loop task/state

    inline posix_serial_private()
        : fd(0), loop(0), state(ChannelStopped)
    {}
};


LssPosixChannel::LssPosixChannel(const char* channel_name)
        : LssChannelBase(channel_name), priv(nullptr), devname(nullptr), baudrate(115200), bytes_sent(0), bytes_received(0)
{}

LssPosixChannel::~LssPosixChannel() {
    free();
}

bool LssPosixChannel::begin(const char* _devname, int _baudrate)
{
    free();     // close and will reopen

    if(devname) ::free((void*)devname);
    devname = strdup(_devname);
    baudrate = _baudrate;

    bytes_sent = 0;
    bytes_received = 0;

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
        return false;
    }
    return true;
}

void LssPosixChannel::free() {
    // todo: rename this to close()
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

void* LssPosixChannel::s_run(void* inst) {
    return ((LssPosixChannel*)inst)->run();
}

void* LssPosixChannel::run()
{
    if(priv == nullptr) return nullptr;
    priv->state = ChannelStarting;

    int errors = 0;
    int consecutive_errors = 0;

    // file descriptor of serial port
    int fd;
    fd_set readfs;
    struct pollfd ufds;
    struct timeval timeout;

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
        if(consecutive_errors > 300)
            goto exit_serial_processing;
        sleep(1);
        goto reopen;
    }

    // todo: generate a callback or promise here so the app can do some startup init anytime the port is reopened

    /* Make the file descriptor asynchronous (the manual page says only
       O_APPEND and O_NONBLOCK, will work with F_SETFL...) */
    //fcntl(fd, F_SETFL, FASYNC);

    ioctl(fd, TIOCGSERIAL, &serial);
    serial.flags |= ASYNC_LOW_LATENCY;
    ioctl(fd, TIOCSSERIAL, &serial);

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

    /* Initialize the input set */
    FD_ZERO(&readfs);
    FD_SET(fd, &readfs);

    ufds.fd = fd;
    ufds.events = POLLIN;

    /* allow the process to receive SIGIO */
    //fcntl(fd, F_SETOWN, getpid());

    /* after receiving SIGIO, wait_flag = FALSE, input is available
       and can be read */
    while (priv->state >= ChannelIdle && consecutive_errors < 15) {
        priv->state = ChannelIdle;

        /* Initialize the timeout structure */
        timeout.tv_sec  = 0;
        timeout.tv_usec = 1000;

        int res = poll(&ufds, 1, 1);
        if(res<0) {
            IFLOG(perror("polling error"));
            goto exit_serial_processing;
        } else if(res == 0) {
            // check for new transaction to send
            driverIdle();
            continue;
        }

        // check if poll() is telling us this port is closed or missing
        if(ufds.revents & (POLLNVAL|POLLERR|POLLHUP)) {
            priv->state = ChannelError;
            // port is not longer available
            close(fd);
            priv->fd = fd = 0;
            goto reopen;
        }

        // process data, read from serial
        priv->state = ChannelProcessing;

        //IFLOG(if(pbuffer > buffer)  printf("[+= %s]\n", buffer));
        int bytes_read = read(fd, pbuffer, sizeof(buffer) - (pbuffer - buffer));
        char *pb = buffer;
        pbuffer += bytes_read;
        *pbuffer = 0;  // null terminate
        bytes_received += bytes_read;

        char* p = pb;
        while(*p) {
            if(*p == '*') {
                pb = p;
            }
            else if(*p == '\r') {
                IFLOG(printf("<=%s\n", pb));

                // dispatch packet to destination servo (if we have it)
                LynxPacket packet(pb + 1);
                driverDispatch(packet);
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
        close(fd);
        fd = 0;
    }
    priv->fd = 0;

    // restore old port settings
    if(restore_tio)
        tcsetattr(fd, TCSANOW, &oldtio);

    priv->state = ChannelError;
    return nullptr;
}

void LssPosixChannel::update() {
    // now in update we just check for notifications we need to process on the main thread
}

void LssPosixChannel::transmit(const char* pkt_bytes, int count) {
    if(priv->state >= ChannelIdle) {
        if(count<0)
            count = strlen(pkt_bytes);
        IFLOG(printf("=>%s", pkt_bytes));
        bytes_sent += count;
        write(priv->fd, pkt_bytes, count);
    }
}

