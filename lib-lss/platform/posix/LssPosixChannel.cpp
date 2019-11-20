#include "LssPosixChannel.h"
#include "../../LynxmotionLSS.h"

#include <termios.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/signal.h>
#include <sys/types.h>
#include <sys/select.h>
#include <pthread.h>

#include <functional>

#define BAUDRATE B115200
#define _POSIX_SOURCE 1 /* POSIX compliant source */


#if 0
#define IFLOG(x) x
#else
#define IFLOG(x)
#endif

class posix_serial_private {
public:
    // file descriptor of serial port
    int fd;

    // our port settings, and saved settings to restore when closing
    struct termios oldtio, newtio;

    // processing loop variables
    pthread_t loop;
    ChannelState state; // current loop task/state

    inline posix_serial_private()
        : fd(0), state(ChannelStopped)
    {}
};


LssPosixChannel::LssPosixChannel(const char* channel_name)
        : LssChannelBase(channel_name), priv(nullptr), devname(nullptr), baudrate(115200), bytes_sent(0), bytes_received(0)
{}

LssPosixChannel::~LssPosixChannel() {
    free();
}

void LssPosixChannel::begin(const char* _devname, int _baudrate)
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
    }
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

void LssPosixChannel::open()
{
    // todo: just move this into the processling loop
    pbuffer = buffer;

    /* open the device to be non-blocking (read will return immediatly) */
    priv->fd = ::open(devname, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (priv->fd <0) {
        perror(devname);
        goto error;
    }

    /* Make the file descriptor asynchronous (the manual page says only
       O_APPEND and O_NONBLOCK, will work with F_SETFL...) */
    //fcntl(priv->fd, F_SETFL, FASYNC);

    tcgetattr(priv->fd,&priv->oldtio); /* save current port settings */
    priv->newtio.c_cflag = BAUDRATE | CS8 | CLOCAL | CREAD; // | CRTSCTS
    priv->newtio.c_cflag &= ~(PARENB | PARODD); // No parity
    priv->newtio.c_cflag &= ~CRTSCTS; // No hardware handshake
    priv->newtio.c_cflag &= ~CSTOPB; // 1 stopbit

    priv->newtio.c_iflag = IGNBRK;
    priv->newtio.c_iflag &= ~(IXON | IXOFF | IXANY); // No software handshake
    priv->newtio.c_iflag = IGNPAR;

    priv->newtio.c_oflag = 0;
    priv->newtio.c_lflag = 0;
    priv->newtio.c_cc[VMIN]=1;
    priv->newtio.c_cc[VTIME]=0;
    tcflush(priv->fd, TCIFLUSH);
    tcsetattr(priv->fd,TCSANOW,&priv->newtio);

    return;
error:
    if(priv->fd >0) {
        close(priv->fd);
        priv->fd = 0;
    }
}

void* LssPosixChannel::s_run(void* inst) {
    return ((LssPosixChannel*)inst)->run();
}

void* LssPosixChannel::run()
{
    if(priv == nullptr) return nullptr;
    priv->state = ChannelStarting;

    fd_set         readfs;
    struct timeval timeout;
    int errors = 0;
    int consecutive_errors = 0;

    open();

    priv->state = ChannelIdle;

    /* Initialize the input set */
    FD_ZERO(&readfs);
    FD_SET(priv->fd, &readfs);

    /* allow the process to receive SIGIO */
    //fcntl(priv->fd, F_SETOWN, getpid());

    /* after receiving SIGIO, wait_flag = FALSE, input is available
       and can be read */
    while (priv->state >= ChannelIdle && consecutive_errors < 15) {
        priv->state = ChannelIdle;

        /* Initialize the timeout structure */
        timeout.tv_sec  = 0;
        timeout.tv_usec = 20000;

        /* Do the select */
        int n = select(priv->fd+1, &readfs, NULL, NULL, &timeout);

/* See if there was an error */
        if (n < 0) {
            IFLOG(perror("select failed"));
            errors++;
            consecutive_errors++;
        } else if (n == 0) {
            IFLOG( puts("+to") );
            dispatchPromises();
            continue;
        } else {
            // We have input
            if (!FD_ISSET(priv->fd, &readfs))
                continue;   // go back to loop begin, but we could call process() here
        }

        // process data, read from serial
        priv->state = ChannelProcessing;

        IFLOG(if(pbuffer > buffer)  printf("[+= %s]\n", buffer));
        int bytes_read = read(priv->fd, pbuffer, sizeof(buffer) - (pbuffer - buffer));
        char *pb = buffer;
        pbuffer += bytes_read;
        *pbuffer = 0;  // null terminate
        bytes_received += bytes_read;

        IFLOG(printf("<=%s\n", buffer));

        char* p = pb;
        while(*p) {
            if(*p == '*') {
                pb = p;
            }
            else if(*p == '\r') {
                // dispatch packet to destination servo (if we have it)
                LynxPacket packet(pb + 1);
                for(int i=0; i<count; i++) {
                    if(servos[i]->id == packet.id) {
                        servos[i]->dispatch(packet);
                        consecutive_errors = 0;
                        dispatchPromises();
                        break;
                    }
                }

                pb = p + 1;
            }
            p++;
        }

        if(pb > buffer) {
            // move buffer to remove processed packets
            int n = pbuffer - pb;
            if(n > 0) {
                IFLOG(printf("[mv %d:%s]\n", n, pb));
                memmove(buffer, pb, n+1);
                pbuffer = buffer + n;
            } else
                pbuffer = buffer;
        }
    }

    // restore old port settings
    tcsetattr(priv->fd, TCSANOW, &priv->oldtio);

    // close the port
    close(priv->fd);
    priv->fd = 0;

    priv->state = ChannelStopped;
    return priv;
}

void LssPosixChannel::update() {
    // now in update we just check for notifications we need to process on the main thread
}

void LssPosixChannel::transmit(const char* pkt_bytes, int count) {
    IFLOG(printf("=> %s", pkt_bytes));
    bytes_sent += count;
    write(priv->fd, pkt_bytes, count);
}

