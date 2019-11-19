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
        : LssChannelBase(channel_name), priv(nullptr)
{}

LssPosixChannel::~LssPosixChannel() {
    if(priv != nullptr)
        free();
}

void LssPosixChannel::begin(const char* devname, int baudrate)
{
    free();     // close and will reopen

    pbuffer = buffer;

    /* open the device to be non-blocking (read will return immediatly) */
    posix_serial_private* prv = new posix_serial_private();
    prv->fd = open(devname, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (prv->fd <0) {
        perror(devname);
        goto error;
    }

    /* allow the process to receive SIGIO */
    fcntl(prv->fd, F_SETOWN, getpid());

    /* Make the file descriptor asynchronous (the manual page says only
       O_APPEND and O_NONBLOCK, will work with F_SETFL...) */
    fcntl(prv->fd, F_SETFL, FASYNC);

    tcgetattr(prv->fd,&prv->oldtio); /* save current port settings */
    prv->newtio.c_cflag = BAUDRATE | CS8 | CLOCAL | CREAD; // | CRTSCTS
    prv->newtio.c_iflag = IGNPAR;
    prv->newtio.c_oflag = 0;
    prv->newtio.c_lflag = 0;
    prv->newtio.c_cc[VMIN]=1;
    prv->newtio.c_cc[VTIME]=0;
    tcflush(prv->fd, TCIFLUSH);
    tcsetattr(prv->fd,TCSANOW,&prv->newtio);


    // start the processing loop
    prv->state = ChannelStarting;
    if(pthread_create( &prv->loop, NULL, s_run, (void*) this) !=0) {
        // todo: should probably print some kind of error here
        printf("failed to start serial processing thread\n");
        goto error;
    }

    priv = prv;
    return;
error:
    if(prv->fd >0)
        close(prv->fd);
    delete prv;
}

void LssPosixChannel::free() {
    if(priv != nullptr) {
        // restore old port settings
        tcsetattr(priv->fd, TCSANOW, &priv->oldtio);

        // close the port
        close(priv->fd);

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
    priv->state = ChannelIdle;

    fd_set         input;
    struct timeval timeout;
    int errors = 0;
    int consecutive_errors = 0;

    /* Initialize the input set */
    FD_ZERO(&input);
    FD_SET(priv->fd, &input);

    /* Initialize the timeout structure */
    timeout.tv_sec  = 1;
    timeout.tv_usec = 0;

    /* after receiving SIGIO, wait_flag = FALSE, input is available
       and can be read */
    while (priv->state >= ChannelIdle && consecutive_errors < 15) {
        priv->state = ChannelIdle;

        /* Do the select */
        int n = select(priv->fd+1, &input, NULL, NULL, &timeout);

/* See if there was an error */
        if (n < 0) {
            perror("select failed");
            errors++;
            consecutive_errors++;
        } else if (n == 0) {
            puts("+to");
        } else {
            // We have input
            if (!FD_ISSET(priv->fd, &input))
                continue;   // go back to loop begin, but we could call process() here
        }

        // process data, read from serial
        priv->state = ChannelProcessing;

        IFLOG(if(pbuffer > buffer)  printf("[+= %s]\n", buffer));
        int bytes_read = read(priv->fd, pbuffer, sizeof(buffer) - (pbuffer - buffer));
        char *pb = buffer;
        pbuffer += bytes_read;
        *pbuffer = 0;  // null terminate

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

    priv->state = ChannelStopped;
    return priv;
}

void LssPosixChannel::update() {
    // now in update we just check for notifications we need to process on the main thread
}

void LssPosixChannel::transmit(const char* pkt_bytes, int count) {
    IFLOG(printf("=> %s", pkt_bytes));
    write(priv->fd, pkt_bytes, count);
}

