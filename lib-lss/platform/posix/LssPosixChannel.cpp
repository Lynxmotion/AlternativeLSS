#include "LssPosixChannel.h"
#include "../../LynxmotionLSS.h"

#include <termios.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/signal.h>
#include <sys/types.h>

#include <functional>

#define BAUDRATE B115200
#define _POSIX_SOURCE 1 /* POSIX compliant source */

#define FALSE 0
#define TRUE 1

#if 0
#define IFLOG(x) x
#else
#define IFLOG(x)
#endif

class posix_serial_private {
public:
    // open and configure the posix serial port
    int fd;
    struct termios oldtio, newtio;
    struct sigaction saio;           /* definition of signal action */

    inline posix_serial_private()
        : fd(0)
    {}
};

//void signal_handler_IO (int status);   /* definition of signal handler */
int wait_flag=TRUE;                    /* TRUE while no signal received */


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
        return;
    }

    /* install the signal handler before making the device asynchronous */
    //priv->saio.sa_handler = std::bind(&LssPosixChannel::signal_handler_IO, this, std::placeholders::_1);
    //priv->saio.sa_handler = [this](int status) -> void { wait_flag = FALSE; };
    prv->saio.sa_handler = signal_handler_IO;
    sigemptyset(&prv->saio.sa_mask);
    prv->saio.sa_flags = 0;
    prv->saio.sa_restorer = NULL;
    if(sigaction(SIGIO,&prv->saio,NULL) <0) {
        perror(devname);
        return;
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

    priv = prv;
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

void LssPosixChannel::update()
{
    if(priv == nullptr) return;

    /* after receiving SIGIO, wait_flag = FALSE, input is available
       and can be read */
    if (wait_flag==FALSE) {
        wait_flag = TRUE;      /* wait for new input */
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
}

void LssPosixChannel::transmit(const char* pkt_bytes, int count) {
    IFLOG(printf("=> %s", pkt_bytes));
    write(priv->fd, pkt_bytes, count);
}


/***************************************************************************
* signal handler. sets wait_flag to FALSE, to indicate above loop that     *
* characters have been received.                                           *
***************************************************************************/

void LssPosixChannel::signal_handler_IO(int status)
{
    wait_flag = FALSE;
}