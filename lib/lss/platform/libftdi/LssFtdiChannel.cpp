#include "LssFtdiChannel.h"
#include "../../LssChannel.h"

#include <termios.h>
#include <stdio.h>
#include <unistd.h>
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

#include <ftdi.h>

#define BAUDRATE B115200
#define _POSIX_SOURCE 1 /* POSIX compliant source */


#if 0
#define IFLOG(x) x
#else
#define IFLOG(x)
#endif

//#define LOG_ACTIONS

class ftdi_serial_private {
public:
    int vid;        // typically 0x403
    int pid;
    int baudrate;
    ftdi_interface interface;

    // processing loop variables
    struct ftdi_context *ftdi;
    pthread_t loop;
    ChannelState state; // current loop task/state

    union {
        int client_service[2];
        struct {
            int client;
            int service;
        };
    } notify;

    inline ftdi_serial_private()
        : vid(0x403), pid(0x6001), baudrate(115200), interface(INTERFACE_ANY), ftdi(nullptr), loop(0), state(ChannelStopped)
    {
        notify.client = 0;
        notify.service = 0;
    }
};


LssFtdiChannel::LssFtdiChannel(LssChannel* channel)
        : LssChannelDriver(channel), priv(nullptr), baudrate(115200)
{}

LssFtdiChannel::~LssFtdiChannel() {
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

ChannelDriverError LssFtdiChannel::begin(const char* _devname, int _baudrate)
{
    if(priv !=nullptr)
        return DriverAlreadyInitialized;

    baudrate = _baudrate;

    statistics = Statistics();

    priv = new ftdi_serial_private();

    int pid = 0x403, vid = 0x6001;
    char _interface = 0;
    sscanf(_devname, "%x:%x:%c", &pid, &vid, &_interface);
    switch(toupper(_interface)) {
        case 'A': priv->interface = INTERFACE_A; break;
        case 'B': priv->interface = INTERFACE_B; break;
        case 'C': priv->interface = INTERFACE_C; break;
        case 'D': priv->interface = INTERFACE_D; break;
        default: priv->interface = INTERFACE_ANY; break;
    }

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

void processSignal(int s) {
}

void* LssFtdiChannel::s_run(void* inst) {
    return ((LssFtdiChannel*)inst)->run();
}

void* LssFtdiChannel::run()
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
    unsigned char buffer[256];
    unsigned char* pbuffer = buffer;

    //LssTransaction* ctx = nullptr;

reopen:
    /* open the device to be non-blocking (read will return immediately) */
    priv->ftdi = ftdi_new();

    if(priv->ftdi !=NULL) {
        if (!priv->vid && !priv->pid && (priv->interface == INTERFACE_ANY))
        {
            ftdi_set_interface(priv->ftdi, INTERFACE_ANY);
            struct ftdi_device_list *devlist;
            int rv;
            if ((rv = ftdi_usb_find_all(priv->ftdi, &devlist, 0, 0)) < 0)
            {
                fprintf(stderr, "No FTDI with default VID/PID found\n");
                goto exit_serial_processing;
            }
            if (rv == 1)
            {
                res = ftdi_usb_open_dev(priv->ftdi,  devlist[0].dev);
                if (res<0)
                {
                    fprintf(stderr, "Unable to open device %d: (%s)",
                            res, ftdi_get_error_string(priv->ftdi));
                }
            }
            ftdi_list_free(&devlist);
            if (rv > 1)
            {
                fprintf(stderr, "%d Devices found, please select Device with VID/PID\n", res);
                /* TODO: List Devices*/
                goto exit_serial_processing;
            }
            if (rv == 0)
            {
                fprintf(stderr, "No Devices found with default VID/PID\n");
                goto exit_serial_processing;
            }
        }
        else
        {
            // Select interface
            ftdi_set_interface(priv->ftdi, priv->interface);

            // Open device
            res = ftdi_usb_open(priv->ftdi, priv->vid, priv->pid);
        }
        if (res < 0)
        {
            fprintf(stderr, "unable to open ftdi device: %d (%s)\n", res, ftdi_get_error_string(priv->ftdi));
            exit(-1);
        }

        // Set baudrate
        res = ftdi_set_baudrate(priv->ftdi, priv->baudrate);
        if (res < 0)
        {
            fprintf(stderr, "unable to set baudrate: %d (%s)\n", res, ftdi_get_error_string(priv->ftdi));
            exit(-1);
        }

        res = ftdi_set_line_property(priv->ftdi, BITS_8, STOP_BIT_1, NONE);
        if (res < 0)
        {
            fprintf(stderr, "unable to set line parameters: %d (%s)\n", res, ftdi_get_error_string(priv->ftdi));
            exit(-1);
        }

        res = ftdi_set_latency_timer(priv->ftdi, 1);
        if (res < 0)
        {
            fprintf(stderr, "unable to set latency timer: %d (%s)\n", res, ftdi_get_error_string(priv->ftdi));
            exit(-1);
        }

        res = ftdi_set_event_char(priv->ftdi, '\r', 1);
        if (res < 0)
        {
            fprintf(stderr, "unable to set event char: %d (%s)\n", res, ftdi_get_error_string(priv->ftdi));
            exit(-1);
        }

        res = ftdi_write_data_set_chunksize(priv->ftdi, 4096);
        if (res < 0)
        {
            fprintf(stderr, "unable to set write chunk size: %d (%s)\n", res, ftdi_get_error_string(priv->ftdi));
            exit(-1);
        }

        res = ftdi_read_data_set_chunksize(priv->ftdi, 64);
        if (res < 0)
        {
            fprintf(stderr, "unable to set read chunk size: %d (%s)\n", res, ftdi_get_error_string(priv->ftdi));
            exit(-1);
        }
    }


    if (priv->ftdi ==NULL) {
        printf("failed to initialize ftdi driver\n");
        errors++;
        consecutive_errors++;
        if(consecutive_errors > 300)
            goto exit_serial_processing;
        sleep(1);
        goto reopen;
    }

    // todo: generate a callback or promise here so the app can do some startup init anytime the port is reopened

    // setup the client/service notification domain socket
    if(socketpair(AF_LOCAL, SOCK_STREAM | SOCK_NONBLOCK, 0, priv->notify.client_service) == -1) {
        perror("failed to create domain socket for posix serial processing");
        goto exit_serial_processing;
    }

    priv->state = ChannelIdle;

    printf("Using FTDI channel\n");

    /* after receiving SIGIO, wait_flag = FALSE, input is available
       and can be read */
    while (priv->state >= ChannelIdle && consecutive_errors < 15) {
        priv->state = ChannelIdle;

        int bytes_read = ftdi_read_data(priv->ftdi, pbuffer, sizeof(buffer) - (pbuffer - buffer));
        if(bytes_read == -666) {
            // usb device unavailable
            printf("usb device unavailable\n");
            ftdi_free(priv->ftdi);
            priv->ftdi = nullptr;
            goto exit_serial_processing;
            continue;
        } else if(bytes_read <0) {
            // error code from usb_bulk_read
            printf("bulk read error\n");
            goto exit_serial_processing;
        } else if(bytes_read ==0) {
            // no data
            channel->driverIdle();
            continue;
        }

        // process data, read from serial
        priv->state = ChannelProcessing;

        unsigned char *pb = buffer;
        pbuffer += bytes_read;
        *pbuffer = 0;  // null terminate
        statistics.bytes_received += bytes_read;

        unsigned char* p = pb;
        while(*p) {
            if(*p == '*') {
                pb = p;
            }
            else if(*p == '\r') {
                IFLOG(printf("<=%s\n", pb));

                // dispatch packet to destination servo (if we have it)
                LynxPacket packet( (const char*)(pb + 1));
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

        channel->driverIdle();
    }

exit_serial_processing:
    printf("exiting ftdi serial processing\n");
    priv->state = ChannelStopping;
    if(priv->ftdi !=nullptr) {
        ftdi_usb_close(priv->ftdi);
        ftdi_free(priv->ftdi);
    }

    // restore old port settings
    if(restore_tio)
        tcsetattr(fd, TCSANOW, &oldtio);

    if(priv->notify.service)
        close(priv->notify.service);
    if(priv->notify.client)
        close(priv->notify.client);
    priv->notify.client = priv->notify.service = 0;

    priv->state = ChannelError;
    return nullptr;
}

intptr_t LssFtdiChannel::signal(ChannelDriverSignal signal, unsigned long a, const void* ptr)
{
    // we send a simple character to awaken the processing thread
    switch(signal) {
        case OpenSignal:
            begin( (const char*)ptr, a);
            break;

        case TransactionSignal:
        case DataSignal:
            channel->driverIdle();
            //write(priv->notify.client, "*", 1);
            break;

        case TransmitSignal:
            if(ptr && a>0)
                transmit((const char*)ptr, a);
            break;
    }
    return 0;
}

void LssFtdiChannel::transmit(const char* pkt_bytes, int count) {
    if(priv->state >= ChannelIdle) {
        if(count<0)
            count = strlen(pkt_bytes);
        IFLOG(printf("=>%s", pkt_bytes));
        statistics.bytes_sent += count;
        if(priv->ftdi) {
            ftdi_write_data(priv->ftdi, (unsigned char*)pkt_bytes, count);
        }
#ifdef LOG_ACTIONS
        printf("=> %s\n", pkt_bytes);
#endif
    }
}

