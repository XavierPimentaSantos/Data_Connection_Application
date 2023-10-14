// Link layer protocol implementation

#include "link_layer.h"

#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <termios.h>
#include <unistd.h>
#include <signal.h>

// MISC
#define _POSIX_SOURCE 1 // POSIX compliant source

// macros
#define FLAG 0x7E

#define A_TC_RR 0x03 // A for Transmitter commands or Receiver replies
#define A_RC_TR 0x01 // A for Receiver comands or Transmitter replies

#define C_SET 0x03 // C for SET messages
#define C_UA 0x07 // C for UA messages
#define C_RR0 0x05 // C for Receiver to signal it is ready to receive 0-type data message
#define C_RR1 0x85 // C for Receiver to signal it is ready to receive 1-type data message
#define C_REJ0 0x01 // C for Receiver to signal it rejects the last 0-type data message
#define C_REJ1 0x81 // C for Receiver to signal it rejects the last 1-type data message
#define C_DISC 0x0B // C for DISC messages

#define ESC_SEQUENCE 0x7D // escape sequence for byte stuffing
#define ESC_5d 0x5D // if 0x7D in Data
#define ESC_5e 0x5E // if 0x7E in Data

// isto fica de fora para ser acedido por todas as funções
int fd;
struct termios oldtio; // old port parameters
struct termios newtio; // new port parameters

////////////////////////////////////////////////
// LLOPEN
////////////////////////////////////////////////
int llopen(LinkLayer connectionParameters)
{
    // TODO
    fd = open(connectionParameters.serialPort, O_RDWR | O_NOCTTY);
    if(fd < 0) {
        perror(connectionParameters.serialPort);
        exit(-1);
    }

    if(tcgetattr(fd, &oldtio) == -1) {
        perror("tcgetattr");
        exit(-1);
    }

    // Clear struct for new port settings
    memset(&newtio, 0, sizeof(newtio));

    newtio.c_cflag = connectionParameters.baudRate | CS8 | CLOCAL | CREAD;
    newtio.c_iflag = IGNPAR;
    newtio.c_oflag = 0;

    // Set input mode (non-canonical, no echo,...)
    newtio.c_lflag = 0;
    newtio.c_cc[VTIME] = 0; // value subject to change
    newtio.c_cc[VMIN] = 5;  // value subject to change

    // Now clean the line and activate the settings for the port
    // tcflush() discards data written to the object referred to
    // by fd but not transmitted, or data received but not read,
    // depending on the value of queue_selector:
    //   TCIFLUSH - flushes data received but not read.
    tcflush(fd, TCIOFLUSH);

    // Set new port settings
    if (tcsetattr(fd, TCSANOW, &newtio) == -1)
    {
        perror("tcsetattr");
        exit(-1);
    }

    printf("New termios structure set\n");

    return 1;
}

////////////////////////////////////////////////
// LLWRITE
////////////////////////////////////////////////
int llwrite(const unsigned char *buf, int bufSize)
{
    // TODO
    //em tramas I, cria o BCC2 com os 1024 bytes D, e depois é que aplica stuffing
    return 0;
}

////////////////////////////////////////////////
// LLREAD
////////////////////////////////////////////////
int llread(unsigned char *packet)
{
    // TODO

    return 0;
}

////////////////////////////////////////////////
// LLCLOSE
////////////////////////////////////////////////
int llclose(int showStatistics)
{
    // TODO
    // Restore the old port settings
    if (tcsetattr(fd, TCSANOW, &oldtio) == -1)
    {
        perror("tcsetattr");
        exit(-1);
    }

    close(fd);

    return 1;
}
