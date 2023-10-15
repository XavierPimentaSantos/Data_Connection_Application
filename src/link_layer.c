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
#define TRUE 1
#define FALSE 0

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

#define C_INFO_0 0x00 // C for Information messages with byte 0
#define C_INFO_1 0x40 // C for Information messages with byte 1

#define ESC_SEQUENCE 0x7D // escape sequence for byte stuffing
#define ESC_5d 0x5D // if 0x7D in Data
#define ESC_5e 0x5E // if 0x7E in Data

#define START 0
#define FLAG_1_OK 1
#define A_OK 2
#define A_UA_OK 3
#define INFO_0 4
#define INFO_1 5
#define is_SET 6
#define is_DISC 7
#define C_UA_OK 8
#define READ_INFO 9
#define awaiting_FLAG 10
#define awaiting_FLAG_DISC 11
#define BCC_UA_OK 12
#define got_7d 13
#define STANDBY 14
#define A_DISC_OK 15
#define C_DISC_OK 16
#define BCC_DISC_OK 17
#define MUST_RESEND 18
#define C_R_0 19
#define C_R_1 20

#define SET_MESSAGE [FLAG, A_TC_RR, C_SET, A_TC_RR^C_SET, FLAG]
#define TRANSMITTER_UA_MESSAGE [FLAG, A_RC_TR, C_UA, A_RC_TR^C_UA, FLAG]
#define RECEIVER_UA_MESSAGE [FLAG, A_TC_RR, C_UA, A_TC_RR^C_UA, FLAG]
#define REJECT_0 [FLAG, A_TC_RR, C_REJ0, A_TC_RR^C_REJ0, FLAG]
#define REJECT_1 [FLAG, A_TC_RR, C_REJ1, A_TC_RR^C_REJ1, FLAG]
#define RR_0 [FLAG, A_TC_RR, C_RR0, A_TC_RR^C_RR0, FLAG]
#define RR_1 [FLAG, A_TC_RR, C_RR1, A_TC_RR^C_RR1, FLAG]
#define TRANSMITTER_DISC [FLAG, A_TC_RR, C_DISC, A_TC_RR^C_DISC, FLAG]
#define RECEIVER_DISC [FLAG, A_RC_TR, C_DISC, A_RC_TR^C_DISC, FLAG]

#define type_SET 0
#define type_INFO_0 1
#define type_INFO_1 2
#define type_UA 3
#define type_DISC 4

// isto fica de fora para ser acedido por todas as funções
int fd;
struct termios oldtio; // old port parameters
struct termios newtio; // new port parameters

int timeout_;
int nTries_;

unsigned char buf_*;
int bufSize_;

int alarmEnabled = FALSE;
int alarmCount = 0;

int message_to_send = type_SET;

////////////////////////////////////////////////
// LLOPEN
////////////////////////////////////////////////
int llopen(LinkLayer connectionParameters)
{
    // TODO
    timeout_ = connectionParameters.timeout;
    nTries_ = connectionParameters.nRetransmissions;

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

void send_message(int signal) {
    alarmEnabled = FALSE;
    alarmCount++;

    if(message_to_send==type_SET) {
        int bytes = write(fd, SET_MESSAGE, 5);
        printf("%d bytes written\n", bytes);
    }
    else if(message_to_send==type_UA) {
        int bytes = write(fd, TRANSMITTER_UA_MESSAGE, 5);
        printf("%d bytes written\n", bytes);
    }
    else if(message_to_send==type_DISC) {
        int bytes = write(fd, TRANSMITTER_DISC, 5);
        printf("%d bytes written\n", bytes);
    }
    else if(message_to_send==type_INFO_0) {
        unsigned char BCC2 = 0x00;
        unsigned char buf_to_send[bufSize_ + 6] = {0};
        buf_to_send[0] = FLAG;
        buf_to_send[1] = A_TC_RR;
        buf_to_send[2] = C_INFO_0;
        buf_to_send[3] = A_TC_RR ^ C_INFO_0;
        for(int i = 4; i < bufSize_; i++) {
            buf_to_send[i] = buf_[i-4];
            BCC2 = BCC2 ^ buf_[i-4];
        }
        buf_to_send[bufSize_] = BCC2;
        buf_to_send[bufSize_+1] = FLAG;

        write(fd, buf_to_send, bufSize_ + 6);
    }
    else /*if(message_to_send==type_INFO_1)*/ {
        unsigned char BCC2 = 0x00;
        unsigned char buf_to_send[bufSize_ + 6] = {0};
        buf_to_send[0] = FLAG;
        buf_to_send[1] = A_TC_RR;
        buf_to_send[2] = C_INFO_1;
        buf_to_send[3] = A_TC_RR ^ C_INFO_1;
        for(int i = 4; i < bufSize_; i++) {
            buf_to_send[i] = buf_[i-4];
            BCC2 = BCC2 ^ buf_[i-4];
        }
        buf_to_send[bufSize_] = BCC2;
        buf_to_send[bufSize_+1] = FLAG;

        write(fd, buf_to_send, bufSize_ + 6);
    }
}

////////////////////////////////////////////////
// LLWRITE
////////////////////////////////////////////////
int llwrite(const unsigned char *buf, int bufSize)
{
    // TODO
    unsigned char STOP = FALSE;
    buf_ = buf;
    bufSize_ = bufSize;
    unsigned char receiver_message[1] = {0};
    unsigned char state = START;
    unsigned char DISC = FALSE;

    (void) signal(SIGALRM, send_message);

    while(alarmCount < nTries_ && STOP==FALSE) {
        if (alarmEnabled == FALSE)
        {
            alarm(timeout_); // Set alarm to be triggered in timeout_ seconds
            alarmEnabled = TRUE;
        }

        while (STOP==FALSE)
        {
            int byte = read(fd, receiver_message, 1);
            if(byte <= 0) continue;
            // IMPLEMENT STATE MACHINE HERE
            switch(state) {
                case START:
                    if(receiver_message[0]==FLAG) {
                        state = FLAG_1_OK;
                    }
                    break;
                case FLAG_1_OK:
                    if(receiver_message[0]==FLAG) {
                        state = FLAG_1_OK;
                    }
                    else if(receiver_message[0]==A_TC_RR) {
                        state = A_OK;
                    }
                    else if(receiver_message[0]==A_RC_TR && DISC==TRUE) {
                        state = A_DISC_OK;
                    }
                    else {
                        state = START;
                    }
                    break;
                case A_OK:
                    if(receiver_message[0]==C_UA) {
                        state = C_UA_OK;
                    }
                    else if(receiver_message[0]==C_REJ0 && message_to_send==type_INFO_1) {
                        state = C_;
                    }
                    
            }

            // Returns after 5 chars have been input
            int bytes = read(fd, receiver_message, 5);
            ua[bytes] = '\0'; // Set end of string to '\0', so we can printf

            printf("received UA = ");
            for(int i = 0; i < BUF_SIZE; i++) {
                printf("%02X", ua[i]);
            }
            printf("\n");
            printf(":%s:%d\n", ua, bytes);

            if (ua[BUF_SIZE] == '\0') {
                STOP = TRUE;
                can_read_UA = TRUE;
                alarm(0);
                printf("succesfully received UA\n");
            }
        }
    }
    //em tramas I, cria o BCC2 com os 1024 bytes D, e depois é que aplica stuffing
    return 0;
}

////////////////////////////////////////////////
// LLREAD
////////////////////////////////////////////////
int llread(unsigned char *packet)
{
    // TODO
    unsigned char STOP = FALSE;
    unsigned char BCC2_calc = 0x00;
    unsigned char state = START;
    unsigned char buf[1] = {0};
    unsigned char received_size = 0;
    unsigned char index = 0;
    unsigned char DISC = FALSE;
    unsigned char SET = FALSE;
    unsigned char can_receive_bit = 0x00;
    unsigned char standby_byte;

    while(STOP == FALSE) {
        int byte = read(fd, buf, 1);
        if(byte <= 0) continue;

        switch(state) {
            case START:
                if(buf[0]==FLAG) {
                    state = FLAG_1_OK;
                }
                break;
            
            case FLAG_1_OK:
                if(buf[0]==FLAG) {
                    state = FLAG_1_OK;
                }
                else if(buf[0]==A_TC_RR) {
                    state = A_OK;
                }
                else if(DISC==TRUE && buf[0]==A_RC_TR) {
                    state = A_UA_OK;
                }
                else {
                    state = START;
                }
                break;
            case A_OK:
                if(buf[0]==FLAG) {
                    state = FLAG_1_OK;
                }
                else if(SET==TRUE && (buf[0]==C_INFO_0 && can_receive_bit==C_INFO_0)) {
                    state = INFO_0;
                }
                else if(SET==TRUE && (buf[0]==C_INFO_1 && can_receive_bit==C_INFO_1)) {
                    state = INFO_1;
                }
                else if(buf[0]==C_SET) {
                    state = is_SET;
                }
                else if(buf[0]==C_DISC) {
                    state = is_DISC;
                }
                else {
                    state = START;
                }
                break;
            case A_UA_OK:
                if(buf[0]==FLAG) {
                    state = FLAG_1_OK;
                }
                else if(buf[0]==C_UA) {
                    state = C_UA_OK;
                }
                else {
                    state = START;
                }
                break;
            case INFO_0:
                if(buf[0]==FLAG) {
                    state = FLAG_1_OK;
                }
                else if(buf[0]==0x03) {
                    state = READ_INFO;
                }
                else {
                    state = START;
                }
                break;
            case INFO_1:
                if(buf[0]==FLAG) {
                    state = FLAG_1_OK;
                }
                else if(buf[0]==0x43) {
                    state = READ_INFO;
                }
                else {
                    state = START;
                }
                break;
            case is_SET:
                if(buf[0]==FLAG) {
                    state = FLAG_1_OK;
                }
                else if(buf[0]=0x00) {
                    state = awaiting_FLAG;
                }
                else {
                    state = START;
                }
                break;
            case is_DISC:
                if(buf[0]==FLAG) {
                    state = FLAG_1_OK;
                }
                else if(buf[0]==0x08) {
                    state = awaiting_FLAG_DISC;
                }
                else {
                    state = START;
                }
                break;
            case C_UA_OK:
                if(buf[0]==FLAG) {
                    state = FLAG_1_OK;
                }
                else if(buf[0]==0x06) {
                    state = BCC_UA_OK;
                }
                else {
                    state = START;
                }
                break;
            case READ_INFO:
                if(buf[0]==BCC2_calc) {
                    state = STANDBY;
                    standby_byte = buf[0];
                }
                else if(buf[0]==FLAG) {
                    state = START;
                    index = 0;
                    if(can_receive_bit==0x00) {
                        write(fd, REJECT_0, 5);
                        printf("there was an error in the message; requesting retransmission\n");
                    }
                    else {
                        write(fd, REJECT_1, 5);
                        printf("there was an error in the message; requesting retransmission\n");
                    }
                }
                else if(buf[0]==ESC_SEQUENCE) {
                    state = got_7d;
                }
                else {
                    state = READ_INFO;
                    packet[index] = buf[0];
                    index++;
                    BCC2_calc = BCC2_calc ^ buf[0];
                }
                break;
            case got_7d:
                if(buf[0]==ESC_5d) {
                    if(BCC2_calc == 0x7D) {
                        state = STANDBY; // BCC2 could be 0x7D, in which case we need to check for  0x7d 0x5d 0x7e
                        standby_byte = ESC_5d;
                    }
                    else {
                       state = READ_INFO;
                        packet[index] = 0x7D;
                        index++;
                        BCC2_calc = BCC2_calc ^ 0x7D; 
                    }   
                }
                else if(buf[0]==ESC_5e) {
                    if(BCC2_calc == 0x7E) {
                        state = STANDBY; // BCC2 could be 0x7E, in which case we need to check for 0x7d 0x5e 0x7e
                        standby_byte = ESC_5e;
                    }
                    else {
                        state = READ_INFO;
                        packet[index] = 0x7E;
                        index++;
                        BCC2_calc = BCC2_calc ^ 0x7E;
                    }                    
                }
                else {
                    STOP = TRUE;
                    state = START;
                    index = 0;
                    if(can_receive_bit==0x00) {
                        write(fd, REJECT_0, 5);
                        printf("there was an error in the message; requesting retransmission\n");
                    }
                    else {
                        write(fd, REJECT_1, 5);
                        printf("there was an error in the message; requesting retransmission\n");
                    }
                    // need to reject message and wait for new one
                }
            case STANDBY:
                if(standby_byte==ESC_5d) {
                    if(buf[0]==FLAG) {
                        can_receive_bit = can_receive_bit ^ 0x40;
                        // BCC2 was 0x7D, and there was nothing wrong with the message; we can send RR
                        if(can_receive_bit==0x40) {
                            write(fd, RR_0, 5);
                        }
                        else {
                            write(fd, RR_1, 5);
                        }
                        state = START;
                    }
                    else { // didn't receive F, which means 0x7D was NOT the BCC2 byte, and therefore we have to send 0x7D to the packet
                        packet[index] = 0x7D;
                        index++;
                        if(buf[0]==ESC_SEQUENCE) {
                            state = got_7d;
                            BCC2_calc = 0x00;
                        }
                        else {
                            packet[index] = buf[0];
                            BCC2_calc = buf[0];
                            index++;
                            state = READ_INFO;
                        }
                    }
                }
                else if(standby_byte==ESC_5e) {
                    if(buf[0]==FLAG) {
                        can_receive_bit = can_receive_bit ^ 0x40;
                        // BCC2 was 0x7E, and there was nothing wrong with the message; we can send RR and return
                        if(can_receive_bit==0x40) {
                            write(fd, RR_0, 5);
                        }
                        else {
                            write(fd, RR_1, 5);
                        }
                        state = START;
                    }
                    else { // didn't receive F, which means 0x7E was NOT the BCC2 byte, and therefore we have to send 0x7E to the packet
                        packet[index] = 0x7E;
                        index++;
                        if(buf[0]==ESC_SEQUENCE) {
                            state = got_7d;
                            BCC2_calc = 0x00;
                        }
                        else {
                            packet[index] = buf[0];
                            BCC2_calc = buf[0];
                            index++;
                            state = READ_INFO;
                        }
                    }
                }
                else if(buf[0]==FLAG) {
                    can_receive_bit = can_receive_bit ^ 0x40;
                    // BCC2 checks off as true, we received the flag, can send RR message and return
                    if(can_receive_bit==0x40) {
                        write(fd, RR_0, 5);
                    }
                    else {
                        write(fd, RR_1, 5);
                    }
                    state = START;
                }
                else if(buf[0]==0x00) { // BCC2 may be 0x00, so we must check for that
                    state = STANDBY;
                    BCC2_calc = 0x00;
                }
                else {
                    state = READ_INFO;
                    packet[index] = standby_byte
                }
                break;
            case awaiting_FLAG:
                if(buf[0]==FLAG) {
                    SET = TRUE;
                    state = START;
                }
                else {
                    state = START;
                }
                break;
            case awaiting_FLAG_DISC:
                if(buf[0]==FLAG) {
                    DISC = TRUE;
                    //send its own DISC message
                    write(fd, RECEIVER_DISC, 5);
                    state = START;
                }
                else {
                    state = START;
                }
                break;
            default:
                break;
        }
    }

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
