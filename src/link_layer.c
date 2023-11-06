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

// llread() state machine macros
#define state_START 0
#define state_FLAG_1_OK 1
#define state_A_OK 2
#define state_A_UA_OK 3
#define state_INFO_0 4
#define state_INFO_1 5
#define state_is_SET 6
#define state_is_DISC 7
#define state_C_UA_OK 8
#define state_READ_INFO 9
#define state_awaiting_FLAG 10
#define state_awaiting_FLAG_DISC 11
#define state_BCC_UA_OK 12
#define state_GOT_7D 13
#define state_STANDBY 14
#define state_A_DISC_OK 15
#define state_C_DISC_OK 16
#define state_BCC_DISC_OK 17
#define state_MUST_RESEND 18
#define state_C_REJ_0 19
#define state_C_REJ_1 20
#define state_C_RR_0 21
#define state_C_RR_1 22
#define state_FINISH 23

#define type_SET 0
#define type_INFO_0 1
#define type_INFO_1 2

// static messages
unsigned char SET_MESSAGE[5] = {0x7E, 0x03, 0x03, 0x00, 0x7E};
unsigned char TRANSMITTER_UA_MESSAGE[5] = {0x7E, 0x01, 0x07, 0x06, 0x7E};
unsigned char RECEIVER_UA_MESSAGE[5] = {0x7E, 0x03, 0x07, 0x04, 0x7E};
unsigned char REJECT_0[5] = {0x7E, 0x03, 0x01, 0x02, 0x7E};
unsigned char REJECT_1[5] = {0x7E, 0x03, 0x81, 0x82, 0x7E};
unsigned char RR_0[5] = {0x7E, 0x03, 0x05, 0x02, 0x7E};
unsigned char RR_1[5] = {0x7E, 0x03, 0x85, 0x82, 0x7E};
unsigned char TRANSMITTER_DISC[5] = {0x7E, 0x03, 0x0B, 0x08, 0x7E};
unsigned char RECEIVER_DISC[5] = {0x7E, 0x01, 0x0B, 0x0A, 0x7E};

// Connection-related variables
int fd; // port
struct termios oldtio; // old port parameters
struct termios newtio; // new port parameters
LinkLayer connectionParameters_;

// Alarm variables
int timeout_;
int nTries_;
int alarmEnabled = FALSE;
int alarmCount = 0;

// used by llwrite to hold the data sent by applicationLayer
unsigned char *buf_[3000]; // 3000 = 3 * MAX_PAYLOAD_SIZE (plenty of space for whichever message it has to send)
int bufSize_ = 0;

int message_to_send = type_SET; // defines the type of message to be sent by Transmitter

unsigned char buf_to_send[3000] = {0}; // 3 * MAX_PAYLOAD_SIZE, more than enough space

// statistics
int packets_sent = 0;
int bit_rate;
int packets_received = 0;

void send_message(int signal) {
    alarmEnabled = FALSE;
    alarmCount++;
    packets_sent++;

    if(message_to_send==type_SET) {
        (void) write(fd, SET_MESSAGE, 5);
    }
    else {
        unsigned char BCC2 = 0x00;
        int actual_bufSize_ = bufSize_ + 4;
        buf_to_send[0] = 0x7E;
        buf_to_send[1] = 0x03;
        if(message_to_send==type_INFO_0) {
            buf_to_send[2] = 0x00;
            buf_to_send[3] = 0x03;
        }
        else {
            buf_to_send[2] = 0x40;
            buf_to_send[3] = 0x43;
        }

        for(int i = 0, i_helper = 0; i < bufSize_; i++) {
            unsigned char read_byte = (unsigned char) buf_[i];
            BCC2 ^= read_byte;
            if(read_byte==0x7E) {
                buf_to_send[4 + i + i_helper] = 0x7D;
                i_helper += 1;
                buf_to_send[4 + i + i_helper] = 0x5E;
                actual_bufSize_ += 1;
            }
            else if(read_byte==0x7D) {
                buf_to_send[4 + i + i_helper] = 0x7D;
                i_helper += 1;
                buf_to_send[4 + i + i_helper] = 0x5D;
                actual_bufSize_ += 1;
            }
            else {
                buf_to_send[4 + i + i_helper] = read_byte;
            }
        }

        buf_to_send[actual_bufSize_++] = BCC2;
        buf_to_send[actual_bufSize_++] = 0x7E;
	
        (void) write(fd, buf_to_send, actual_bufSize_);
    }
}

////////////////////////////////////////////////
// LLOPEN
////////////////////////////////////////////////
int llopen(LinkLayer connectionParameters)
{
    connectionParameters_ = connectionParameters;
    timeout_ = connectionParameters.timeout;
    nTries_ = connectionParameters.nRetransmissions;
    
    bit_rate = connectionParameters.baudRate;

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
    newtio.c_cc[VMIN] = 0;  // value subject to change

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

    // int open_STOP = FALSE;
    unsigned char open_buf[1] = {0};
    int open_state = 0;

    if(connectionParameters.role == LlTx) {
        message_to_send = type_SET;
        timeout_ = connectionParameters.timeout;
        nTries_ = connectionParameters.nRetransmissions;
        (void) signal(SIGALRM, send_message);

        while(alarmCount < nTries_ /* && open_STOP == FALSE */) {
            if(alarmEnabled == FALSE) {
                alarm(timeout_);
                alarmEnabled = TRUE;
            }

            int byte = read(fd, open_buf, 1);
            if(byte <= 0) continue;

            switch(open_state) {
                case 0: // START
                    if(open_buf[0]==0x7E) {
                        open_state = 1;
                    }
                    else {
                        open_state = 0;
                    }
                    break;
                case 1: // READ FLAG
                    if(open_buf[0]==0x7E) {
                        open_state = 1;
                    }
                    else if(open_buf[0]==0x03) {
                        open_state = 2;
                    }
                    else {
                        open_state = 0;
                    }
                    break;
                case 2: // READ A
                    if(open_buf[0]==0x7E) {
                        open_state = 1;
                    }
                    else if(open_buf[0]==0x07) {
                        open_state = 3;
                    }
                    else {
                        open_state = 0;
                    }
                    break;
                case 3: // READ C
                    if(open_buf[0]==0x7E) {
                        open_state = 1;
                    }
                    else if(open_buf[0]==0x04) {
                        open_state = 4;
                    }
                    else {
                        open_state = 0;
                    }
                    break;
                case 4: // READ BCC
                    if(open_buf[0]==0x7E) {
                        alarm(0);
                        alarmCount = 0;
                        alarmEnabled = FALSE;
                        message_to_send = type_INFO_0;
                        return 1;
                    }
                    else {
                        open_state = 0;
                    }
                    break;
                default:
                    break;
                
            }
        }
        alarm(0);
        alarmCount = 0;
        alarmEnabled = FALSE;
        return -1;
    }

    else {
        while(TRUE) {
            int byte = read(fd, open_buf, 1);
            if(byte <= 0) continue;

            switch(open_state) {
                case 0: // START
                    if(open_buf[0]==0x7E) {
                        open_state = 1;
                    }
                    else {
                        open_state = 0;
                    }
                    break;
                case 1: // RECEIVED FLAG
                    if(open_buf[0]==0x7E) {
                        open_state = 1;
                    }
                    else if(open_buf[0]==0x03) {
                        open_state = 2;
                    }
                    else {
                        open_state = 0;
                    }
                    break;
                case 2: // RECEIVED A
                    if(open_buf[0]==0x7E) {
                        open_state = 1;
                    }
                    else if(open_buf[0]==0x03) {
                        open_state = 3;
                    }
                    else {
                        open_state = 0;
                    }
                    break;
                case 3: // RECEIVED C
                    if(open_buf[0]==0x7E) {
                        open_state = 1;
                    }
                    else if(open_buf[0]==0x00) {
                        open_state = 4;
                    }
                    else {
                        open_state = 0;
                    }
                    break;
                case 4: // RECEIVED BCC
                    if(open_buf[0]==0x7E) {
                        write(fd, RECEIVER_UA_MESSAGE, 5);
                        packets_received++;
                        return 1;
                    }
                    else {
                        open_state = 0;
                    }
                    break;
            }
        }
        // return 1;
    }

    return -1;
}



////////////////////////////////////////////////
// LLWRITE
////////////////////////////////////////////////
int llwrite(const unsigned char *buf, int bufSize)
{
    unsigned char receiver_message[1] = {0};
    unsigned char state = state_START;

    for(int n = 0; n < bufSize; n++) {
    	buf_[n] = buf[n];
  	}
    
  	bufSize_ = bufSize;
    
    (void)signal(SIGALRM, send_message);

    while(alarmCount < nTries_) {
    	if(alarmEnabled == FALSE) {
            alarm(timeout_);
            alarmEnabled = TRUE;
        }
	
        int byte = read(fd, receiver_message, 1);
        if(byte <= 0) continue;

        switch(state) {
            case state_START:
                if(receiver_message[0]==0x7E) {
                    state = state_FLAG_1_OK;
                }
                else {
                    state = state_START;
                }
                break;
            case state_FLAG_1_OK:
                if(receiver_message[0]==0x7E) {
                    state = state_FLAG_1_OK;
                }
                else if(receiver_message[0]==0x03) {
                    state = state_A_OK;
                }
                else {
                    state = state_START;
                }
                break;
            case state_A_OK:
                if(receiver_message[0]==0x7E) {
                    state = state_FLAG_1_OK;
                }
                else if(receiver_message[0]==0x01 && message_to_send==type_INFO_0) {
                    state = state_C_REJ_0;
                }
                else if(receiver_message[0]==0x81 && message_to_send==type_INFO_1) {
                    state = state_C_REJ_1;
                }
                else if(receiver_message[0]==0x05 && message_to_send==type_INFO_1) {
                    state = state_C_RR_0;
                }
                else if(receiver_message[0]==0x85 && message_to_send==type_INFO_0) {
                    state = state_C_RR_1;
                }
                else {
                    state = state_START;
                }
                break;
            case state_C_REJ_0:
                if(receiver_message[0]==0x7E) {
                    state = state_FLAG_1_OK;
                }
                else if(receiver_message[0]==(0x03^0x01)) {
                    state = state_MUST_RESEND;
                }
                else {
                    state = state_START;
                }
                break;
            case state_C_REJ_1:
                if(receiver_message[0]==0x7E) {
                    state = state_FLAG_1_OK;
                }
                else if(receiver_message[0]==(0x03^0x81)) {
                    state = state_MUST_RESEND;
                }
                else {
                    state = state_START;
                }
                break;
            case state_MUST_RESEND:
                if(receiver_message[0]==0x7E) {
                    alarm(0);
                    alarmCount = 0;
                    alarmEnabled = FALSE;
                    state = state_START;
                }
                else {
                    state = state_START;
                }
                break;
            case state_C_RR_0:
                if(receiver_message[0]==0x7E) {
                    state = state_FLAG_1_OK;
                }
                else if(receiver_message[0]==0x02) {
                    state = state_FINISH;
                }
                else {
                    state = state_START;
                }
                break;
            case state_C_RR_1:
                if(receiver_message[0]==0x7E) {
                    state = state_FLAG_1_OK;
                }
                else if(receiver_message[0]==0x82) {
                    state = state_FINISH;
                }
                else {
                    state = state_START;
                }
                break;
            case state_FINISH:
                if(receiver_message[0]==0x7E) {
                    alarm(0);
                    alarmCount = 0;
                    alarmEnabled = FALSE;
                    if(message_to_send==type_INFO_0) {
                        message_to_send = type_INFO_1;
                    }
                    else {
                        message_to_send = type_INFO_0;
                    }
                    return bufSize_;
                }
                else {
                    state = state_START;
                }
                break;
            default:
                break;
        }
    }
    alarm(0);
    alarmCount = 0;
    alarmEnabled = FALSE;
    return -1;
}

unsigned char can_receive_bit = 0x00;

////////////////////////////////////////////////
// LLREAD
////////////////////////////////////////////////
int llread(unsigned char *packet) // -1: error; 0: no more data; n > 0: number of bytes written to packet
{
    unsigned char BCC2_calc = 0x00;
    unsigned char state = state_START;
    unsigned char transmitter_message[1] = {0};
    int index = 0;
    unsigned char last_read = 0x00;

    while(TRUE) {
        int byte = read(fd, transmitter_message, 1);
        if(byte <= 0) continue;

        switch(state) {
            case state_START:
                if(transmitter_message[0]==0x7E) {
                    state = state_FLAG_1_OK;
                }
                else {
                	state = state_START;
                }
                break;
            case state_FLAG_1_OK:
                if(transmitter_message[0]==0x7E) {
                    state = state_FLAG_1_OK;
                }
                else if(transmitter_message[0]==0x03) {
                    state = state_A_OK;
                }
                else {
                    state = state_START;
                }
                break;
            case state_A_OK:
                if(transmitter_message[0]==0x7E) {
                    state = state_FLAG_1_OK;
                }
                else if(transmitter_message[0]==can_receive_bit) {
                    if(can_receive_bit==0x00) {
                        state = state_INFO_0;
                    }
                    else {
                        state = state_INFO_1;
                    }
                }
                else if(transmitter_message[0]==(can_receive_bit^0x40)) {
                    if(can_receive_bit==0x00) {
                        (void) write(fd, RR_0, 5);
                        packets_received++;
                        return -1;
                    }
                    else {
                        (void) write(fd, RR_1, 5);
                        packets_received++;
                        return -1;
                    }
                    state = state_START;
                }
                else if(transmitter_message[0]==0x0B) {
                    state = state_is_DISC;
                }
                else {
                    state = state_START;
                }
                break;
            case state_INFO_0:
                if(transmitter_message[0]==0x7E) {
                    state = state_FLAG_1_OK;
                }
                else if(transmitter_message[0]==0x03) {
                    state = state_READ_INFO;
                }
                else {
                    state = state_START;
                }
                break;
            case state_INFO_1:
                if(transmitter_message[0]==0x7E) {
                    state = state_FLAG_1_OK;
                }
                else if(transmitter_message[0]==0x43) {
                    state = state_READ_INFO;
                }
                else {
                    state = state_START;
                }
                break;
            case state_READ_INFO:
                if(transmitter_message[0]==0x7E) {
                    if(last_read == BCC2_calc) {
                        if(can_receive_bit==0x00) {
                            (void) write(fd, RR_1, 5);
                            can_receive_bit = 0x40;
                        }
                        else {
                            (void) write(fd, RR_0, 5);
                            can_receive_bit = 0x00;
                        }
                        packets_received++;
                        return index-1;
                    }
                    else {
                        if(can_receive_bit==0x00) {
                            (void) write(fd, REJECT_0, 5);
                            packets_received++;
                            return -1;
                        }
                        else {
                            (void) write(fd, REJECT_1, 5);
                            packets_received++;
                            return -1;
                        }
                    }
                }
                else if(transmitter_message[0]==0x7D) {
                    state = state_GOT_7D;
                }
                else {
                    packet[index] = transmitter_message[0];
                    index++;
                    // ret++;
                    BCC2_calc ^= last_read;
                    last_read = transmitter_message[0];
                    state = state_READ_INFO;
                }
                break;
            case state_GOT_7D:
                if(transmitter_message[0]==0x5D) {
                    packet[index] = 0x7D;
                    index++;
                    BCC2_calc ^= last_read;
                    last_read = 0x7D;
                    state = state_READ_INFO;
                }
                else if(transmitter_message[0]==0x5E) {
                    packet[index] = 0x7E;
                    index++;
                    BCC2_calc ^= last_read;
                    last_read = 0x7E;
                    state = state_READ_INFO;
                }
                else {
                    state = state_START;
                    index = 0;
                    if(can_receive_bit==0x00) {
                        (void) write(fd, REJECT_0, 5);
                        packets_received++;
                    }
                    else {
                        (void) write(fd, REJECT_1, 5);
                        packets_received++;
                    }
                    return -1;
                }
                break;
            case state_is_DISC:
                if(transmitter_message[0]==0x7E) {
                    state = state_FLAG_1_OK;
                }
                else if(transmitter_message[0]==0x08) {
                    state = state_awaiting_FLAG_DISC;
                }
                else {
                    state = state_START;
                }
                break;
            case state_awaiting_FLAG_DISC:
                if(transmitter_message[0]==0x7E) {
                    (void) write(fd, RECEIVER_DISC, 5);
                    state = state_START;
                    return 0;
                }
                else {
                    state = state_START;
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
    if(connectionParameters_.role == LlTx) {
        (void) write(fd, TRANSMITTER_DISC, 5);
    }

    // Restore the old port settings
    if (tcsetattr(fd, TCSANOW, &oldtio) == -1)
    {
        perror("tcsetattr");
        exit(-1);
    }

    if(showStatistics==TRUE) {
        if(connectionParameters_.role==LlTx) {
            printf("BitRate = %d\n", bit_rate);
            printf("Number of packets sent = %d\n", packets_sent);
        }
        else {
            printf("BitRate = %d\n", bit_rate);
            printf("Number of packets received = %d\n", packets_received);
        }
    }

    close(fd);

    return 1;
}
