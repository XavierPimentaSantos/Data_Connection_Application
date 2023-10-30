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

// unsigned char FLAG = 0x7E;

// unsigned char A_TC_RR = 0x03; // A for Transmitter commands or Receiver replies
// unsigned char A_RC_TR = 0x01; // A for Receiver comands or Transmitter replies

// unsigned char C_SET = 0x03; // C for SET messages
// unsigned char C_UA = 0x07; // C for UA messages
// unsigned char C_RR0 = 0x05; // C for Receiver to signal it is ready to receive 0-type data message
// unsigned char C_RR1 = 0x85; // C for Receiver to signal it is ready to receive 1-type data message
// unsigned char C_REJ0 = 0x01; // C for Receiver to signal it rejects the last 0-type data message
// unsigned char C_REJ1 = 0x81; // C for Receiver to signal it rejects the last 1-type data message
// unsigned char C_DISC = 0x0B; // C for DISC messages

// unsigned char C_INFO_0 = 0x00; // C for Information messages with byte 0
// unsigned char C_INFO_1 = 0x40; // C for Information messages with byte 1

// unsigned char ESC_SEQUENCE = 0x7D; // escape sequence for byte stuffing
// unsigned char ESC_5d = 0x5D; // if 0x7D in Data
// unsigned char ESC_5e = 0x5E; // if 0x7E in Data

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

// possible values for message_to_send
#define type_SET 0
#define type_INFO_0 1
#define type_INFO_1 2
#define type_UA 3
#define type_DISC 4

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

// isto fica de fora para ser acedido por todas as funções
int fd; // port
struct termios oldtio; // old port parameters
struct termios newtio; // new port parameters
LinkLayer connectionParameters_;

// related to alarm
int timeout_;
int nTries_;
int alarmEnabled = FALSE;
int alarmCount = 0;

// used by llwrite to hold the data sent by applicationLayer
unsigned char *buf_[3000]; // 3000 = 3 * MAX_PAYLOAD_SIZE
int bufSize_ = 0;

int message_to_send = type_SET; // defines the type of message to be sent by Transmitter

unsigned char DISC = FALSE; // TRUE after Tx sends DISC message
unsigned char DISC_final = FALSE; // TRUE after Rx replies to DISC message

unsigned char can_receive_bit = 0x00; // used by llread() to know whether to expect INFO_0 or INFO_1

unsigned char buf_to_send[3000] = {0}; // 3 * MAX_PAYLOAD_SIZE, more than enough space

void send_message(int signal) {
    alarmEnabled = FALSE;
    alarmCount++;

	printf("message_to_send = %i\n", message_to_send);

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
        DISC = TRUE;
        printf("%d bytes written\n", bytes);
    }
    else {
        unsigned char BCC2 = 0x00; // used to calculate the BCC2 of the bytes added into buf_to_send
        int actual_bufSize_ = bufSize_ + 4; // +4: we account for the first 4 static bytes; +=1 when we insert an escape character sequence into buf_to_send

        // insert the static information bytes (F, A, C, BCC1) into buf_to_send
        buf_to_send[0] = 0x7E;
        buf_to_send[1] = 0x03;
        if(message_to_send==type_INFO_0) {
            buf_to_send[2] = 0x00;
            buf_to_send[3] = 0x03; // 0x00^0x03
        }
        else /*if(message_to_send==type_INFO_1)*/ {
            buf_to_send[2] = 0x40;
            buf_to_send[3] = 0x43; // 0x40^0x03
        }

        // insert data values (D1...Dn) into buf_to_send
        for(int i = 0, i_helper = 0; i < bufSize_; i++) {
            unsigned char read_byte = (unsigned char) buf_[i];
            // printf("byte put into message = 0x%02X\n", read_byte);
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

        // insert BCC2 and F into buf_to_send
        buf_to_send[actual_bufSize_++] = BCC2;
        buf_to_send[actual_bufSize_++] = 0x7E;
	
        printf("message = 0x");
        for(int k = 0; k < actual_bufSize_; k++) {
            printf("%02X", buf_to_send[k]);
        }
        printf("\n");
        printf("ABF = %i\n", actual_bufSize_);
	
        // write buf_to_send to fd
        (void) write(fd, buf_to_send, actual_bufSize_);

        float sleepy = 0.001 * actual_bufSize_;
        sleep(sleepy); // in order to give time for the receiver to go through all of the message
    }
    // else if(message_to_send==type_INFO_0) {
    //     unsigned char BCC2 = 0x00;
    //     int bufSize_helper = 0;
    //     buf_to_send[0] = 0x7E;
    //     buf_to_send[1] = 0x03;
    //     buf_to_send[2] = 0x00;
    //     buf_to_send[3] = 0x03; // 0x03^0x00
    //     printf("added static values\n");
    //     for(int i = 4; i < bufSize_+4; i++) {
    //     	unsigned char read_byte = buf_[i-4];
    //     	printf("read_byte = 0x%02X\n", read_byte);
    //         BCC2 ^= read_byte;
    //         if(read_byte==0x7E) {
    //             buf_to_send[i] = 0x7D;
    //             printf("h");
    //             i++;
    //             printf("h");
    //             buf_to_send[i] = 0x5E;
    //             printf("h");
    //             bufSize_helper++;
    //             printf("h\n");
    //         }
    //         else if(read_byte==0x7D) {
    //             buf_to_send[i] = 0x7D;
    //             printf("j");
    //             i++;
    //             printf("j");
    //             buf_to_send[i] = 0x5D;
    //             printf("j");
    //             bufSize_helper++;
    //         	printf("j\n");
    //         }
    //         else {
    //             buf_to_send[i] = read_byte;
    //             printf("k\n");
    //         }
    //         printf("buf_to_send = 0x");
	// 		for(int l = 0; l < bufSize_+bufSize_helper; l++) {
	// 			printf("%02X", buf_to_send[l]);
	// 		}
	// 		printf("\n");
    //     }
    //     buf_to_send[bufSize_+bufSize_helper+4] = BCC2;
    //     buf_to_send[bufSize_+bufSize_helper+5] = 0x7E;
	// 	printf("buf_to_send = 0x");
	// 	for(int l = 0; l < bufSize_+6; l++) {
	// 		printf("%02X", buf_to_send[l]);
	// 	}
	// 	printf("\n");
    //     write(fd, buf_to_send, bufSize_ + bufSize_helper + 6);
    // }
    // else /*if(message_to_send==type_INFO_1)*/ {
    //     unsigned char BCC2 = 0x00;
    //     int bufSize_helper = 0;
    //     buf_to_send[0] = 0x7E;
    //     buf_to_send[1] = 0x03;
    //     buf_to_send[2] = 0x40;
    //     buf_to_send[3] = 0x43; // 0x03^0x40
    //     printf("added static values\n");
    //     for(int i = 4; i < bufSize_+4; i++) {
    //     	unsigned char read_byte = buf_[i-4];
    //         BCC2 ^= read_byte;
    //         if(read_byte==0x7E) {
    //             buf_to_send[i] = 0x7D;
    //             printf("h");
    //             i++;
    //             printf("h");
    //             buf_to_send[i] = 0x5E;
    //             printf("h");
    //             bufSize_helper++;
    //             printf("h\n");
    //         }
    //         else if(read_byte==0x7D) {
    //             buf_to_send[i] = 0x7D;
    //             printf("j");
    //             i++;
    //             printf("j");
    //             buf_to_send[i] = 0x5D;
    //             printf("j");
    //             bufSize_helper++;
    //         	printf("j\n");
    //         }
    //         else {
    //             buf_to_send[i] = read_byte;
    //             printf("k\n");
    //         }
    //         printf("buf_to_send = 0x");
	// 		for(int l = 0; l < bufSize_+bufSize_helper; l++) {
	// 			printf("%02X", buf_to_send[l]);
	// 		}
	// 		printf("\n");
    //     }
    //     buf_to_send[bufSize_+4] = BCC2;
    //     buf_to_send[bufSize_+5] = 0x7E;
	// 	printf("buf_to_send = 0x");
	// 	for(int l = 0; l < bufSize_+6; l++) {
	// 		printf("%02X", buf_to_send[l]);
	// 	}
	// 	printf("\n");
    //     write(fd, buf_to_send, bufSize_ + bufSize_helper + 6);
    // }
}

////////////////////////////////////////////////
// LLOPEN
////////////////////////////////////////////////
int llopen(LinkLayer connectionParameters)
{
    // TODO
    connectionParameters_ = connectionParameters_;
    timeout_ = connectionParameters.timeout;
    nTries_ = connectionParameters.nRetransmissions;

    int open_STOP = FALSE;
    unsigned char open_buf[1] = {0};
    int open_state = 0;

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

    if(connectionParameters.role == LlTx) { // llopen() was called by the Transmitter
        message_to_send = type_SET;
        timeout_ = connectionParameters.timeout;
        nTries_ = connectionParameters.nRetransmissions;
        (void) signal(SIGALRM, send_message);

        while(alarmCount < nTries_ && open_STOP == FALSE) {
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
                        open_STOP = TRUE; // received UA from receiver, can stop execution
                    }
                    else {
                        open_state = 0;
                    }
                    break;
            }
        }
        alarm(0);
        alarmCount = 0;
        alarmEnabled = FALSE;
        message_to_send = type_INFO_0;
        return 1;
    }

    else /*if(connectionParameters.role == LlRx)*/{ // llopen() was called by the Receiver
        while(open_STOP == FALSE) {
            int byte = read(fd, open_buf, 1);
            if(byte <= 0) continue;

            switch(open_state) {
                case 0: // START
                    if(open_buf[0]==0x7E) {
                        open_state = 1;
                        printf("read a byte open_read = 0x%02X\n", open_buf[0]);
                    }
                    else {
                        open_state = 0;
                        printf("read a byte open_read = 0x%02X\n", open_buf[0]);
                    }
                    break;
                case 1: // RECEIVED FLAG
                    if(open_buf[0]==0x7E) {
                        open_state = 1;
                        printf("read a byte open_read = 0x%02X\n", open_buf[0]);
                    }
                    else if(open_buf[0]==0x03) {
                        open_state = 2;
                        printf("read a byte open_read = 0x%02X\n", open_buf[0]);
                    }
                    else {
                        open_state = 0;
                        printf("read a byte open_read = 0x%02X\n", open_buf[0]);
                    }
                    break;
                case 2: // RECEIVED A
                    if(open_buf[0]==0x7E) {
                        open_state = 1;
                        printf("read a byte open_read = 0x%02X\n", open_buf[0]);
                    }
                    else if(open_buf[0]==0x03) {
                        open_state = 3;
                        printf("read a byte open_read = 0x%02X\n", open_buf[0]);
                    }
                    else {
                        open_state = 0;
                        printf("read a byte open_read = 0x%02X\n", open_buf[0]);
                    }
                    break;
                case 3: // RECEIVED C
                    if(open_buf[0]==0x7E) {
                        open_state = 1;
                        printf("read a byte open_read = 0x%02X\n", open_buf[0]);
                    }
                    else if(open_buf[0]==0x00) {
                        open_state = 4;
                        printf("read a byte open_read = 0x%02X\n", open_buf[0]);
                    }
                    else {
                        open_state = 0;
                        printf("read a byte open_read = 0x%02X\n", open_buf[0]);
                    }
                    break;
                case 4: // RECEIVED BCC
                    if(open_buf[0]==0x7E) {
                        open_STOP = TRUE;
                        write(fd, RECEIVER_UA_MESSAGE, 5);
                        printf("sent UA message\n");
                    }
                    else {
                        open_state = 0;
                        printf("read a byte open_read = 0x%02X\n", open_buf[0]);
                    }
                    break;
            }
        }
        return 1;
    }

    return -1;
}



////////////////////////////////////////////////
// LLWRITE
////////////////////////////////////////////////
int llwrite(const unsigned char *buf, int bufSize)
{
    // TODO
    unsigned char write_STOP = FALSE; // TRUE after receiving RR message
    unsigned char receiver_message[1] = {0}; // message from Receiver
    unsigned char state = state_START;

    printf("buf = 0x");
    for(int n = 0; n < bufSize; n++) {
    	buf_[n] = buf[n]; // copy every u_char from buf to buf_
    		printf("%02X", buf_[n]);
  	}
    printf("\n");
    printf("bufSize = %i\n", bufSize);
    printf("llwrite was called\n");
    
  	bufSize_ = bufSize;
    
    (void)signal(SIGALRM, send_message); // activate alarm-based message sending

    while(alarmCount < nTries_ && write_STOP==FALSE) {
    	if(alarmEnabled == FALSE) {
            alarm(timeout_);
            alarmEnabled = TRUE;
        }
        //printf("CURRENT STATE: %i\n", state);
	
        int byte = read(fd, receiver_message, 1);
        if(byte <= 0) continue;

        // STATE MACHINE
        switch(state) {
            case state_START:
                if(receiver_message[0]==0x7E) {
                    state = state_FLAG_1_OK;
                    printf("byte read = 0x%02X and will change state \n", receiver_message[0]);
                }
                else {
                    state = state_START;
                    printf("byte read = 0x%02X\n", receiver_message[0]);
                }
                break;
            case state_FLAG_1_OK:
                if(receiver_message[0]==0x7E) {
                    state = state_FLAG_1_OK;
                    printf("byte read = 0x%02X and will change state \n", receiver_message[0]);
                }
                else if(receiver_message[0]==0x03) {
                    state = state_A_OK;
                    printf("byte read = 0x%02X and will change state \n", receiver_message[0]);
                }
                /* else if(receiver_message[0]==0x01 && DISC==TRUE) {
                    state = A_DISC_OK;
                    printf("byte read = 0x%02X and will change state \n", receiver_message[0]);
                } */
                else {
                    state = state_START;
                    printf("byte read = 0x%02X\n", receiver_message[0]);
                }
                break;
            case state_A_OK:
                if(receiver_message[0]==0x7E) {
                    state = state_FLAG_1_OK;
                    printf("byte read = 0x%02X and will change state \n", receiver_message[0]);
                }
                /* else if(receiver_message[0]==0x07) {
                    state = C_UA_OK;
                    printf("byte read = 0x%02X and will change state \n", receiver_message[0]);
                } */
                else if(receiver_message[0]==0x01 && message_to_send==type_INFO_0) {
                    state = state_C_REJ_0;
                    printf("byte read = 0x%02X and will change state \n", receiver_message[0]);
                }
                else if(receiver_message[0]==0x81 && message_to_send==type_INFO_1) {
                    state = state_C_REJ_1;
                    printf("byte read = 0x%02X and will change state \n", receiver_message[0]);
                }
                else if(receiver_message[0]==0x05 && message_to_send==type_INFO_1) {
                    state = state_C_RR_0;
                    printf("byte read = 0x%02X and will change state \n", receiver_message[0]);
                }
                else if(receiver_message[0]==0x85 && message_to_send==type_INFO_0) {
                    state = state_C_RR_1;
                    printf("byte read = 0x%02X and will change state \n", receiver_message[0]);
                }
                else {
                    state = state_START;
                    printf("byte read = 0x%02X\n", receiver_message[0]);
                }
                break;
            /* case A_DISC_OK:
                if(receiver_message[0]==0x7E) {
                    state = FLAG_1_OK;
                    printf("byte read = 0x%02X and will change state \n", receiver_message[0]);
                }
                else if(receiver_message[0]==0x0B) {
                    state = C_DISC_OK;
                    printf("byte read = 0x%02X and will change state \n", receiver_message[0]);
                }
                else {
                    state = START;
                    printf("byte read = 0x%02X and will change state \n", receiver_message[0]);
                }
                break;
            case C_DISC_OK:
                if(receiver_message[0]==0x7E) {
                    state = FLAG_1_OK;
                    printf("byte read = 0x%02X and will change state \n", receiver_message[0]);
                }
                else if(receiver_message[0]==(0x01^0x0B)) {
                    state = BCC_DISC_OK;
                    printf("byte read = 0x%02X and will change state \n", receiver_message[0]);
                }
                else {
                    state = START;
                    printf("byte read = 0x%02X and will change state \n", receiver_message[0]);
                }
                break;
            case BCC_DISC_OK:
                if(receiver_message[0]==0x7E) {
                    // received DISC from Receiver, must send UA and terminate connection
                    alarmCount = 0;
                    alarm(0); // disable pending alarms (will be restored in the next execution I think)
                    nTries_ = 1; // we only want to send UA once (acknowledgement on the receiver's part is not needed)
                    message_to_send = type_UA;
                    STOP = TRUE; // on the next cycle we exit the execution
                    printf("will disconnect\n");
                    printf("byte read = 0x%02X and will change state \n", receiver_message[0]);
                }
                else {
                    state = START;
                    printf("byte read = 0x%02X and will change state \n", receiver_message[0]);
                }
                break; */
            /* case C_UA_OK:
                if(receiver_message[0]==0x7E) {
                    state = FLAG_1_OK;
                    printf("byte read = 0x%02X and will change state \n", receiver_message[0]);
                }
                else if(receiver_message[0]==(0x03^0x07)) {
                    state = BCC_UA_OK;
                    printf("byte read = 0x%02X and will change state \n", receiver_message[0]);
                }
                else {
                    state = START;
                    printf("byte read = 0x%02X and will change state \n", receiver_message[0]);
                }
                break;
            case BCC_UA_OK:
                if(receiver_message[0]==0x7E) {
                    if(message_to_send==type_SET) {
                        message_to_send = type_INFO_0;
                        printf("sent SET message and was acknowledged\n");
                    printf("byte read = 0x%02X and will change state \n", receiver_message[0]);, receiver_message[0]);
                    }
                    else if(message_to_send==type_INFO_0) {
                        message_to_send = type_INFO_1;
                        printf("sent INFO_0 message and was acknowledged\n");
                    printf("byte read = 0x%02X and will change state \n", receiver_message[0]);
                    }
                    else  {
                        message_to_send = type_INFO_0;
                        printf("sent INFO_1 message and was acknowledged\n");
                    printf("byte read = 0x%02X and will change state \n", receiver_message[0]);
                    }
                    state = START;
                }
                else {
                    state = START;
                    printf("byte read = 0x%02X and will change state \n", receiver_message[0]);
                }
                break; */
            case state_C_REJ_0:
                if(receiver_message[0]==0x7E) {
                    state = state_FLAG_1_OK;
                    printf("byte read = 0x%02X and will change state \n", receiver_message[0]);
                }
                else if(receiver_message[0]==(0x03^0x01)) {
                    state = state_MUST_RESEND;
                    printf("byte read = 0x%02X and will change state \n", receiver_message[0]);
                }
                else {
                    state = state_START;
                    printf("byte read = 0x%02X\n", receiver_message[0]);
                }
                break;
            case state_C_REJ_1:
                if(receiver_message[0]==0x7E) {
                    state = state_FLAG_1_OK;
                    printf("byte read = 0x%02X and will change state \n", receiver_message[0]);
                }
                else if(receiver_message[0]==(0x03^0x81)) {
                    state = state_MUST_RESEND;
                    printf("byte read = 0x%02X and will change state \n", receiver_message[0]);
                }
                else {
                    state = state_START;
                    printf("byte read = 0x%02X\n", receiver_message[0]);
                }
                break;
            case state_MUST_RESEND:
                if(receiver_message[0]==0x7E) {
                    //resend message, reset alarm and alarmCount, etc etc
                    alarmCount = 0;
                    alarmEnabled = FALSE;
                    alarm(0);
                    state = state_START;
                    printf("byte read = 0x%02X and will change state \n", receiver_message[0]);
                    continue; //immediately sends execution to the next "while" cycle (unsure if this works as intended)
                }
                else {
                    state = state_START;
                    printf("byte read = 0x%02X\n", receiver_message[0]);
                }
                break;
            case state_C_RR_0:
                if(receiver_message[0]==0x7E) {
                    state = state_FLAG_1_OK;
                    printf("byte read = 0x%02X and will change state \n", receiver_message[0]);
                }
                else if(receiver_message[0]==0x02) {
                    state = state_FINISH;
                    printf("byte read = 0x%02X and will change state \n", receiver_message[0]);
                }
                else {
                    state = state_START;
                    printf("byte read = 0x%02X\n", receiver_message[0]);
                }
                break;
            case state_C_RR_1:
                if(receiver_message[0]==0x7E) {
                    state = state_FLAG_1_OK;
                    printf("byte read = 0x%02X and will change state \n", receiver_message[0]);
                }
                else if(receiver_message[0]==0x82) {
                    state = state_FINISH;
                    printf("byte read = 0x%02X and will change state \n", receiver_message[0]);
                }
                else {
                    state = state_START;
                    printf("byte read = 0x%02X\n", receiver_message[0]);
                }
                break;
            case state_FINISH:
                if(receiver_message[0]==0x7E) {
                    printf("byte read = 0x%02X and will finish \n", receiver_message[0]);
                    write_STOP = TRUE;
                    if(message_to_send==type_INFO_0) {
                        message_to_send = type_INFO_1;
                    }
                    else {
                        message_to_send = type_INFO_0;
                    }
                    printf("i will return\n");
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

    return -1;
}

////////////////////////////////////////////////
// LLREAD
////////////////////////////////////////////////
int llread(unsigned char *packet) // -1: error; 0: no more data; n > 0: number of bytes written to packet
{
    // TODO
    unsigned char read_STOP = FALSE; // TRUE after receiving a correct frame and sending appropriate RR reply
    unsigned char BCC2_calc = 0x00; // used to calculate the BCC2 of data frames
    unsigned char state = state_START;
    unsigned char transmitter_message[1] = {0};
    int index = 0;
    unsigned char DISC_r = FALSE;
    unsigned char standby_byte;
    unsigned char last_read = 0x00; // arbitrary value
    int ret = 0;

    /*
    7E 03 0B 08 7E : DISC frame
    7E 01 07 06 7E : UA frame
    7E 03 00 03 data bcc2 7E : INFO_0 frame
    7E 03 40 43 data bcc2 7E : INFO_1 frame
    */

    printf("llread was called\n");
    
    while(read_STOP == FALSE) {
        int byte = read(fd, transmitter_message, 1);
        if(byte <= 0) continue;
        printf("byte read initially = 0x%02X\n", transmitter_message[0]);
        switch(state) {
            // main states (F1 = 0x7E)
            case state_START:
                if(transmitter_message[0]==0x7E) {
                    state = state_FLAG_1_OK;
                    //printf("byte read = 0x%02X\n", transmitter_message[0]);
                }
                else {
                	state = state_START;
                	//printf("byte read = 0x%02X\n", transmitter_message[0]);
                }
                break;
            case state_FLAG_1_OK:
                if(transmitter_message[0]==0x7E) {
                    state = state_FLAG_1_OK;
                    //printf("byte read = 0x%02X\n", transmitter_message[0]);
                }
                else if(transmitter_message[0]==0x03) {
                    state = state_A_OK;
                    printf("byte read = 0x%02X\n", transmitter_message[0]);
                }
                /* else if(DISC_r==TRUE && transmitter_message[0]==0x01) {
                    state = state_A_UA_OK;
                    printf("byte read = 0x%02X\n", transmitter_message[0]);
                } */
                else {
                    state = state_START;
                    // printf("byte read = 0x%02X\n", transmitter_message[0]);
                }
                break;
            case state_A_OK:
                if(transmitter_message[0]==0x7E) {
                    state = state_FLAG_1_OK;
                    // printf("byte read = 0x%02X\n", transmitter_message[0]);
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
                        // send RR0
                        (void) write(fd, RR_0, 5);
                        return -1;
                    }
                    else {
                        // send RR1
                        (void) write(fd, RR_1, 5);
                        return -1;
                    }
                    state = state_START;
                }
                /* else if(transmitter_message[0]==0x03) {
                    state = is_SET;
                    printf("byte read = 0x%02X\n", transmitter_message[0]);
                } */
                else if(transmitter_message[0]==0x0B) {
                    state = state_is_DISC;
                    // printf("byte read = 0x%02X\n", transmitter_message[0]);
                }
                else {
                    state = state_START;
                    // printf("byte read = 0x%02X\n", transmitter_message[0]);
                }
                break;
            /* case state_awaiting_FLAG:
                if(transmitter_message[0]==0x7E) {
                    state = state_START;
                    write(fd, RECEIVER_UA_MESSAGE, 5);
                    printf("byte read = 0x%02X\n", transmitter_message[0]);
                }
                else {
                    state = START;
                    printf("byte read = 0x%02X\n", transmitter_message[0]);
                }
                break; */

            // UA message states (A = 0x01, C = 0x07, BCC = 0x06, F2 = 0x7E)
            /* case state_A_UA_OK:
                if(transmitter_message[0]==0x7E) {
                    state = state_FLAG_1_OK;
                    printf("byte read = 0x%02X\n", transmitter_message[0]);
                }
                else if(transmitter_message[0]==0x07) {
                    state = state_C_UA_OK;
                    printf("byte read = 0x%02X\n", transmitter_message[0]);
                }
                else {
                    state = state_START;
                    printf("byte read = 0x%02X\n", transmitter_message[0]);
                }
                break;
            case state_C_UA_OK:
                if(transmitter_message[0]==0x7E) {
                    state = state_FLAG_1_OK;
                    printf("byte read = 0x%02X\n", transmitter_message[0]);
                }
                else if(transmitter_message[0]==0x06) {
                    state = state_BCC_UA_OK;
                    printf("byte read = 0x%02X\n", transmitter_message[0]);
                }
                else {
                    state = state_START;
                    printf("byte read = 0x%02X\n", transmitter_message[0]);
                }
                break;
            case state_BCC_UA_OK:
                if(transmitter_message[0]==0x7E) {
                    // we can close
                    return 0;
                }
                else {
                    state = state_START;
                }
                break; */

            // information states
            case state_INFO_0:
                if(transmitter_message[0]==0x7E) {
                    state = state_FLAG_1_OK;
                    // printf("byte read = 0x%02X\n", transmitter_message[0]);
                }
                else if(transmitter_message[0]==0x03) {
                    state = state_READ_INFO;
                    // printf("byte read = 0x%02X\n", transmitter_message[0]);
                }
                else {
                    state = state_START;
                    // printf("byte read = 0x%02X\n", transmitter_message[0]);
                }
                break;
            case state_INFO_1:
                if(transmitter_message[0]==0x7E) {
                    state = state_FLAG_1_OK;
                    // printf("byte read = 0x%02X\n", transmitter_message[0]);
                }
                else if(transmitter_message[0]==0x43) {
                    state = state_READ_INFO;
                    // printf("byte read = 0x%02X\n", transmitter_message[0]);
                }
                else {
                    state = state_START;
                    // printf("byte read = 0x%02X\n", transmitter_message[0]);
                }
                break;
            case state_READ_INFO:
                if(transmitter_message[0]==0x7E) {
                    /* state = state_START;
                    printf("byte read = 0x%02X\n", transmitter_message[0]);
                    index = 0;
                    if(can_receive_bit==0x00) {
                        write(fd, REJECT_0, 5);
                        printf("there was an error in the message; requesting retransmission\n");
                    }
                    else {
                        write(fd, REJECT_1, 5);
                        printf("there was an error in the message; requesting retransmission\n");
                    } */
                    if(last_read == BCC2_calc) {
                        if(can_receive_bit==0x00) {
                            write(fd, RR_1, 5);
                        }
                        else {
                            write(fd, RR_0, 5);
                        }
                        return ret;
                    }
                    else {
                        if(can_receive_bit==0x00) {
                            write(fd, REJECT_0, 5);
                            return -1;
                        }
                        else {
                            write(fd, REJECT_1, 5);
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
                    ret++;
                    BCC2_calc ^= last_read;
                    last_read = transmitter_message[0];
                }
                /* else if(transmitter_message[0]==BCC2_calc) {
                    state = state_STANDBY;
                    standby_byte = transmitter_message[0];
                    printf("byte read = 0x%02X\n", transmitter_message[0]);
                }
                else if(transmitter_message[0]==0x7D) {
                    state = state_GOT_7D;
                    printf("byte read = 0x%02X\n", transmitter_message[0]);
                }
                else {
                    state = state_READ_INFO;
                    printf("byte read = 0x%02X\n", transmitter_message[0]);
                    packet[index] = transmitter_message[0];
                    index++;
                    BCC2_calc = BCC2_calc ^ transmitter_message[0];
                } */
                break;
            case state_GOT_7D:
                if(transmitter_message[0]==0x5D) {
                    /* if(BCC2_calc == 0x7D) {
                        state = state_STANDBY; // BCC2 could be 0x7D, in which case we need to check for  0x7d 0x5d 0x7e
                        standby_byte = 0x5D;
                        printf("byte read = 0x%02X\n", transmitter_message[0]);
                    }
                    else {
                        state = state_READ_INFO;
                        packet[index] = 0x7D;
                        index++;
                        BCC2_calc = BCC2_calc ^ 0x7D;
                        printf("byte read = 0x%02X\n", transmitter_message[0]);
                    } */
                    packet[index] = 0x7D;
                    index++;
                    ret++;
                    BCC2_calc ^= last_read;
                    last_read = 0x7D;
                    state = state_READ_INFO;
                }
                else if(transmitter_message[0]==0x5E) {
                    /* if(BCC2_calc == 0x7E) {
                        state = state_STANDBY; // BCC2 could be 0x7E, in which case we need to check for 0x7d 0x5e 0x7e
                        standby_byte = 0x5E;
                        printf("byte read = 0x%02X\n", transmitter_message[0]);
                    }
                    else {
                        state = state_READ_INFO;
                        packet[index] = 0x7E;
                        index++;
                        BCC2_calc = BCC2_calc ^ 0x7E;
                        printf("byte read = 0x%02X\n", transmitter_message[0]);
                    } */
                    packet[index] = 0x7E;
                    index++;
                    ret++;
                    BCC2_calc ^= last_read;
                    last_read = 0x7E;
                    state = state_READ_INFO;
                }
                else {
                    state = state_START;
                    index = 0;
                    ret = 0;
                    printf("byte read = 0x%02X\n", transmitter_message[0]);
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
            /* case state_STANDBY:
                if(standby_byte==0x5D) {
                    if(transmitter_message[0]==0x7E) {
                        can_receive_bit = can_receive_bit ^ 0x40;
                        // BCC2 was 0x7D, and there was nothing wrong with the message; we can send RR
                        if(can_receive_bit==0x40) {
                            write(fd, RR_0, 5);
                        }
                        else {
                            write(fd, RR_1, 5);
                        }
                        printf("got data\n");
                        state = state_START;
                        printf("byte read = 0x%02X\n", transmitter_message[0]);
                    }
                    else { // didn't receive F, which means 0x7D was NOT the BCC2 byte, and therefore we have to send 0x7D to the packet
                        packet[index] = 0x7D;
                        index++;
                        if(transmitter_message[0]==0x7D) {
                            state = got_7d;
                            BCC2_calc = 0x00;
                    printf("byte read = 0x%02X\n", transmitter_message[0]);
                        }
                        else {
                            packet[index] = transmitter_message[0];
                            BCC2_calc = transmitter_message[0];
                            index++;
                            state = READ_INFO;
                    printf("byte read = 0x%02X\n", transmitter_message[0]);
                        }
                    }
                }
                else if(standby_byte==0x5E) {
                    if(transmitter_message[0]==0x7E) {
                        can_receive_bit = can_receive_bit ^ 0x40;
                        // BCC2 was 0x7E, and there was nothing wrong with the message; we can send RR and return
                        if(can_receive_bit==0x40) {
                            write(fd, RR_0, 5);
                        }
                        else {
                            write(fd, RR_1, 5);
                        }
                        printf("read data\n");
                        state = START;
                    printf("byte read = 0x%02X\n", transmitter_message[0]);
                    }
                    else { // didn't receive F, which means 0x7E was NOT the BCC2 byte, and therefore we have to send 0x7E to the packet
                        packet[index] = 0x7E;
                        index++;
                        if(transmitter_message[0]==0x7D) {
                            state = got_7d;
                            BCC2_calc = 0x00;
                        }
                        else {
                            packet[index] = transmitter_message[0];
                            BCC2_calc = transmitter_message[0];
                            index++;
                            state = READ_INFO;
                        }
                    printf("byte read = 0x%02X\n", transmitter_message[0]);
                    }
                }
                else if(transmitter_message[0]==0x7E) {
                    can_receive_bit = can_receive_bit ^ 0x40;
                    // BCC2 checks off as true, we received the flag, can send RR message and return
                    if(can_receive_bit==0x40) {
                        write(fd, RR_0, 5);
                        printf("received INFO_0\n");
                    }
                    else {
                        write(fd, RR_1, 5);
                        printf("received INFO_1\n");
                    }
                    state = START;
                    printf("byte read = 0x%02X\n", transmitter_message[0]);
                }
                else if(transmitter_message[0]==0x00) { // BCC2 may be 0x00, so we must check for that
                    state = STANDBY;
                    BCC2_calc = 0x00;
                    printf("byte read = 0x%02X\n", transmitter_message[0]);
                }
                else {
                    state = READ_INFO;
                    packet[index] = standby_byte;
                    printf("byte read = 0x%02X\n", transmitter_message[0]);
                }
                break; */

            /* case is_SET:
                if(transmitter_message[0]==0x7E) {
                    state = FLAG_1_OK;
                    printf("byte read = 0x%02X\n", transmitter_message[0]);
                }
                else if(transmitter_message[0]==0x00) {
                    state = awaiting_FLAG;
                    printf("byte read = 0x%02X\n", transmitter_message[0]);
                }
                else {
                    state = START;
                    printf("byte read = 0x%02X\n", transmitter_message[0]);
                }
                break; */
            
            // DISC states
            case state_is_DISC:
                if(transmitter_message[0]==0x7E) {
                    state = state_FLAG_1_OK;
                    // printf("byte read = 0x%02X\n", transmitter_message[0]);
                }
                else if(transmitter_message[0]==0x08) {
                    state = state_awaiting_FLAG_DISC;
                    // printf("byte read = 0x%02X\n", transmitter_message[0]);
                }
                else {
                    state = state_START;
                    // printf("byte read = 0x%02X\n", transmitter_message[0]);
                }
                break;
            case state_awaiting_FLAG_DISC:
                if(transmitter_message[0]==0x7E) {
                    DISC_r = TRUE;
                    printf("received DISC from Tx\n");
                    //send its own DISC message
                    write(fd, RECEIVER_DISC, 5);
                    printf("sent DISC to Tx\n");
                    state = state_START;
                    // printf("byte read = 0x%02X\n", transmitter_message[0]);
                    return 0; // after sending DISC we can return
                }
                else {
                    state = state_START;
                    // printf("byte read = 0x%02X\n", transmitter_message[0]);
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
    if(connectionParameters_.role == LlTx) {
        message_to_send = type_DISC;
        unsigned char close_STOP = FALSE;
        unsigned char close_buf[1] = {0};
        int close_state = 0;

        (void) signal(SIGALRM, send_message);

        while(alarmCount < nTries_ && close_STOP == FALSE) {
            if(alarmEnabled == FALSE) {
                alarm(timeout_);
                alarmEnabled = TRUE;
            }

            int byte = read(fd, close_buf, 1);
            if(byte <= 0) continue;

            switch(close_state) {
                case 0: // START
                    if(close_buf[0]==0x7E) {
                        close_state = 1;
                    }
                    else {
                        close_state = 0;
                    }
                    break;
                case 1: // READ FLAG
                    if(close_buf[0]==0x7E) {
                        close_state = 1;
                    }
                    else if(close_buf[0]==0x01) {
                        close_state = 2;
                    }
                    else {
                        close_state = 0;
                    }
                    break;
                case 2: // READ A
                    if(close_buf[0]==0x7E) {
                        close_state = 1;
                    }
                    else if(close_buf[0]==0x0B) {
                        close_state = 3;
                    }
                    else {
                        close_state = 0;
                    }
                    break;
                case 3: // READ C
                    if(close_buf[0]==0x7E) {
                        close_state = 1;
                    }
                    else if(close_buf[0]==0x0A) {
                        close_state = 4;
                    }
                    else {
                        close_state = 0;
                    }
                    break;
                case 4: // READ BCC
                    if(close_buf[0]==0x7E) {
                        close_STOP = TRUE; // received UA from receiver, can stop execution
                    }
                    else {
                        close_state = 0;
                    }
                    break;
            }
        }
        alarm(0);
        write(fd, TRANSMITTER_UA_MESSAGE, 5);
    }

    // Restore the old port settings
    if (tcsetattr(fd, TCSANOW, &oldtio) == -1)
    {
        perror("tcsetattr");
        exit(-1);
    }

    close(fd);

    return 1;
}
