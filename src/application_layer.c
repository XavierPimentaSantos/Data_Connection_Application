// Application layer protocol implementation

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <sys/stat.h>
#include <sys/types.h>

#include "application_layer.h"

#include "link_layer.h" // in order to call transmission level functions

void applicationLayer(const char *serialPort, const char *role, int baudRate,
                      int nTries, int timeout, const char *filename)
{
    // TODO
    LinkLayer connection;
    connection.baudRate = baudRate;
    connection.nRetransmissions = nTries;
    connection.timeout = timeout;
    strcpy(connection.serialPort, serialPort);
    if(strcmp(role, "rx") == 0)
        connection.role = LlRx;
    else if(strcmp(role, "tx") == 0)
        connection.role = LlTx;
    else return;

    printf("started application layer for %s\n", role);
    
    // unsigned char *buffer_helper[MAX_PAYLOAD_SIZE] = {0};
    
    unsigned long int size = 0;
    FILE *file_ptr;
    unsigned char START_CONTROL_PACKET[310] = {0};
    unsigned char END_CONTROL_PACKET[310] = {0};

    //calls for opening of port
    if(llopen(connection) < 0) {
        exit(-1);
    }

    // IF TRANSMITTER
    if(connection.role == LlTx) {
        unsigned char buffer[MAX_PAYLOAD_SIZE+3] = {0};
        file_ptr = fopen(filename, "r"); // load the file into file_ptr
	    printf("opened %s\n", filename);
	    
        struct stat st;
	    if(stat(filename, &st) == 0) {
            size = st.st_size;
        }
	    printf("read file ig, size is %i\n", size);

        unsigned char ex = 0xFF;

        // char *ex_filename;
        // ex_filename = "pissguin.gif";

        // FILE *ppp = fopen(ex_filename, "a");
        // unsigned char interim[size+1];
        // int aslhd = fread(interim, sizeof ex, size, file_ptr);
        // if(aslhd<size) {
        //     printf("alsdjjjj\n");
        // }
        // printf("%i", sizeof interim);
        // interim[size] = EOF;
      	// int mmm = 0;
      	// while(interim[mmm]!=EOF) {
      	// 	fputc(interim[mmm], ppp);
        //     mmm++;
      	// }

        // int akshda = fwrite(file_ptr, sizeof ex, size, ppp);
        // if(akshda < size) {
            // printf("alshdlnasdh\n");
        // }

        // create the static Control packets
        unsigned char size_size = 0;
        unsigned int size_2 = size;
        while(size_2 != 0) {
            size_2 = size_2 >> 8;
            size_size++;
        }
        
        int scp_i = 0;
	    printf("before assigning values to SCP\n");
        START_CONTROL_PACKET[scp_i++] = 2; // it is a START Control packet
        printf("h");
        START_CONTROL_PACKET[scp_i++] = 0; // section: size of file
        printf("h");
        START_CONTROL_PACKET[scp_i++] = size_size; // size of size of file
        printf("h");
        for(int l = 1; l <= size_size; l++) {
            START_CONTROL_PACKET[scp_i + l - 1] = (unsigned char) (size >> ((size_size - l) * 8));
        }
        // START_CONTROL_PACKET[scp_i] = size; // size of file
        printf("h");
        scp_i = scp_i + size_size;
        START_CONTROL_PACKET[scp_i++] = 1;
        printf("h");
        int keep_scp_i = scp_i++;
        START_CONTROL_PACKET[keep_scp_i] = 0;
        for(int k = 0; filename[k] != '\0'; k++) {
            START_CONTROL_PACKET[scp_i++] = filename[k];
            START_CONTROL_PACKET[keep_scp_i]++;
        }
        // scp_i is the size of START_CONTROL_PACKET and END_CONTROL_PACKET
	    printf("\nassigned values to SCP, will assign the same to ECP\n");
        END_CONTROL_PACKET[0] = 3; // ECP is essentially SCP but with 3 intead of 2 on the Control byte
        for(int j = 1; j < scp_i; j++) {
            END_CONTROL_PACKET[j] = START_CONTROL_PACKET[j];
        }
	    printf("successfully assigned values to ECP\n");

        unsigned char PROCEED_START = FALSE;
        unsigned char PROCEED_END = FALSE;
	    int f;

        if(llwrite(START_CONTROL_PACKET, scp_i) < scp_i) {
            printf("could not send the SCP\n");
            exit(-1);
        }

        int i = 0;
        int bufSize;
	    int can_seek = FALSE;
        while(i != size) {
            if(size - i < MAX_PAYLOAD_SIZE) {
                bufSize = size - i;
            }
            else {
                bufSize = MAX_PAYLOAD_SIZE;
            }
	        printf("bufSize Tx = %i\n", bufSize);
            unsigned char bufSize_lhs = (unsigned char) (bufSize>>8);
            unsigned char bufSize_rhs = (unsigned char) bufSize;            
            buffer[0] = 1;
            buffer[1] = bufSize_lhs;
            buffer[2] = bufSize_rhs;
            //printf("bufSize is 0x%02X\n", bufSize);
            printf("bufSize is 0x%02X%02X\n", bufSize_lhs, bufSize_rhs);

            // for(int j = 0; j < bufSize; j++) {
            //     buffer[j+3] = file_ptr[i + j];
            // }
            if(can_seek==TRUE) {
                int fseek_ret  = fseek(file_ptr, i, SEEK_SET);
                if(fseek_ret!=0) {
                    printf("fseek failed\n");
                }
            	(void) fread(buffer/*_helper*/+3, 1, bufSize, file_ptr/*+i*/);    
                can_seek = FALSE;
            }
            //printf("file_ptr = %i\n", file_ptr);
            else {
            	(void) fread(buffer/*_helper*/+3, 1, bufSize, file_ptr/*+i*/);
            }
            
            printf("Tx will send buf = 0x");
            for(int m = 0; m < bufSize; m++) {
            	printf("%02X", (unsigned char) buffer[m]);
            }
            printf("\n");
            // calls llwrite() for those bytes
            if(llwrite(buffer, bufSize+3) >= /*==*/ bufSize+3) {
                i += bufSize; // no errors, we can send another group of bytes
                can_seek = TRUE;
                printf("successfully sent %i bytes\n", bufSize);
            }
            else {
                printf("could not send the bytes\n");
                return;
            }
            // if there was an error, we resend the bytes
        } // do this until there is no more data to send

        // send the END_CONTROL_PACKET
        // while(PROCEED_END == FALSE) {
        //     if(llwrite(END_CONTROL_PACKET, scp_i) > 0 /*== scp_i*/) {
        //         PROCEED_END = TRUE;
        //     }
        //     else {
        //         continue;
        //     }
        // }

        if(llwrite(END_CONTROL_PACKET, scp_i) < scp_i) {
            printf("failed to send ECP\n");
            return;
        }

        //when llwrite() returns succesfully for the last time, close the file, call llclose() and finish execution
        fclose(file_ptr);
        if(llclose(FALSE) < 0) { // could be TRUE if we want to see the statistics
            return;
        }    
    }

    //IF RECEIVER
    else {
        unsigned char new_filename[50] = {0};
        unsigned char read_from_here[MAX_PAYLOAD_SIZE] = {0};
        int data_size = 0;
        int packet_size;
        int file_size_size;
        int loop = TRUE;
        // int i = 0;

        printf("i am a %s\n", role);

        while(loop == TRUE) {
            // call llread()
            memset(read_from_here, 0, MAX_PAYLOAD_SIZE);
            packet_size = llread(read_from_here);
            printf("packet_size = %i\n", packet_size);
            printf("received 0x");
            for(int l = 0; l < packet_size; l++) {
                printf("%02X", read_from_here[l]);
            }
            printf("\n");
            if(packet_size == -1) {
                continue; // we ignore what already exists in the array and try again
            }
            else if(packet_size == 0) {
                if(size == 0 && llclose(FALSE) > 0) { // right amount of data
                    printf("finishing execution\n");
                    return;
                }
                else {
                    printf("something went wrong, size = %d\n", size);
                    exit(-1);
                }
            }
            else {
                // succeeded in receiving the data; use state machine to determine what to do with it
                printf("llread read %i bytes\n", packet_size);
                if(read_from_here[0]==1) { // data
                    data_size = (read_from_here[1]<<8);
                    data_size += read_from_here[2];
                    unsigned char copy_[MAX_PAYLOAD_SIZE];
                    (void) memcpy(copy_, read_from_here+3, packet_size-3);
                    fwrite(copy_, sizeof read_from_here[0], packet_size-3, file_ptr/*+i*/);
                    // for(int l = 3; l < packet_size; l++) {
                    //     fwrite(read_from_here[l], sizeof read_from_here[0], 1, file_ptr);
                    // }
                    printf("succesfully wrote %i bytes to new file\n", packet_size-3);
                    // i += data_size;
                    size -= (packet_size-3);
                }
                else if(read_from_here[0]==2) { // START frame
                    (void) memcpy(START_CONTROL_PACKET, read_from_here, packet_size);
                    // START_CONTROL_PACKET = read_from_here;
                    file_size_size = read_from_here[2];
                    /* for(int j = 0; j < name_size; j++) {
                        new_filename[j] = read_from_here[3+j];
                    } */
                    file_ptr = fopen(filename, "a+"); // we want to append to file, if it already exists, or create new if it does not exist
                    printf("opened file for writing\n");
                    for(int j = 0; j < file_size_size; j++) {
                        printf("current size = %i\n", size);
                        size = (size << 8);
                        size += read_from_here[3+j];
                    }
                    printf("size was determined to be %i\n", size);
                }
                else if(read_from_here[0]==3) { // END frame
                    (void) memcpy(END_CONTROL_PACKET, read_from_here, packet_size);
                    // END_CONTROL_PACKET = read_from_here;
                    // END_CONTROL_PACKET[0] = 2;
                    unsigned char something_wrong = FALSE;

                    for(int j = 0; j < packet_size; j++) {
                        if(END_CONTROL_PACKET[j] != START_CONTROL_PACKET[j]) { // something must have happened; we must re-read
                            something_wrong = TRUE;
                        }
                        else {
                            continue;
                        }
                    }

                    if(something_wrong == TRUE) {
                        continue;
                    }
                    else { // there was no error; we can finish execution
                        printf("closing file\n");
                        loop = FALSE;
                    }
                }
                else { // there must have been an error; we must re-read
                    printf("shiiiii\n");
                    continue;
                }
            }
        }

        /* // we good, received everything correctly; we can close the file and the connection
        fclose(file_ptr);
        if(llclose(FALSE) < 0) {
            exit(-1);
        } */
    }

    return;
}
