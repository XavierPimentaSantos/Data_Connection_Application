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

    // to be used by the Receiver side
    unsigned char new_filename[50] = {0}; // adicionar tamanho aqui
    unsigned char read_from_here[MAX_PAYLOAD_SIZE] = {0}; // e aqui

    // to be used by Transmitter side
    unsigned char buffer[MAX_PAYLOAD_SIZE] = {0}; // e aqui
    // unsigned char *buffer_helper[MAX_PAYLOAD_SIZE] = {0}; // e aqui
    
    unsigned int size = 0;
    
    FILE *file_ptr;
    char ch;
    
    unsigned char START_CONTROL_PACKET[310] = {0};
    unsigned char END_CONTROL_PACKET[310] = {0};

    //calls for opening of port
    if(llopen(connection) < 0) {
        exit(-1);
    }

    // IF TRANSMITTER
    if(connection.role == LlTx) {
        file_ptr = fopen(filename, "r"); // load the file into file_ptr
	    printf("opened %s\n", filename);
	    
        struct stat st;
	    if(stat(filename, &st) == 0) {
            size = st.st_size;
        }
	    printf("read file ig, size is %i\n", size);

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
        
        // send the START_CONTROL_PACKET    
        // while(PROCEED_START == FALSE) {
        //     printf("sending %i bytes\n", scp_i);
        //     printf("data = 0x");
        //     for(int j = 0; j < scp_i; j++) {
        //     	printf("%02X", START_CONTROL_PACKET[j]);
        //     }
        //     printf("\n");
        //     if((f = llwrite(START_CONTROL_PACKET, scp_i)) > 0 /*== scp_i*/) {
        //         PROCEED_START = TRUE;
        //     }
        //     else {
        //         printf("failed to send the %i bytes, sent %i instead\n", scp_i, f);
        //         continue;
        //     }
        // }
	    // printf("sent the SCP\n");

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
                fseek(file_ptr, i, SEEK_SET);
            	(void) fread(buffer/*_helper*/+3, 1, bufSize-3, file_ptr/*+i*/);    
                can_seek = FALSE;
            }
            //printf("file_ptr = %i\n", file_ptr);
            else {
            	(void) fread(buffer/*_helper*/+3, 1, bufSize-3, file_ptr/*+i*/);
            }
            
            /* printf("Tx will send buf = 0x");
            for(int m = 0; m < bufSize; m++) {
            	printf("%02X", (unsigned char) buffer[m]);
            }
            printf("\n"); */
            // calls llwrite() for those bytes
            if(llwrite(buffer, bufSize) >= /*==*/ bufSize) {
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
        
        //reads from array
        printf("i am a %s\n", role);
        // fopen used to be here
        int data_size = 0;
        int packet_size;
        int file_size_size;
        int loop = TRUE;
        // int i = 0;

        while(loop == TRUE) {
            // call llread()
            packet_size = llread(read_from_here);
            if(packet_size == -1) {
                continue; // we ignore what already exists in the array and try again
                printf("ah deu merda aqui no llread\n");
            }
            else {
                // succeeded in receiving the data; use state machine to determine what to do with it
                printf("llread read %i bytes\n", packet_size);
                if(read_from_here[0]==1) { // data
                    data_size = (read_from_here[1]<<8);
                    data_size += read_from_here[2];
                    unsigned char copy_[MAX_PAYLOAD_SIZE];
                    (void) memcpy(copy_, read_from_here+3, data_size);
                    fwrite(copy_, sizeof read_from_here[0], data_size, file_ptr/*+i*/);
                    printf("succesfully wrote %i bytes to new file\n", data_size);
                    // i += data_size;
                    size -= data_size;
                }
                else if(read_from_here[0]==2) { // START frame
                    (void) memcpy(START_CONTROL_PACKET, read_from_here, 310);
                    // START_CONTROL_PACKET = read_from_here;
                    int name_size = read_from_here[2];
                    for(int j = 0; j < name_size; j++) {
                        new_filename[j] = read_from_here[3+j];
                    }
                    file_ptr = fopen(filename, "a+"); // we want to append to file, if it already exists, or create new if it does not exist
                    printf("opened file for writing\n");
                    file_size_size = read_from_here[name_size+4];
                    for(int j = 0; j < file_size_size; j++) {
                        size = size << 8;
                        size += read_from_here[data_size+5+j];
                    }
                    printf("asjdhaskgdasgd\n");
                }
                else if(read_from_here[0]==3) { // END frame
                    (void) memcpy(END_CONTROL_PACKET, read_from_here, 310);
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
                        loop = FALSE;
                    }
                }
                else { // there must have been an error; we must re-read
                    printf("shiiiii\n");
                    continue;
                }
            }
        }

        // we good, received everything correctly; we can close the file and the connection
        fclose(file_ptr);
        if(llclose(FALSE) < 0) {
            exit(-1);
        }
    }

    return;
}
