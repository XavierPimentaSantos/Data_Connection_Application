// Application layer protocol implementation

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <sys/stat.h>
#include <sys/types.h>

#include "application_layer.h"

#include "link_layer.h"

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
    
    unsigned long int size = 0;
    FILE *file_ptr;
    unsigned char START_CONTROL_PACKET[310] = {0};
    unsigned char END_CONTROL_PACKET[310] = {0};

    if(llopen(connection) < 0) {
        exit(-1);
    }

    // IF TRANSMITTER
    if(connection.role == LlTx) {
        unsigned char buffer[MAX_PAYLOAD_SIZE+3] = {0};
        file_ptr = fopen(filename, "r");
	    printf("opened %s\n", filename);
	    
        struct stat st;
	    if(stat(filename, &st) == 0) {
            size = st.st_size;
        }

        unsigned char size_size = 0;
        unsigned int size_2 = size;
        while(size_2 != 0) {
            size_2 = size_2 >> 8;
            size_size++;
        }
        
        int scp_i = 0;
        START_CONTROL_PACKET[scp_i++] = 2;
        START_CONTROL_PACKET[scp_i++] = 0;
        START_CONTROL_PACKET[scp_i++] = size_size;
        for(int l = 1; l <= size_size; l++) {
            START_CONTROL_PACKET[scp_i + l - 1] = (unsigned char) (size >> ((size_size - l) * 8));
        }
        scp_i = scp_i + size_size;
        START_CONTROL_PACKET[scp_i++] = 1;
        int keep_scp_i = scp_i++;
        START_CONTROL_PACKET[keep_scp_i] = 0;
        for(int k = 0; filename[k] != '\0'; k++) {
            START_CONTROL_PACKET[scp_i++] = filename[k];
            START_CONTROL_PACKET[keep_scp_i]++;
        }
        END_CONTROL_PACKET[0] = 3;
        for(int j = 1; j < scp_i; j++) {
            END_CONTROL_PACKET[j] = START_CONTROL_PACKET[j];
        }

        if(llwrite(START_CONTROL_PACKET, scp_i) < scp_i) {
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

            unsigned char bufSize_lhs = (unsigned char) (bufSize>>8);
            unsigned char bufSize_rhs = (unsigned char) bufSize;

            buffer[0] = 1;
            buffer[1] = bufSize_lhs;
            buffer[2] = bufSize_rhs;

            if(can_seek==TRUE) {
                (void) fseek(file_ptr, i, SEEK_SET);
            	(void) fread(buffer+3, 1, bufSize, file_ptr);    
                can_seek = FALSE;
            }
            else {
            	(void) fread(buffer+3, 1, bufSize, file_ptr);
            }

            if(llwrite(buffer, bufSize+3) >= bufSize+3) {
                i += bufSize;
                can_seek = TRUE;
            }
            else {
                return;
            }
        }

        if(llwrite(END_CONTROL_PACKET, scp_i) < scp_i) {
            return;
        }

        fclose(file_ptr);
        if(llclose(TRUE) < 0) { // could be TRUE if we want to see the statistics
            return;
        }    
    }

    //IF RECEIVER
    else {
        unsigned char read_from_here[MAX_PAYLOAD_SIZE] = {0};
        int data_size = 0;
        int packet_size;
        int file_size_size;
        int loop = TRUE;

        while(loop == TRUE) {
            memset(read_from_here, 0, MAX_PAYLOAD_SIZE);
            packet_size = llread(read_from_here);
            if(packet_size == -1) {
                continue;
            }
            else if(packet_size == 0) {
                if(size == 0 && llclose(TRUE) > 0) {
                    return;
                }
                else {
                    exit(-1);
                }
            }
            else {
                if(read_from_here[0]==1) {
                    data_size = (read_from_here[1]<<8);
                    data_size += read_from_here[2];
                    unsigned char copy_[MAX_PAYLOAD_SIZE];
                    (void) memcpy(copy_, read_from_here+3, packet_size-3);
                    fwrite(copy_, sizeof read_from_here[0], packet_size-3, file_ptr);
                    size -= (packet_size-3);
                }
                else if(read_from_here[0]==2) {
                    (void) memcpy(START_CONTROL_PACKET, read_from_here, packet_size);
                    file_size_size = read_from_here[2];
                    file_ptr = fopen(filename, "a+");
                    for(int j = 0; j < file_size_size; j++) {
                        size = (size << 8);
                        size += read_from_here[3+j];
                    }
                }
                else if(read_from_here[0]==3) {
                    (void) memcpy(END_CONTROL_PACKET, read_from_here, packet_size);
                    unsigned char something_wrong = FALSE;
                    for(int j = 0; j < packet_size; j++) {
                        if(END_CONTROL_PACKET[j] != START_CONTROL_PACKET[j]) {
                            something_wrong = TRUE;
                        }
                        else {
                            continue;
                        }
                    }
                    if(something_wrong == TRUE) {
                        continue;
                    }
                    else {
                        loop = FALSE;
                    }
                }
                else {
                    continue;
                }
            }
        }
    }

    return;
}
