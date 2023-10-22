// Application layer protocol implementation

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

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
    *connection.serialPort = serialPort; 
    connection.role = *role;

    //calls for opening of port
    if(llopen(connection) < 0) {
        exit(-1);
    }
    
    int size = 0;
    
    FILE* file_ptr;
    unsigned char ch;
    
    unsigned char *START_CONTROL_PACKET;
    unsigned char *END_CONTROL_PACKET;

    // IF TRANSMITTER
    if(*role == LlTx) {
        file_ptr = fopen(*filename, "r"); // load the file into file_ptr

        do {
            ch = fgetc(file_ptr);
            size++;
        } while (ch != EOF); 

        // create the static Control packets
        unsigned char size_size = 0;
        unsigned char size_2 = size;
        while(size_2 != 0) {
            size_2 = size_2 >> 8;
            size_size++;
        }
        
        int scp_i = 0;

        START_CONTROL_PACKET[scp_i++] = 2; // it is a START Control packet
        START_CONTROL_PACKET[scp_i++] = 0; // section: size of file
        START_CONTROL_PACKET[scp_i++] = size_size; // size of size of file
        START_CONTROL_PACKET[scp_i] = size; // size of file
        scp_i = scp_i + size_size -1;
        START_CONTROL_PACKET[scp_i++] = 1;
        int keep_scp_i = scp_i++;
        START_CONTROL_PACKET[keep_scp_i] = 0;
        for(int k = 0; filename[k] != '\0'; k++) {
            START_CONTROL_PACKET[scp_i++] = filename[k];
            START_CONTROL_PACKET[keep_scp_i]++;
        }
        // scp_i is the size of START_CONTROL_PACKET and END_CONTROL_PACKET

        END_CONTROL_PACKET = START_CONTROL_PACKET;
        END_CONTROL_PACKET[0] = 3; // ECP is essentially SCP but with 3 intead of 2 on the Control byte

        unsigned char PROCEED_START = FALSE;
        unsigned char PROCEED_END = FALSE;

        // sends the START_CONTROL_PACKET    
        while(PROCEED_START == FALSE) {
            if(llwrite(START_CONTROL_PACKET, scp_i) == scp_i) {
                PROCEED_START = TRUE;
            }
            else {
                continue;
            }
        }

        int i = 0;
        int bufSize;

        while(i != size) {
            if(size - i < MAX_PAYLOAD_SIZE) {
                bufSize = size - i;
            }
            else {
                bufSize = MAX_PAYLOAD_SIZE;
            }

            unsigned char bufSize_lhs = (unsigned char) bufSize>>8;
            unsigned char bufSize_rhs = (unsigned char) bufSize;

            //loads bufSize bytes of that into array
            unsigned char *buffer;
            buffer[0] = 1;
            buffer[1] = bufSize_lhs;
            buffer[2] = bufSize_rhs;
            unsigned char *buffer_helper;
            // for(int j = 0; j < bufSize; j++) {
            //     buffer[j+3] = file_ptr[i + j];
            // }
            (void) fwrite(buffer_helper, sizeof buffer[1], bufSize, file_ptr+i);
            // calls llwrite() for those bytes
            if(llwrite(buffer, bufSize) == bufSize) {
                i += bufSize; // no errors, we can send another group of bytes
            }
            // if there was an error, we resend the bytes
        } // do this until there is no more data to send

        // send the END_CONTROL_PACKET
        while(PROCEED_END == FALSE) {
            if(llwrite(END_CONTROL_PACKET, scp_i) == scp_i) {
                PROCEED_END = TRUE;
            }
            else {
                continue;
            }
        }

        //when llwrite() returns succesfully for the last time, close the file, call llclose() and finish execution
        fclose(file_ptr);
        if(llclose(FALSE) < 0) { // could be TRUE if we want to see the statistics
            exit(-1);
        }    
    }

    //IF RECEIVER
    else {
        unsigned char *read_from_here;
        //reads from array
        char *new_filename;
        // fopen used to be here
        int data_size;
        int packet_size;
        int file_size_size;
        int loop = TRUE;
        // int i = 0;

        while(loop == TRUE) {
            // call llread()
            packet_size = llread(read_from_here);
            if(packet_size == -1) {
                continue; // we ignore what already exists in the array and try again
            }
            else {
                // succeeded in receiving the data; use state machine to determine what to do with it
                if(read_from_here[0]==1) { // data
                    data_size = read_from_here[1]<<8;
                    data_size += read_from_here[2];
                    fwrite(read_from_here+3, sizeof read_from_here[0], data_size, file_ptr/*+i*/);
                    // i += data_size;
                    size -= data_size;
                }
                else if(read_from_here[0]==2) { // START frame
                    START_CONTROL_PACKET = read_from_here;
                    int name_size = read_from_here[2];
                    for(int j = 0; j < name_size; j++) {
                        new_filename[j] = read_from_here[3+j];
                    }
                    new_filename[name_size] = '_copy';
                    file_ptr = fopen(*filename, 'a+'); // we want to append to file, if it already exists, or create new if it does not exist
                    file_size_size = read_from_here[name_size+4];
                    for(int j = 0; j < file_size_size; j++) {
                        size = size << 8;
                        size += read_from_here[data_size+5+j];
                    }
                }
                else if(read_from_here[0]==3) { // END frame
                    END_CONTROL_PACKET = read_from_here;
                    END_CONTROL_PACKET[0] = 2;
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
