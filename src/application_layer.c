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
    connection.serialPort = serialPort; 
    connection.role = &role;

    //calls for opening of port
    if(llopen(connection) < 0) {
        exit(-1);
    }
    
    int size = 0;
    
    FILE* file_ptr;
    unsigned char ch;
    
    // IF TRANSMITTER

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

    unsigned char *START_CONTROL_PACKET;
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

    unsigned char *END_CONTROL_PACKET = START_CONTROL_PACKET;
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
        for(int j = 0; j < bufSize; j++) {
            buffer[j+3] = file_ptr[i + j];
        }
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

    //when llwrite() returns succesfully for the last time, call llclose() and finish execution
    if(llclose(FALSE) < 0) { // could be TRUE if we want to see the statistics
        exit(-1);
    }

    //IF RECEIVER
    
    unsigned char *read_from_here;
    //reads from array
    
    //when array ends, calls for llread()
    //if llread() returns 1, read from array into file, using state machine to determine type of data
    //if llread() returns -1, try again, ignoring what already exists in the array
    //when it reads the END control packet, calls for llclose() and finishes execution
}
