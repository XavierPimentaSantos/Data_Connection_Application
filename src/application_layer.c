// Application layer protocol implementation

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
    connection.serialPort = &serialPort; // não funciona, aparentemente serialPort não pode ser um const
    connection.role = &role;

    //calls for opening of port

    //IF TRANSMITTER
    //loads 1024 bytes of that into array
    //calls llwrite() for those bytes
    //when llwrite() returns 1, loads another 1024 bytes and repeats
    //if llwrite() returns -1, tries again with the same info
    //repeat this process until it finishes

    //when llwrite() returns succesfully for the last time, call llclose() and finish execution

    //IF RECEIVER
    //reads from array
    //when array ends, calls for llread()
    //if llread() returns 1, read from array into file, using state machine to determine type of data
    //if llread() returns -1, try again, ignoring what already exists in the array
    //when it reads the END control packet, calls for llclose() and finishes execution
}
