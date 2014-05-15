/*!
 \file      Example1.cpp

 \brief     Example code source for class serialib.
            This example open the device on ttyACM0.
            (USB to RS232 converter under linux).
            If the opening is successful, it sends the AT command
            and waits for a string being received from the device.
            After 5 seconds, if no valid data are received from the
            device, reception is giving up.

 \author    Philippe Lucidarme (University of Angers) <serialib@googlegroups.com>
 \version   1.2
 \date      05/01/2011
 */

#include <stdio.h>
#include "serialib.h"


#if defined (_WIN32) || defined( _WIN64)
#define         DEVICE_PORT             "COM1"                               // COM1 for windows
#endif

#ifdef __linux__
#define         DEVICE_PORT             "/dev/tty01"                         // tty01 for linux
#endif


int main()
{
    serialib LS;                                                            // Object of the serialib class
    int Ret;                                                                // Used for return values
    char Buffer[128];
    char buf;




    // Open serial port

    Ret=LS.Open(DEVICE_PORT,115200);                                        // Open serial link at 115200 bauds
    if (Ret!=1) {                                                           // If an error occured...
        printf ("Error while opening port. Permission problem ?\n");        // ... display a message ...
        return Ret;                                                         // ... quit the application
    }
    printf ("Serial port opened successfully !\n");


    // Read a char from the serial device
    Ret=LS.ReadChar(buf,5000);  // read a char to buf, timeout of 5 seconds                              
                                                                            // The final character of the string must be a line feed ('\n')
    if (Ret>0)                                                              // If a string has been read from, print the string
        printf ("String read from serial port : %c",buf);
    else
        printf ("TimeOut reached. No data received !\n");                   // If not, print a message.



    // Close the connection with the device

    LS.Close();

    return 0;
}


