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
#include <stdlib.h>
#include <iostream> // library that contain basic input/output functions
#include <fstream>  // library that contains file input/output functions
using namespace std;

#ifdef __linux__
#define         DEVICE_PORT             "/dev/ttyO1"                         // tty01 for linux
#endif

int main()
{
    serialib LS;                                                            // Object of the serialib class
    int Ret,Mes;                                                                // Used for return values
    char buf[1];
    int count=0;
    char message[22];
    int speed = 0, index = 0, invalid = 0,
            signalStrength = 0, strengthWarning = 0, distance = 0;

    volatile int distances[360];
    for(int j=0;j<360;j++)
    {
        distances[j] = 0;
    }


    // Open serial port
    Ret=LS.Open(DEVICE_PORT,115200);                                        // Open serial link at 115200 bauds
    if (Ret!=1) {                                                           // If an error occured...
        printf ("Error while opening port. Permission problem ?\n");        // ... display a message ...
        return Ret;                                                         // ... quit the application
    }
    printf ("Serial port opened successfully !\n");


    while(count<5000000)
    {
        // Read a char from the serial device
        Ret=LS.Read(buf,1,5000);
        
        // wait for start byte then parse the message
        if(*buf== 0xfa )
        {            
            Mes = LS.Read(message,21,5000);

            index           = message[0];
            speed           = (message[2] << 8) | message[1];

            // parse four data packets
            for (int i=0;i<=3;i++)
            {
                invalid         = message[4+4*i] >> 7;
                strengthWarning = message[4+4*i] >> 6 & 0x01; 
                distance = ((message[4+4*i] & 0x3f) << 8) | message[3+4*i];
                signalStrength = ( message[6+4*i] << 8 ) | message[5+4*i];
                
                //printf("distance: %x | %x | %x | %x | %x a \n ",message[3],message[4],message[4] & 0x3f, (message[4] & 0x3f) << 8,distance);
                if(index-0xa0 == 0)
                    printf("Index: %x | Speed: %x | invalid: %x | distance: %d\n ",index,speed, invalid,distance);

                //printf("1: %d - 2: %d - 3: %d - 4: %d - 5: %d \r ",distances[1],distances[31],distances[61],distances[91],distances[121],distances[151]);
                


                // store based on index
                if(!invalid)
                {
                    //printf("index: %d\n",(index-0xa0)*4+i);    
                    distances[(index-0xa0)*4+i] = distance;
                }
            }
        }

        count++;
    }


    // Close the connection with the device

    LS.Close();

    for(int j=0;j<360;j++)
    {
        printf("%d : ", distances[j]);
    }

    ofstream fout("test.txt"); //opening an output stream for file test.txt
    /*checking whether file could be opened or not. If file does not exist or don't have write permissions, file
    stream could not be opened.*/
    if(fout.is_open())
    {
    //file opened successfully so we are here
    cout << "File Opened successfully!!!. Writing data from array to file" << endl;

        for(int i = 0; i<360; i++)
        {
            fout << distances[i]; //writing ith character of array in the file
            fout << ",";
        }
    cout << "Array data successfully saved into the file test.txt" << endl;
    }
    else //file could not be opened
    {
        cout << "File could not be opened." << endl;
    }
    return 0;
}


