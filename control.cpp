/**
Control Algorithm for collision avoidance

Inputs:     xv-11 lidar distance data
            desired pilot inputs to KK2 board 

Outputs:    corrected outputs to KK2 board

TODO

Goal for 0.1
working hackish porportional control

@author Miguel Rufino
**/

#include <stdio.h>
#include "serialib.h"
#include <stdlib.h>
#include <iostream> // library that contain basic input/output functions
#include <fstream>  // library that contains file input/output functions
using namespace std;
#include <pthread.h>
#include <unistd.h>
#include <prussdrv.h>
#include <pruss_intc_mapping.h>


///LIDAR DEFINES
#define DEVICE_PORT             "/dev/ttyO4"
volatile int distances[360];


///PRU DEFINES
#define PRU_NUM 0

static void *pru0DataMemory;
static unsigned int *pru0DataMemory_int, *period;


/// PWM DEFINES
// factor to turn # cycles into ms
#define PERIODCORRECT 216346

// how many cycles to run for
#define RUNTIME 10000000


/// CONTROL DEFINES
// porportional gain costant
#define KP 1

volatile int pwmSignals[4];
volatile int pwmCorrections[4];
#define CORRECTIONCAP 0;

// distance to obstacle before correction is applied
#define DISTFORCORRECT 2000;

// used for converting RC to DUTY CYCLE
#define MINDUTY 15000;
#define MAXDUTY 20000;
#define DUTYRANGE MAXDUTY-MINDUTY;

// conversion factor from duty cycle (in us) to RC input magnitude ([-100  to 100%]) 
#define DUTY2RC(x) ((x-MINDUTY)/DUTYRANGE *200 - 100)
#define RC2DUTY(x) ((x+100)*DUTYRANGE/200 + MINDUTY)


void *threadFunction(void *value){
    int pwmCorrected = 0;
    do {

        // read memory into pwmSignals for use in control
        int notimes = prussdrv_pru_wait_event (PRU_EVTOUT_1);
        pwmSignals[0] = (int)*(pru0DataMemory_int+2);
        pwmSignals[1] = (int)*(pru0DataMemory_int+3);
        pwmSignals[2] = (int)*(pru0DataMemory_int+4);
        pwmSignals[3] = (int)*(pru0DataMemory_int+5);

        // write corrected pwms to memory
        /*for (int i=0;i<=3;i++)
        {
            pwmCorrected = pwmSignals[i] - pwmCorrections[i];

            // make sure to "cap" correction   
            if (pwmCorrected<correctionCap)
                pwmCorrected = correctionCap;

            // store to memort
            *(pru0DataMemory_int+2+i) = DUTY2RC(pwmCorrected);
        }*/

      printf("Signals are Aileron: %f , Elevator: %f , Throttle: %f , Rudder: %f \r", DUTY2RC(pwmSignals[0]), DUTY2RC(pwmSignals[1]), DUTY2RC(pwmSignals[2]), DUTY2RC(pwmSignals[3]));

      prussdrv_pru_clear_event (PRU_EVTOUT_1, PRU0_ARM_INTERRUPT);
   } while (1);
}

void *lidar(void *value)
{
    serialib LS;                                                            // Object of the serialib class
    int Ret;
    char buf[1];
    char message[22];
    
    int speed = 0, index = 0, invalid = 0,
            signalStrength = 0, strengthWarning = 0, distance = 0;

    printf("Inside Lidar thread\n");

    // Open serial port
    Ret=LS.Open(DEVICE_PORT,115200);                                        // Open serial link at 115200 bauds
    if (Ret!=1) {                                                           // If an error occured...
        printf ("Error while opening port. Permission problem ?\n");        // ... display a message ...                                                        // ... quit the application
    }
    printf ("Serial port opened successfully !\n");

    
    while(1)
    {
        // Read a char from the serial device
        LS.Read(buf,1,5000);
        // wait for start bit then parse the message
        if(*buf== 0xfa )
        {           
            LS.Read(message,21,5000);

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
                /*if(index-0xa0 == 0)
                    printf("Index: %x | Speed: %x | invalid: %x | distance: %d\n ",index,speed, invalid,distance);*/

                printf("1: %d - 2: %d - 3: %d - 4: %d - 5: %d \r ",distances[0],distances[30],distances[60],distances[90],distances[120],distances[150]);

                // store based on index
                if(!invalid)
                {
                    //printf("index: %d\n",(index-0xa0)*4+i);    
                    distances[(index-0xa0)*4+i] = distance;
                }
            }
        }

    }
}

int main()
{

    float err, dist, dutyCycle, correctedPWM;

    pthread_t threadLidar, threadPWM;

    for(int j=0;j<360;j++)
    {
        distances[j] = 6000;
    }

    // start thread for Lidar measurements
    if(pthread_create(&threadLidar, NULL, &lidar, NULL)){
       printf("Failed to create thread!\n");
   }
    printf("Lidar Thread created\n");

   // start thread for handling pwm passthrough & correction
      if(pthread_create(&threadPWM, NULL, &threadFunction, NULL)){
       printf("Failed to create thread!");
   }
   printf("PWM Thread created\n");

   tpruss_intc_initdata pruss_intc_initdata = PRUSS_INTC_INITDATA;

   // Allocate and initialize memory
   prussdrv_init ();
   prussdrv_open (PRU_EVTOUT_0);
   prussdrv_open (PRU_EVTOUT_1);

   // Map PRU's INTC
   prussdrv_pruintc_init(&pruss_intc_initdata);

   // Copy data to PRU memory - different way
   prussdrv_map_prumem(PRUSS0_PRU0_DATARAM, &pru0DataMemory);
   pru0DataMemory_int = (unsigned int *) pru0DataMemory;
   // Use the first 4 bytes for the number of samples
   *pru0DataMemory_int = RUNTIME;

   // Period
   *(pru0DataMemory_int+6) = PERIODCORRECT;

   // Load and execute binary on PRU
   prussdrv_exec_program (PRU_NUM, "./measureChannels.bin");
   



     /********************/
     // Compute Errors   //
     /********************/
     for (int direction=0;direction<=7;direction++)
     {
         // compute error term 
         err = distance[direction]-distForCorrect
        
         // only bother with corrections if distance is within the correction distance
         if (err>0)
             error[direction] = err;
         else // else error = 0 || don't make corrections
             error[direction] = 0;
     }

     // compute derivative average of last N errors
     // compute Integral of last N errors

        
     /***********************/
     // Apply Corrections   //
     /***********************/
     // correct pilot's desired inputs
     // when error is at max it should result in 
     // complete reversal of direction

     for (int channel=0;channel<=3;channel++)
     {
         // Determine orientation of ultrasonic readings


         // correct based on error
         correctedPWM = PWM[channel] - ( * Kp);

         // make sure PWMs are within -100 to 100 range before writing to memory
         if (correctedPWM > 100)
             correctedPWM = 100;
         else if (correctedPWM < -100)
             correctedPWM = -100;

         PWM[channel] = correctedPWM;

     }





   printf("waiting for event\n");
   int n = prussdrv_pru_wait_event (PRU_EVTOUT_0);
   printf("PRU program completed, event number %d.\n", n);
   printf("The data that is in memory is:\n");
   printf("- the number of samples used is %d.\n", *pru0DataMemory_int);

   // number of loops = period
   float period1 = (float)*(pru0DataMemory_int+2);
   float period2 = (float)*(pru0DataMemory_int+3);
   float period3 = (float)*(pru0DataMemory_int+4);
   float period4 = (float)*(pru0DataMemory_int+5);
   printf("periods are Ch1: %f s , Ch2: %f s , Ch3: %f s , Ch4: %f s \n", period1, period2, period3, period4);

   /* Disable PRU and close memory mappings */
   prussdrv_pru_disable(PRU_NUM);
   prussdrv_exit ();
   return EXIT_SUCCESS;

}
