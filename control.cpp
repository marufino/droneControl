/**
Control Algorithm for collision avoidance

Inputs:     xv-11 lidar distance data
            desired pilot inputs to KK2 board

Outputs:    corrected outputs to KK2 board

TODO

Goal for 0.1
working hackish porportional control

Changes to be made:
---Add a SIGINT Handler
while(1) fix/wait()/semaphore
Ultrasonic interrupt/semaphore
Confirm Serial.read blocking
Semaphore for threads for safe R/W

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
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/mman.h>
#include <netinet/in.h>
#include <netdb.h>
#include <string.h>

///LIDAR DEFINES
#define DEVICE_PORT             "/dev/ttyO1"
int distances[360];

/// TELEMETRY DEFINES
bool volatile sendMessage = true;

static void *pru0DataMemory, *pru1DataMemory;
static unsigned int *pru0DataMemory_int, *pru1DataMemory_int, *pruSharedMemory, *period;
static unsigned int *sharedMemory;

/// PWM DEFINES
// factor to turn # cycles into ms
#define PERIODCORRECT 216346

// how many cycles to run for
#define RUNTIME 10000000000


/// CONTROL DEFINES
// porportional gain costant
#define KP 0

float volatile pwmSignals[4],ultraDistances[8];
int volatile pwmCorrection[4] = {30,30,30,30};
float volatile pwmCorrected[4] = {0};
#define LOWESTPWM -100
#define HIGHESTPWM 100


// distance to obstacle before correction is applied
#define DISTFORCORRECT 2000

// used for converting RC to DUTY CYCLE
static double minduty[4] = {11189.0,11150.0,12460.0,10630.0};
static double maxduty[4] = {20355.0, 21320.0, 21640.0, 21810.0};
static double dutyrange[4] = {maxduty[0]-minduty[0],maxduty[1]-minduty[1],maxduty[2]-minduty[2],maxduty[3]-minduty[3]};

// Signal handler to catch CTRL-C and Exit
static volatile int keepRunning = 1;

void  INThandler(int sig)
{
     keepRunning = 0;
}

// conversion factor from duty cycle (in us) to RC input magnitude ([-100  to 100%])
double duty2rc(float x, int d)
{
  return (((double)x-minduty[d])/dutyrange[d])*200.0 - 100;
}

float rc2duty(float x, int d)
{
  return (((double)x+100.0)*dutyrange[d]/200.0 + minduty[d]);
}

void *PWMThread(void *value){
    
    do {
        // read memory into pwmSignals for use in control
        int notimes = prussdrv_pru_wait_event (PRU_EVTOUT_0);

		for(int i=0;i<=9;i++)
        {
            ultraDistances[i] = (float)*((unsigned int *)(pru1DataMemory+0x110+i*0x10));
            ultraDistances[i] = ultraDistances[i]*0.0000872103;
        }
        printf("ULTRASONIC: %f : %f : %f : %f : %f : %f : %f : %f \r", ultraDistances[0], ultraDistances[1], ultraDistances[2], ultraDistances[3], ultraDistances[4], ultraDistances[5], ultraDistances[6], ultraDistances[7], ultraDistances[8]);


        pwmSignals[0] = (float)*(pru0DataMemory_int+2);
        pwmSignals[1] = (float)*(pru0DataMemory_int+3);
        pwmSignals[2] = (float)*(pru0DataMemory_int+4);
        pwmSignals[3] = (float)*(pru0DataMemory_int+5);

	// store to memory
	for (int i=0;i<=3;i++)
        {
	    *(pru0DataMemory_int+(7+i)) = rc2duty(pwmCorrected[i],i);
        }
        
        // compute corrected pwms
        for (int i=0;i<=3;i++)
        {
            if (i==0){
              pwmCorrected[i] = duty2rc(pwmSignals[i],i) + pwmCorrection[1] - pwmCorrection[3];
            }
            else if (i==1){
              pwmCorrected[i] = duty2rc(pwmSignals[i],i) + pwmCorrection[0] - pwmCorrection[2];
            }
            else{
              pwmCorrected[i] = duty2rc(pwmSignals[i],i);
            }

            // make sure to "cap" correction
            /*if (pwmCorrected[i] < LOWESTPWM)
                pwmCorrected[i] = LOWESTPWM;

            if (pwmCorrected[i] > HIGHESTPWM)
                pwmCorrected[i] = HIGHESTPWM;
			*/
        }

      //printf("Signals are Aileron: %f , %f ,  %f\r", duty2rc(pwmSignals[0],0),pwmCorrected[0], rc2duty(pwmCorrected[0],0)  );

      //printf("Signals are Aileron: %f , Rudder: %f , Throttle: %f , Elevator: %f \r", duty2rc(pwmSignals[0],0), duty2rc(pwmSignals[1],1), duty2rc(pwmSignals[2],2), duty2rc(pwmSignals[3],3));

      prussdrv_pru_clear_event (PRU_EVTOUT_0, PRU0_ARM_INTERRUPT);
      usleep(100);
   } while (1);
}

void *USThread(void *value)
{
    while(1)
    {
        for(int i=0;i<=9;i++)
        {
            ultraDistances[i] = (float)*(pru1DataMemory_int+0x10*i);
            ultraDistances[i] = ultraDistances[i]*0.0000872103;
        }
        printf("ULTRASONIC: %f : %f : %f : %f : %f : %f : %f \r", ultraDistances[0], ultraDistances[1], ultraDistances[2], ultraDistances[3], ultraDistances[4], ultraDistances[5], ultraDistances[6], ultraDistances[7]);
        usleep(1000);
    }
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
                if(index-0xa0 == 0 || index-0xc0 == 0)
                {
                  //printf("Index: %x | Speed: %x | invalid: %x | distance: %d\n ",index,speed, invalid,distance);*/

                  // send telemetry data
                  sendMessage = true;
                }
                // store based on index
                if(!invalid)
                {
                    //printf("index: %d\n",(index-0xa0)*4+i);
                    distances[(index-0xa0)*4+i] = distance;
                }
            }

            //printf("1: %d - 2: %d - 3: %d - 4: %d \r ",distances[1],distances[91],distances[181],distances[271]);

            /*if(!invalid)
            {
              printf("%d\n",(index-0xa0)*4);
            }*/
        }else{
            //if the start bit is not detected, yield the processor for 8us?
            usleep(8);

        }

    }
}


void *controlThread(void *value)
{
   float err = 0;
   printf("Starting control algorithm\n");
   while(1)
   {
     //*******************/
     // Compute Errors   //
     //*******************/
     for (int direction=0;direction<=3;direction++)
     {
         // compute error term 100 = max correction
         err = (DISTFORCORRECT - distances[(360/4)*direction+1])*50/DISTFORCORRECT;

         // only bother with corrections if distance is within the correction distance

         if (err>0)
         {
             pwmCorrection[direction] = KP * err;
         }
         else // else error = 0 || don't make corrections
         {
             pwmCorrection[direction] = 0;
         }
     }
   usleep(1000);
   }
}

void error(const char *msg)
{
    perror(msg);
    //exit(0);
}

void *telemetryThread(void *value)
{
    int sockfd, portno, n;
    struct sockaddr_in serv_addr;
    struct hostent *server;
    char* argv[3] = {"","",""};
    argv[1] = "192.168.4.7";
    argv[2] = "520";

    printf("%s:%s\n",argv[1],argv[2]);

    char buffer[2000];
    char temp[4];

    portno = atoi(argv[2]);
    sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd < 0)
        error("ERROR opening socket");
    server = gethostbyname(argv[1]);
    if (server == NULL) {
        fprintf(stderr,"ERROR, no such host\n");
        exit(0);
    }
    bzero((char *) &serv_addr, sizeof(serv_addr));
    serv_addr.sin_family = AF_INET;
    bcopy((char *)server->h_addr,
         (char *)&serv_addr.sin_addr.s_addr,
         server->h_length);
    serv_addr.sin_port = htons(portno);
    if (connect(sockfd,(struct sockaddr *) &serv_addr,sizeof(serv_addr)) < 0)
        error("ERROR connecting");

    while(1)
    {
      while(!sendMessage); //add a sleep here?
      bzero(buffer,2000);

      // add distances from LIDAR
      for (int i=0;i<360;i++)
      {
        sprintf(temp,"%d,",distances[i]);
        strcat(buffer,temp);
      }

      // add inputs
      for (int i=0;i<4;i++)
      {
        sprintf(temp,"%f,",duty2rc(pwmSignals[i],i));
        strcat(buffer,temp);
      }

      // add corrections
      for (int i=0;i<4;i++)
      {
        sprintf(temp,"%d,",pwmCorrection[i]);
        strcat(buffer,temp);
      }


      strcat(buffer,"\n");
      n = write(sockfd,buffer,strlen(buffer));
      if (n < 0)
           error("ERROR writing to socket");
      sendMessage = false;
      usleep(5)
    }

    //close(sockfd);
}

int main()
{
    signal(SIGINT, INThandler);

    pthread_t threadLidar, threadPWM, threadUS, threadControl, threadTelemetry;

    // init volatile globals used
    for(int j=0;j<360;j++)
    {
        distances[j] = 6000;
    }

    for(int j=0;j<4;j++)
    {
      pwmSignals[j]=0;
    }

   // create PRU Interrupt controller
   tpruss_intc_initdata pruss_intc_initdata = PRUSS_INTC_INITDATA;

   // Allocate and initialize memory
   prussdrv_init ();
   prussdrv_open (PRU_EVTOUT_0);
   prussdrv_open (PRU_EVTOUT_1);

   // Map PRU's INTC
   prussdrv_pruintc_init(&pruss_intc_initdata);

   // Copy data to PRU0 memory
   prussdrv_map_prumem(PRUSS0_PRU0_DATARAM, &pru0DataMemory);
   pru0DataMemory_int = (unsigned int *) pru0DataMemory;

   //shared mem 0x00012000

   // Copy data to PRU1 memory
   prussdrv_map_prumem(PRUSS0_PRU1_DATARAM, &pru1DataMemory);
   sharedMemory = (unsigned int *) (0x4A300000);
   pru1DataMemory_int = (unsigned int *) pru1DataMemory;

   // zero the memory we'll be using
   for(int j=0;j<10;j++)
    {
      *(pru0DataMemory_int+j) = 0;
      *(pru1DataMemory_int+j) = 0;
    }

   // number of program cycles to run for
   *pru0DataMemory_int = RUNTIME;

   // signal period (used by PRU program to compute duty cycle)
   *(pru0DataMemory_int+6) = PERIODCORRECT;

   // Load and execute binary on PRU that measures and generates PWM signals
   prussdrv_exec_program (0, "./measureChannels.bin");
   prussdrv_exec_program (1, "./Ultrasonic.bin");

    // start thread for Lidar measurements
    if(pthread_create(&threadLidar, NULL, &lidar, NULL)){
       printf("Failed to create thread!\n");
    }
    printf("Lidar Thread created\n");

   // start thread for handling pwm passthrough & correction
    if(pthread_create(&threadPWM, NULL, &PWMThread, NULL)){
       printf("Failed to create thread!");
    }
    printf("PWM Thread created\n");
 /*
    if(pthread_create(&threadUS, NULL, &USThread, NULL)){
       printf("Failed to create thread!");
    }
   printf("US Thread created\n");*/

    // start thread for calculating control corrections
    if(pthread_create(&threadControl, NULL, &controlThread, NULL)){
       printf("Failed to create thread!\n");
    }
   printf("Control Thread created\n");

   // start thread for telemetry communication
    if(pthread_create(&threadTelemetry, NULL, &telemetryThread, NULL)){
       printf("Failed to create thread!\n");
    }
   printf("Telemetry Thread created\n");

   while(keepRunning);

   /* Disable PRUs and close memory mappings */
   prussdrv_pru_disable(0);
   prussdrv_pru_disable(1);
   prussdrv_exit ();
   return EXIT_SUCCESS;

}
