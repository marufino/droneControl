//=============================================================================
// File: Ultrasonic.p 
// Desc: Read Ultrasonics
// Vers: 1.0
//
.origin 0
.entrypoint ORIGIN
//#include "ReadPWM.hp" 

// Address for the eCap (eCap)
#define ECAP_TIMER_ADDR         0x30000
#define ECCTL2_ADDR             0x30028
#define ECCTL2_VALUE            0x100000        // flag to start time
#define TRIGGER_PULSE_US        20
#define INS_PER_US              200
#define INS_PER_LOOP            2
 #define GPIO0                  0x44E07000
#define GPIO1                   0x48042000
#define GPIO2                   0x481ac000
#define GPIO_CLEARDATAOUT       0x190
#define GPIO_SETDATAOUT         0x194
#define GPIO_DATAOUT            0x138
#define TRIGGER_1               1<<22           //P8_27 GPIO2_22
#define TRIGGER_2               1<<25           //P8_30 GPIO2_25
#define TRIGGER_3               1<<10           //P8_31 GPIO0_10
#define TRIGGER_4               1<<11           //P8_32 GPIO0_11
#define TRIGGER_5               1<<9            //P8_33 GPIO0_9
#define TRIGGER_6               1<<17           //P8_34 GPIO2_17
#define TRIGGER_7               1<<8            //P8_35 GPIO0_8
#define TRIGGER_8               1<<16           //P8_36 GPIO2_16
#define TRIGGER_9               1<<14           //P8_37 GPIO2_14
#define TRIGGER_10              1<<15           //P8_38 GPIO2_15
#define GPIO2_3                 1<<3
#define GPIO2_5                 1<<5
#define GPIO2_4                 1<<4
#define TRIGGER_COUNT       (TRIGGER_PULSE_US * INS_PER_US) / INS_PER_LOOP
#define SAMPLE_DELAY_1MS    (10000000 * INS_PER_US) / INS_PER_LOOP
#define SAMPLE_DELAY_100MS    100*(1000 * INS_PER_US) / INS_PER_LOOP
#define PRU0_R31_VEC_VALID  32
#define PRU_EVTOUT_0        3
#define PRU_EVTOUT_1        4
#define delay r10



// each channel's data are stored in data, start from 00, each channel use 16 bytes, 
// 00 - 03      // flag
// 04 - 07      // last down time
// 08 - 0b      // current down 
// 0c - 0f      // reserved

// pwm input data store in share memory, start from 0x110
// each up time and pulse width are stored, each channel use 16 bytes
// 00 - 03      // flag
// 04 - 07      // pulse width
// 08 - 0b      // frame width 


//PWM In Registers
// r1   temp
// r2   Last frame
// r3   Current Frame
// r4   Current counter
// r5   Last up counter
// r6   Current down  counter
// r7   down counter
// r8   Last Trigger
// r9   Sensor complete tracking
// r10  current trigger group
// r11  triggering
// r12  timeout measure
// r13   GPIO0 store
ORIGIN:
    //Enable the OCP master port
    LBCO    r0, C4, 4, 4
    CLR     r0, r0, 4
    SBCO    r0, C4, 4, 4 

 JMP READ_PWM_INIT      

// macro for process input, it will count the time for high level, as well as whole frame width
.macro ProcessInput     
.mparam     dataOffset, bitNum, sensorNum
    QBBS PROCESS_HIGH, r3, bitNum

PROCESS_LOW:    
    QBBC PROCESS_END, r2, bitNum    // low to low, nothing change
// now process high to low,
    MOV     r1, dataOffset +8
    SBBO    r4, r1, 0, 4                // write down time to data memeory
    SET     r9, sensorNum
JMP PROCESS_END

PROCESS_HIGH:
    QBBS PROCESS_END, r2, bitNum    // high to high, nothing change
// write up timer, for low to high
//WRITE_UP_TIMER:
    MOV     r1, dataOffset + 4    // load data address in DATA to r1
    LBBO    r5, r1, 0, 8            // load last down, current down to r5, r6
    SUB     r7, r6, r5              // current up - current down = pulse width
    SBBO    r4, r1, 0, 4            // update last up time
    MOV     r1, dataOffset+ 0x110
    SBBO    r7, r1, 0, 4            // save pulse width to share data
JMP PROCESS_END
PROCESS_END:
.endm

// macro for triggering sensors
.macro TriggerSensors
TRIGGER:
    MOV     r0, TRIGGER_COUNT       //store length of the trigger pulse delay
    QBEQ    TRIGGER_UP_2, r14, 1
    //Group 1
    MOV     r13, GPIO2 | GPIO_SETDATAOUT        //set the triggers high
    MOV     r11, TRIGGER_1
    SBBO    r11, r13, 0, 4
    //MOV     r11, TRIGGER_2
    //SBBO    r11, r13, 0, 4
    MOV     r11, TRIGGER_6
    SBBO    r11, r13, 0, 4
    MOV     r11, TRIGGER_8
    SBBO    r11, r13, 0, 4
    MOV     r11, TRIGGER_9
    SBBO    r11, r13, 0, 4
    JMP     TRIGGERING
TRIGGER_UP_2:
    //Group 2
    MOV     r13, GPIO2 | GPIO_SETDATAOUT        //set the triggers high
    MOV     r11, TRIGGER_10
    SBBO    r11, r13, 0, 4
    MOV     r13, GPIO0 | GPIO_SETDATAOUT        //set the triggers high
    //MOV     r11, TRIGGER_3
    //SBBO    r11, r13, 0, 4
    MOV     r11, TRIGGER_4
    SBBO    r11, r13, 0, 4
    MOV     r11, TRIGGER_5
    SBBO    r11, r13, 0, 4
    MOV     r11, TRIGGER_7
    SBBO    r11, r13, 0, 4

TRIGGERING:                         // delay for 10us   
    SUB     r0, r0, 1               // decrement loop counter
    QBNE    TRIGGERING, r0, 0       // repeat loop unless zero
    QBEQ    TRIGGER_DOWN_2, r14, 1
    //Group 1
    MOV     r13, GPIO2 | GPIO_CLEARDATAOUT  //set the triggers low
    MOV     r11, TRIGGER_1
    SBBO    r11, r13, 0, 4
    //MOV     r11, TRIGGER_2
    //SBBO    r11, r13, 0, 4
    MOV     r11, TRIGGER_6
    SBBO    r11, r13, 0, 4
    MOV     r11, TRIGGER_8
    SBBO    r11, r13, 0, 4
    MOV     r11, TRIGGER_9
    SBBO    r11, r13, 0, 4
    MOV     r14, 1
    JMP     TRIGGER_END
TRIGGER_DOWN_2:
    //Group 2
    MOV     r13, GPIO2 | GPIO_CLEARDATAOUT
    MOV     r11, TRIGGER_10
    SBBO    r11, r13, 0, 4
    MOV     r13, GPIO0 | GPIO_CLEARDATAOUT  //set the triggers low
    //MOV     r11, TRIGGER_3
    //SBBO    r11, r13, 0, 4
    MOV     r11, TRIGGER_4
    SBBO    r11, r13, 0, 4
    MOV     r11, TRIGGER_5
    SBBO    r11, r13, 0, 4
    MOV     r11, TRIGGER_7
    SBBO    r11, r13, 0, 4
    MOV     r14, 0
TRIGGER_END:
.endm
READ_PWM_INIT:
// start the timer
// start the timer
    MOV     r1, ECCTL2_ADDR         // copy ecap config register address
    MOV     r2, ECCTL2_VALUE        // copy the same value to current frame Reg
    SBBO    r2, r1, 0, 4            // set register of ECCTL2, the timer will start now

    MOV     r2, r31                 // copy current input to last frame Reg
    MOV     r3, r2                  // copy the same value to current frame Reg
    MOV     r12, 5000000
// copy the current timer counter
    MOV     r1, ECAP_TIMER_ADDR     // copy address of ecap to r1
    LBBO    r4, r1, 0, 4            // copy current timer count value
    //MOV     r10.b1, 0x1
TRIGGER_SENSORS:
    MOV     r9,0
    MOV     delay, 300000
RESET_DELAY:
    SUB     delay, delay, 1
    QBNE RESET_DELAY, delay, 0

TriggerSensors                      //Call TriggerSensors macro
    MOV     r1, ECAP_TIMER_ADDR     // copy address of ecap to r1
    LBBO    r8, r1, 0, 4            // copy content of ecap to r8
RUN_PWM: 
// create snap shot by copy input and timer counter
    MOV     r3, r31                 // copy current input to current frame Reg

// copy the current timer counter
    MOV     r1, ECAP_TIMER_ADDR     // copy address of ecap to r1
    LBBO    r4, r1, 0, 4            // copy content of ecap r0
    

    
    ProcessInput    0x00, 10, 0       //  1st  Sensor         //PRU1_r31_10   P8-28
    ProcessInput    0x10, 9,  1       //  2nd  Sensor         //PRU1_r31_9    P8-29
    ProcessInput    0x20, 6,  2       //  3rd  Sensor         //PRU1_r31_6    P8-39
    ProcessInput    0x30, 7,  3       //  4nd  Sensor         //PRU1_r31_7    P8-40
    ProcessInput    0x40, 4,  4       //  5rd  Sensor         //PRU1_r31_4    P8-41
    ProcessInput    0x50, 5,  5       //  6th  Sensor         //PRU1_r31_5    P8-42
    ProcessInput    0x60, 2,  6       //  7th  Sensor         //PRU1_r31_2    P8-43  
    ProcessInput    0x70, 3,  7       //  8th  Sensor         //PRU1_r31_3    P8-44   
    ProcessInput    0x80, 0,  8       //  9th  Sensor         //PRU1_r31_0    P8-45   
    ProcessInput    0x90, 1,  9       //  10th  Sensor        //PRU1_r31_1    P8-46      

UPDATE_LAST:        // update last input frame 
    MOV     r2, r3
    //QBEQ TRIGGER_SENSORS, r9, 1
    SUB r4, r4, r8
    QBGT TRIGGER_SENSORS, r12, r4
JMP RUN_PWM
HALT
