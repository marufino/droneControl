// Accurately measure duty cycle of PWM signal from 4 different channels and store it in
// a memory location accessible from an external program
// Generate PWMs based on the data at those memory locations
// Written by Miguel Rufino

.origin 0               // offset of start of program in PRU memory
.entrypoint START       // program entry point used by the debugger

#define INS_PER_US            200
#define INS_PER_LOOP          2
#define PRU0_R31_VEC_VALID    32
#define PRU_EVTOUT_0          3
#define PRU_EVTOUT_1          4       

// r1 : iteration counter
// r0 : measurement storage location
// r3 : channel 1 timer
// r4 : channel 2 timer
// r5 : channel 3 timer
// r6 : channel 4 timer

// r11 : channel 1 pwm out
// r12 : channel 2 pwm out
// r13 : channel 3 pwm out
// r14 : channel 4 pwm out

// r16 : signal period


START:
   // Read number of samples to read
   MOV    r0, 0x00000000                  //load the memory location, number of samples
   LBBO   r1, r0, 0, 4                    //load the value into memory - keep r1
   MOV    r0, 0x00000008                  // going to write the result to this address 
   LBBO   r16, r0, 16, 4                  // load signal period (1/frequency) 51Hz(?) for Turnigy

   MOV    r3, 1                        // r3 will store the echo pulse width for channel 1
   SBBO   r3, r0, 0, 4                    // initialize to 0
   MOV    r4, 1                        // r4 will store the echo pulse width for channel 2
   SBBO   r4, r0, 4, 4                    // initialize to 0
   MOV    r5, 1                        // r5 will store the echo pulse width for channel 3
   SBBO   r5, r0, 8, 4                    // initialize to 0
   MOV    r6, 1                        // r6 will store the echo pulse width for channel 4
   SBBO   r6, r0, 12, 4                   // initialize to 0                          

   MOV    r11, 100000
   MOV    r12, 100000
   MOV    r13, 100000
   MOV    r14, 100000

   MOV r31.b0, 0

   MEASURE:

      CHANNEL1:               
         QBBS ADDCHAN1, r31.t5               // If r31.t5 is high then count (measuring echo pulse width)
                                            
            QBEQ NOPULSE1, r3, 0             // if pulse has not started do nothing
         
               SBBO   r3, r0, 0, 4           // if pulse is done (not high) store to memory
               MOV    r3, -1                 // reset channel counter ( -1 since ADDCHAN adds 1 after)
               //MOV R31.b0, PRU0_R31_VEC_VALID | PRU_EVTOUT_1 // generate interrupt



         ADDCHAN1:
            ADD r3, r3, 1                    // Increment channel counter

         NOPULSE1:

      CHANNEL2:
         QBBS ADDCHAN2, r31.t3               // If r31.t3 is high then count (measuring echo pulse width)
                                            
            QBEQ NOPULSE2, r4, 0             // if pulse has not started do nothing
         
               SBBO   r4, r0, 4, 4           // if pulse is done (not high) store to memory
               MOV    r4, -1                 // reset channel counter ( -1 since ADDCHAN adds 1 after)
               //MOV R31.b0, PRU0_R31_VEC_VALID | PRU_EVTOUT_1 // generate interrupt

         ADDCHAN2:
            ADD r4, r4, 1                    // Increment channel counter

         NOPULSE2:

      CHANNEL3:
         QBBS ADDCHAN3, r31.t1               // If r31.t1 is high then count (measuring echo pulse width)
                                            
            QBEQ NOPULSE3, r5, 0             // if pulse has not started do nothing
         
               SBBO   r5, r0, 8, 4           // if pulse is done (not high) store to memory
               MOV    r5, -1                 // reset channel counter ( -1 since ADDCHAN adds 1 after)
               //MOV R31.b0, PRU0_R31_VEC_VALID | PRU_EVTOUT_1 // generate interrupt

         ADDCHAN3:
            ADD r5, r5, 1                    // Increment channel counter

         NOPULSE3:

      CHANNEL4:
         QBBS ADDCHAN4, r31.t2               // If r31.t2 is high then count (measuring echo pulse width)
                                            
            QBEQ NOPULSE4, r6, 0             // if pulse has not started do nothing
         
               SBBO   r6, r0, 12, 4           // if pulse is done (not high) store to memory
               MOV    r6, -1                 // reset channel counter ( -1 since ADDCHAN adds 1 after)
               MOV R31.b0, PRU0_R31_VEC_VALID | PRU_EVTOUT_0 // generate interrupt

         ADDCHAN4:
            ADD r6, r6, 1                    // Increment channel counter

         NOPULSE4:


   GENERATE:

      GEN1:
         QBNE NOCHANGE1,r11,0                // if timer isn't 0 skip ( signal doesn't need to be changed)
                                             // else flip signal
         QBBS MAKELOW1,r30.t15                // if high make low

            MAKEHIGH1:                          // if low
               SET r30.t15                      // make high
               LBBO r11,r0,20,4                  // load latest duty cycle for channel
               QBA NOCHANGE1                    

            MAKELOW1:
               CLR r30.t15                      // make low 
               LBBO r11,r0,20,4                  // load latest duty cycle for channel
               SUB r11,r16,r11                  // set timer = period - duty cycle

         NOCHANGE1:
         SUB r11,r11,1                          // decrement counter


      GEN2:
         QBNE NOCHANGE2,r12,0                // if timer isn't 0 skip ( signal doesn't need to be changed)
                                             // else flip signal
         QBBS MAKELOW2,r30.t14                // if high make low

            MAKEHIGH2:                          // if low
               SET r30.t14                      // make high
               LBBO r12,r0,24,4                  // load latest duty cycle for channel
               QBA NOCHANGE2                    

            MAKELOW2:
               CLR r30.t14                      // make low 
               LBBO r12,r0,24,4                  // load latest duty cycle for channel
               SUB r12,r16,r12                  // set timer = period - duty cycle

         NOCHANGE2:
         SUB r12,r12,1                          // decrement counter


      GEN3:
         QBNE NOCHANGE3,r13,0                // if timer isn't 0 skip ( signal doesn't need to be changed)
                                             // else flip signal
         QBBS MAKELOW3,r30.t7                // if high make low

            MAKEHIGH3:                           // if low
               SET r30.t7                      // make high
               LBBO r13,r0,28,4                  // load latest duty cycle for channel
               QBA NOCHANGE3                    

            MAKELOW3:
               CLR r30.t7                      // make low 
               LBBO r13,r0,28,4                  // load latest duty cycle for channel
               SUB r13,r16,r13                  // set timer = period - duty cycle

         NOCHANGE3:
         SUB r13,r13,1                          // decrement counter


      GEN4:
         QBNE NOCHANGE4,r14,0                // if timer isn't 0 skip ( signal doesn't need to be changed)
                                             // else flip signal
         QBBS MAKELOW4,r30.t0                // if high make low

            MAKEHIGH4:                           // if low
               SET r30.t0                      // make high
               LBBO r14,r0,32,4                  // load latest duty cycle for channel
               QBA NOCHANGE4                    

            MAKELOW4:
               CLR r30.t0                      // make low 
               LBBO r14,r0,32,4                  // load latest duty cycle for channel
               SUB r14,r16,r14                  // set timer = period - duty cycle

         NOCHANGE4:
         SUB r14,r14,1                          // decrement counter



   ITERATIONS:
      //MOV R31.b0, PRU0_R31_VEC_VALID | PRU_EVTOUT_0 // generate interrupt      
      QBA   MEASURE


