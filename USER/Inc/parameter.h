#ifndef __PARAMETER_H_
#define __PARAMETER_H_

#include "Compile.h"

/*System setting*/
#define SYS_REFV                                    5.0        //  unit:v      MCU VCC must be 5.0V or 3.3V         
#define SYSCLK_HSI_96MHz                            96000000   //  unit:Hz

#define PWMFREQ                                     10000      //  unit:Hz
#define DEAD_TIME                                   1.0        //  unit:us     determined by Hardware parameter   
#define SLOWLOOP_FREQ                               1000       //  unit:Hz

/*Board parameter setting*/
// Current Sampling
#define ADC_REFV                                    SYS_REFV
#define R_SHUNT                                     (0.15)     //  unit:ohm
#define CURRENT_OPA_GAIN                            (5.00)     //  OPA_GAIN  
#define I_MAX                                       (ADC_REFV/2/CURRENT_OPA_GAIN/R_SHUNT)  //  unit:A   Max current of Hardware         

// Voltage Sampling
#define UDC_MAX                                     (850.0)     //  unit:V      Max DC Voltage of Hardware 
#define U_MAX                                       (UDC_MAX/1.732)  // unit:V      Max Voltage 
#define DCBUS_OVER                                  (380.0)     //  unit:V        if DC Bus voltage over this, it will stop motor
#define DCBUS_UNDER                                 (220.0)     //  unit:V        if DC Bus voltage under this, it will stop motor

/*motor releated paremeter setting*/
//Motor parameter
#define Rs                                          18.1        //  unit:ohm
#define Ls                                          0.070      //  unit:H
#define POLEPAIRS                                   4.0        //  unit
#define SPEED_MAX                                   (5000.0)  //  unit:rpm    max speed of motor

#define MAX_DUTY                                    (0.95)    //   unit
#define D_PI_LIMIT                                  (170.0)    //   unit:V      Vd PI output limitation
#define Q_PI_LIMIT                                  (170.0)    //   unit:V      Vq PI output limitation
#define SPD_PI_LIMIT                                (1.23)     //   unit:A      Speed PI output limitation


/********************************************Motor backward running parameters********************************************/
#define IQ_ALIGN_BACK                               1.0
#define IQ_STARTUP_BACK                             0.4
#define FREEWHEEL_TIME_BACK                          10

#define M1_IQ_KP_BACK                               Q15(0.0915) //   KP Value of Q-axis Current loop,original value is 0.42
#define M1_IQ_KP_SHIFT_BACK                         (-2)      //   KP Shift Value of Q-axis Current loop
#define M1_IQ_KI_BACK                               Q15(0.0599) //   KI Value of Q-axis Current loop,original value is 0.015
#define M1_IQ_KI_SHIFT_BACK                         (-3)      //   KI Shift Value of Q-axis Current loop

#define M1_ID_KP_BACK                               Q15(0.0915) //   KP Value of D-axis Current loop,original value is 0.42
#define M1_ID_KP_SHIFT_BACK                         (-2)      //   KP Shift Value of D-axis Current loop
#define M1_ID_KI_BACK                               Q15(0.0599) //   KI Value of D-axis Current loop,original value is 0.015
#define M1_ID_KI_SHIFT_BACK                         (-3)      //   KI Shift Value of D-axis Current loop

#define M1_SPEED_KP_BACK                            Q15(0.061)
#define M1_SPEED_KP_SHIFT_BACK                      6
#define M1_SPEED_KI_BACK                            Q15(0.061) /*Q15(0.275)*/
#define M1_SPEED_KI_SHIFT_BACK                      -4

// track observer
#define M1_TO_KP_GAIN_BACK                               Q15(0.0915)  //   KP Value of track observer
#define M1_TO_KP_SHIFT_BACK                              (2)          //   KP Shift Value of track observer
#define M1_TO_KI_GAIN_BACK                               Q15(0.214)   //   KI Value of track observer
#define M1_TO_KI_SHIFT_BACK                              (-7)         //   KI Shift Value of track observer
#define M1_TO_THETA_GAIN_BACK                            Q15(0.533337)//   Theta gain value
#define M1_TO_THETA_SHIFT_BACK                           (-3)         //   Theta gain shift value

// run state
#define RAMP_UP_BACK                                     100.0     //  RPM/s            ramp of speed command increase
#define RAMP_DOWN_BACK                                   200.0     //  RPM/s            ramp of speed command decrease

#if (MOTOR_ALTERNATIVE_START_MODE == OPTION_PASSIVE)
/*State machine setting*/

// PreCharge stage
#define PRECHARGE_TIME                              100        //  unit:ms       precharge time setup

#define STOP_TO_RUN_SPEED                           300.0      //  unit:RPM      if speed command higher this, Switch from Stop to Run state
#define STARTUP_TO_SPIN_SPEED                       250.0      //  unit:RPM      if actual speed  higher this, Switch from Startup to Spin state
#define FREEWHEEL_SPEED                             250.0      //  unit:RPM      if speed command lower this,  Switch to Freewheel state

// Align state
#define IQ_ALIGN                                    1.0        //  unit:A        q axis current for align. previous value is 0.2A      
#define ALIGN_CURRENT_RAMP                          1.0        //  unit:A/s      ramp of  align current
#define ALIGN_TIME                                  2.0        //  unit:s        align time

// Startup state
#define IQ_STARTUP                                  1.0        //  unit:A        q axis current for startup      
#define MAXSTARTUP_SPEED                            300.0      //  unit:RPM      max startup speed 
#define STARTUP_SPEED_RAMP                          250.0      //  unit:RPM/s    ramp of startup speed 
#define SPEED_TO_THETA                              Q15(2*SPEED_MAX*POLEPAIRS/60.0/PWMFREQ) //unit: speed to theta Integral constant
#define STARTUP_TIME                                2.0        //  unit:s

// run state
#define RAMP_UP                                     250.0     //  RPM/s            ramp of speed command increase
#define RAMP_DOWN                                   150.0     //  RPM/s            ramp of speed command decrease

// freewheel state
#define FREEWHEEL_TIME                              2.0        //  unit:s           freewheel time
#define FAULTRELEASE_TIME                           20.0       //  unit:s           fault release time 


// CURRENT LOOP PI PARAMETER
#define M1_IQ_KP                                    Q15(0.22) //   KP Value of Q-axis Current loop,original value is 0.42
#define M1_IQ_KP_SHIFT                              (-2)      //   KP Shift Value of Q-axis Current loop
#define M1_IQ_KI                                    Q15(0.005) //   KI Value of Q-axis Current loop,original value is 0.015
#define M1_IQ_KI_SHIFT                              (-3)      //   KI Shift Value of Q-axis Current loop
#define M1_ID_KP                                    Q15(0.22) //   KP Value of D-axis Current loop,original value is 0.42
#define M1_ID_KP_SHIFT                              (-2)      //   KP Shift Value of D-axis Current loop
#define M1_ID_KI                                    Q15(0.005) //   KI Value of D-axis Current loop,original value is 0.015
#define M1_ID_KI_SHIFT                              (-3)      //   KI Shift Value of D-axis Current loop

// SPEED LOOP PI PARAMETER
#define M1_SPEED_KP                                 Q15(0.45)  //   KP Value of speed loop, original value is 0.95
#define M1_SPEED_KP_SHIFT                           (2)       //   KP Shift Value of speed loop
#define M1_SPEED_KI                                 Q15(0.45)  //   KI Value of speed loop, original value is 0.0.9
#define M1_SPEED_KI_SHIFT                           (-8)      //   KI Shift Value of speed loop

// SMO Observer
#define SMO_ERR_MAX                                 Q15(0.075)  //   Max value of SMO error, original value is 0.045,0.09
#define SLIDE_GIAN                                  Q15(0.4)  //   Kslide gain value ,original value is 0.045,0.09
#define IERR_GIAN                                	  Q15(0.5)  //   Current gain value, original value is 0.09,0.18
#define IERR_GIAN_SHIFT                             (1)       //   Shift Value of Current gain

// track observer
#define M1_TO_KP_GAIN                               Q15(0.125)  //   KP Value of track observer
#define M1_TO_KP_SHIFT                              (2)          //   KP Shift Value of track observer
#define M1_TO_KI_GAIN                               Q15(0.29)   //   KI Value of track observer
#define M1_TO_KI_SHIFT                              (-7)         //   KI Shift Value of track observer
#define M1_TO_THETA_GAIN                            Q15(0.533337)//   Theta gain value
#define M1_TO_THETA_SHIFT                           (-3)         //   Theta gain shift value

#else
/*State machine setting*/

// PreCharge stage
#define PRECHARGE_TIME                              100        //  unit:ms       precharge time setup

#define STOP_TO_RUN_SPEED                           300.0      //  unit:RPM      if speed command higher this, Switch from Stop to Run state
#define STARTUP_TO_SPIN_SPEED                       250.0      //  unit:RPM      if actual speed  higher this, Switch from Startup to Spin state
#define FREEWHEEL_SPEED                             250.0      //  unit:RPM      if speed command lower this,  Switch to Freewheel state

// Align state
#define IQ_ALIGN                                    1.0        //  unit:A        q axis current for align. previous value is 0.2A      
#define ALIGN_CURRENT_RAMP                          1.0        //  unit:A/s      ramp of  align current
#define ALIGN_TIME                                  2.0        //  unit:s        align time

// Startup state
#define IQ_STARTUP                                  1.0        //  unit:A        q axis current for startup      
#define MAXSTARTUP_SPEED                            300.0      //  unit:RPM      max startup speed 
#define STARTUP_SPEED_RAMP                          250.0      //  unit:RPM/s    ramp of startup speed 
#define SPEED_TO_THETA                              Q15(2*SPEED_MAX*POLEPAIRS/60.0/PWMFREQ) //unit: speed to theta Integral constant
#define STARTUP_TIME                                2.0        //  unit:s

// run state
#define RAMP_UP                                     250.0     //  RPM/s            ramp of speed command increase
#define RAMP_DOWN                                   250.0     //  RPM/s            ramp of speed command decrease

// freewheel state
#define FREEWHEEL_TIME                              2.0        //  unit:s           freewheel time
#define FAULTRELEASE_TIME                           20.0       //  unit:s           fault release time 

// CURRENT LOOP PI PARAMETER
#define M1_IQ_KP                                    Q15(0.22) //   KP Value of Q-axis Current loop,original value is 0.42
#define M1_IQ_KP_SHIFT                              (-2)      //   KP Shift Value of Q-axis Current loop
#define M1_IQ_KI                                    Q15(0.005) //   KI Value of Q-axis Current loop,original value is 0.015
#define M1_IQ_KI_SHIFT                              (-3)      //   KI Shift Value of Q-axis Current loop
#define M1_ID_KP                                    Q15(0.22) //   KP Value of D-axis Current loop,original value is 0.42
#define M1_ID_KP_SHIFT                              (-2)      //   KP Shift Value of D-axis Current loop
#define M1_ID_KI                                    Q15(0.005) //   KI Value of D-axis Current loop,original value is 0.015
#define M1_ID_KI_SHIFT                              (-3)      //   KI Shift Value of D-axis Current loop

// SPEED LOOP PI PARAMETER
#define M1_SPEED_KP                                 Q15(0.45)  //   KP Value of speed loop, original value is 0.95
#define M1_SPEED_KP_SHIFT                           (2)       //   KP Shift Value of speed loop
#define M1_SPEED_KI                                 Q15(0.45)  //   KI Value of speed loop, original value is 0.0.9
#define M1_SPEED_KI_SHIFT                           (-8)      //   KI Shift Value of speed loop

// SMO Observer
#define SMO_ERR_MAX                                 Q15(0.075)  //   Max value of SMO error, original value is 0.045,0.09
#define SLIDE_GIAN                                  Q15(0.4)  //   Kslide gain value ,original value is 0.045,0.09
#define IERR_GIAN                                	  Q15(0.5)  //   Current gain value, original value is 0.09,0.18
#define IERR_GIAN_SHIFT                             (1)       //   Shift Value of Current gain

// track observer
#define M1_TO_KP_GAIN                               Q15(0.125)  //   KP Value of track observer
#define M1_TO_KP_SHIFT                              (2)          //   KP Shift Value of track observer
#define M1_TO_KI_GAIN                               Q15(0.29)   //   KI Value of track observer
#define M1_TO_KI_SHIFT                              (-7)         //   KI Shift Value of track observer
#define M1_TO_THETA_GAIN                            Q15(0.533337)//   Theta gain value
#define M1_TO_THETA_SHIFT                           (-3)         //   Theta gain shift value


#endif

//Application layer control max speed
#define M1_MAX_SPEED                                1400
#define M1_MAX_SPEED_PU                             Q15(M1_MAX_SPEED/SPEED_MAX)

#define M1_MIN_SPEED_PU                             Q15(STOP_TO_RUN_SPEED/SPEED_MAX)

#endif
