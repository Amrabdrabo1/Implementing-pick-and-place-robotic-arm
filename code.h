#include <stdio.h>
#include <math.h>
#include <wiringPi.h>
#include <wiringSerial.h>
#include <string.h>
#include <pthread.h>
#include <unistd.h>
#include <time.h>

#ifndef MS3_angles_H_
#define MS3_angles_H_

/*********************** RaspberryPi pins *****************************/ 
#define POWER_SWITCH 17
#define OBJECT_SENSOR 24
#define GRIPPER_IR 22
#define GRIPPER_PRESSURE 23
#define DC_MOTOR 18
#define DC_ENCODER 25

/************************ global variables *****************************/
#define ANGLE_GRIPPER_ON 150
#define ANGLE_GRIPPER_OFF 180
#define HOME_POSITION 0
#define PICK_POSITION 1
#define ASSEMBLE_POSITION 2
#define PWM_MIN 0
#define PWM_MAX 1024
#define CONVEYOR_OFF 0
#define CONVEYOR_ON 1
#define scheduler SCHED_FIFO
#define priority_robo_Arm 1
#define priority_conv 1



const double Kp = 0.05;
const double Ki = 0.03;
const double delta_t = 0.1;

/***************************** states **********************************/
extern void* roboticArm_off(void); 
extern void* roboticArm_waiting(void); 
extern void* roboticArm_home(void); 
extern void* roboticArm_pick(void); 
extern void* roboticArm_assemble(void); 
extern void* gripper_on(void); 
extern void* gripper_off(void);
extern void* robotic_arm(void* args);
extern void* conveyor_off(void);
extern void* conveyor_constant(void);
extern void* conveyor_control(void);
extern void* conveyer(void* args);


/*************************** functions *********************************/
extern void* servo(unsigned char num, unsigned char angle);
int calcSpeed(void);
void* set_Pins(void);
void* setPins(void);


/************************ Angles form MATLAB ***************************/
    // These angles is just random angles to test the algorithm and 
    // will be changed when we test the hardware
    int home_angles [4] = {90,45,120,0};
           int home_pick [4][15] = {{90,90,90,90,90,90,90,90,90,90,90,90,90,90,90},
                   {45,50,55,60,65,70,75,80,85,90,93,96,99,102,105},
                   {120,115,110,106,103,100,96,93,90,86,83,80,76,73,70},
                   {0,0,0,0,0,0,0,0,0,0,0,0,5,10,15}};
           int home_assemble_0 [4][15] = {{90,85,80,75,70,65,60,55,50,45,40,35,30,30,30},
                   {45,50,55,60,65,70,75,80,85,90,93,96,99,102,105},
                   {120,115,110,105,100,95,90,85,80,75,70,65,60,55,50},
                   {0,0,0,0,0,0,0,0,0,0,0,5,10,15,20}};
           int home_assemble_1 [4][15] = {{90,85,80,75,70,65,60,55,50,45,40,35,30,30,30},
                   {45,50,55,60,65,70,75,80,85,90,93,96,99,102,105},
                   {120,115,110,105,100,95,90,85,80,75,70,65,60,55,50},
                   {0,0,0,0,0,0,0,0,0,0,0,5,10,15,20}};
			int home_assemble_2 [4][15] = {{90,85,80,75,70,65,60,55,50,45,40,35,30,30,30},
                   {45,50,55,60,65,70,75,80,85,90,93,96,99,102,105},
                   {120,115,110,105,100,95,90,85,80,75,70,65,60,55,50},
                   {0,0,0,0,0,0,0,0,0,0,0,5,10,15,20}};
			int home_assemble_3 [4][15] = {{90,85,80,75,70,65,60,55,50,45,40,35,30,30,30},
                   {45,50,55,60,65,70,75,80,85,90,93,96,99,102,105},
                   {120,115,110,105,100,95,90,85,80,75,70,65,60,55,50},
                   {0,0,0,0,0,0,0,0,0,0,0,5,10,15,20}};
			int home_assemble_4 [4][15] = {{90,85,80,75,70,65,60,55,50,45,40,35,30,30,30},
                   {45,50,55,60,65,70,75,80,85,90,93,96,99,102,105},
                   {120,115,110,105,100,95,90,85,80,75,70,65,60,55,50},
                   {0,0,0,0,0,0,0,0,0,0,0,5,10,15,20}};
			int home_assemble_5 [4][15] = {{90,85,80,75,70,65,60,55,50,45,40,35,30,30,30},
                   {45,50,55,60,65,70,75,80,85,90,93,96,99,102,105},
                   {120,115,110,105,100,95,90,85,80,75,70,65,60,55,50},
                   {0,0,0,0,0,0,0,0,0,0,0,5,10,15,20}};
				                                             

#endif /* MS3_angles_H_ */