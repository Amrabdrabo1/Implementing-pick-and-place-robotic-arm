#include "MS4.h"

int current_position = 10; // any value 
int assemble_done = 0;
int conveyor_state = CONVEYOR_OFF;
pthread_mutex_t Resources, Resources1;
pthread_cond_t cond_v,cond_v1;


int main(int argc , char*argv[]){
    //Declare threads ids' and their arributes objects
    pthread_t thread_robo_arm, thread_conv;
    pthread_attr_t attr_robo_arm, attr_conv;

    // Initialize attribute objects, each attribute object is used to configure the
    // attributes of a thread upon its creation
    pthread_attr_init(&attr_robo_arm);
    pthread_attr_init(&attr_conv);

    //Initialize Mutex
    pthread_mutex_init(&Resources, NULL);
    pthread_mutex_init(&Resources1, NULL);

    //Initialize Condition variables
    pthread_cond_init(&cond_v, NULL);
    pthread_cond_init(&cond_v1,NULL);


    // Set the attributes objects to set the threads to be joinable
    pthread_attr_setdetachstate(&attr_robo_arm, PTHREAD_CREATE_JOINABLE);
    pthread_attr_setdetachstate(&attr_conv, PTHREAD_CREATE_JOINABLE);

    // Set the attributes objects to set the threads to use the scheduler
    // defined in the attribute object
    pthread_attr_setinheritsched(&attr_robo_arm, PTHREAD_EXPLICIT_SCHED);
    pthread_attr_setinheritsched(&attr_conv, PTHREAD_EXPLICIT_SCHED);

    // Set the attribute object scheduler which is FIFO criteria
    pthread_attr_setschedpolicy(&attr_robo_arm, scheduler); 
    pthread_attr_setschedpolicy(&attr_conv, scheduler);

    // Set the scheduling priority of each attribute object
    struct sched_param param_robo_arm, param_conv;

    param_robo_arm.sched_priority = priority_robo_Arm;
    pthread_attr_setschedparam(&attr_robo_arm, &param_robo_arm);
    param_conv.sched_priority= priority_conv;
    pthread_attr_setschedparam(&attr_conv, &param_conv);

    // Create threads, assign the modified attribute objects, assign the
    // threads to the corresponding function to be executed
    pthread_create(&thread_robo_arm , &attr_robo_arm , robotic_arm , NULL);
    pthread_create(&thread_conv , &attr_conv , conveyer , NULL);

    // Force the main method to wait for the threads to finish their execution
    pthread_join(thread_robo_arm,NULL);
    pthread_join(thread_conv,NULL);

    printf("\n Main Program exited normally\n");
    pthread_mutex_destroy(&Resources);

    pthread_cond_destroy(NULL);
    pthread_exit(NULL);
    
    return 0;

}

// Conveyer
void* conveyer(void *args){
    wiringPiSetupGpio();
    setPins();           
    while(1){
        printf("*****************************************\n");  
        printf("Power switch: %d\n", digitalRead(POWER_SWITCH));
        printf("Object sensor: %d\n", digitalRead(OBJECT_SENSOR));
        if(digitalRead(POWER_SWITCH)){
            if(!digitalRead(OBJECT_SENSOR)){
                conveyor_constant();
            }
            else if(conveyor_state == CONVEYOR_ON && digitalRead(OBJECT_SENSOR)){
                pthread_mutex_lock(&Resources1);
                conveyor_control();
                pthread_cond_signal(&cond_v1);
                pthread_mutex_unlock(&Resources1);
            }
            else if(conveyor_state == CONVEYOR_OFF && digitalRead(OBJECT_SENSOR)){
                
                conveyor_off();
                
            }
        }
        else{
            conveyor_off();
        }
        delay(2000);
    }
}

void* setPins(){ // set pins for conveyer
    //pinMode(POWER_SWITCH,INPUT);
    //pinMode(OBJECT_SENSOR,INPUT);
    pinMode(DC_ENCODER,INPUT);
    pinMode(DC_MOTOR,PWM_OUTPUT);

}

void* conveyor_off(){
    // The conveyor velocity is zero
    printf("Current state: conveyor_off\n");
    conveyor_state = CONVEYOR_OFF;
    pwmWrite(DC_MOTOR,PWM_MIN);
    
    
}

void* conveyor_constant(){
    // The conveyor velcoty is max and costant
    pthread_cond_wait(&cond_v,&Resources);
    printf("Current state: conveyor_constant\n");
    pthread_mutex_lock(&Resources1);
    conveyor_state = CONVEYOR_ON;
    pwmWrite(DC_MOTOR,PWM_MAX);
    pthread_mutex_unlock(&Resources1);
}

void* conveyor_control(){
    // The conveyor velocity is decreasing
    // until zero using pi controller
    printf("Current state: conveyor_control\n");
    double e_integral = 0;
    double speed_desired = 0;
    double e = 1;
    int speed_actual;
    int pwm;
    while(e > 0.1){
        
        speed_actual = calcSpeed();
        e = speed_desired - speed_actual;
        e_integral = e_integral + e*delta_t;
        pthread_mutex_lock(&Resources1);
        pwm = (int)(Kp*e + Ki*e_integral);
        if (pwm < PWM_MIN){
            pwm = PWM_MIN;
        }
        else if(pwm > PWM_MAX){
            pwm = PWM_MAX;
        }
        pwmWrite(DC_MOTOR,pwm);
    }
    conveyor_state = CONVEYOR_OFF;
    pthread_mutex_unlock(&Resources1);
}

int calcSpeed(){
    int result = 0;
    for(int i = 0; i<100;i++){
        if((i == 50 && result == 50)||(i == 50 && result == 0)){
            // 50 iterations without change
            result = 0;
            break;
        }
        if(digitalRead(DC_ENCODER)){
            result +=1;
        }
    }
    return result;
}

// Robotic_Arm code
void* robotic_arm(void *args){
    wiringPiSetupGpio();
    set_Pins();
    while(1){
        printf("*****************************************\n");
        printf("Current position: %d\n", current_position);
        printf("Power switch: %d\n", digitalRead(POWER_SWITCH));
        printf("Object sensor: %d\n", digitalRead(OBJECT_SENSOR));
        printf("Gripper IR: %d\n", digitalRead(GRIPPER_IR));
        printf("Gripper pressure: %d\n", digitalRead(GRIPPER_PRESSURE));
        if(digitalRead(POWER_SWITCH)){
            if (current_position == HOME_POSITION){
                if(digitalRead(OBJECT_SENSOR) && !digitalRead(GRIPPER_IR) 
                && !digitalRead(GRIPPER_PRESSURE)){
                    roboticArm_pick();
                }
                else if(digitalRead(GRIPPER_IR) && digitalRead(GRIPPER_PRESSURE)){
                    roboticArm_assemble();
                }
                else{
                    roboticArm_waiting();
                }
            }
            else if (current_position == PICK_POSITION){
                if(digitalRead(OBJECT_SENSOR) && digitalRead(GRIPPER_IR)
                && !digitalRead(GRIPPER_PRESSURE)){
                    gripper_on();
                }
                if(digitalRead(OBJECT_SENSOR) && digitalRead(GRIPPER_IR)
                && digitalRead(GRIPPER_PRESSURE)){
                    roboticArm_assemble();
                }  
            }
            else if (current_position == ASSEMBLE_POSITION){
                if(digitalRead(GRIPPER_IR) && digitalRead(GRIPPER_PRESSURE)){
                    gripper_off();
                }
                if (digitalRead(GRIPPER_IR) && !digitalRead(GRIPPER_PRESSURE)){
                    roboticArm_home();
                }
            }
            else{
                roboticArm_home();
            }
        }
        else{
            roboticArm_off();
        }
        delay(2000);
    }
}

void* set_Pins(){ // set pins for Robotic Arm
    pinMode(POWER_SWITCH,INPUT);
    pinMode(OBJECT_SENSOR,INPUT);
    pinMode(GRIPPER_PRESSURE,INPUT);
    pinMode(GRIPPER_IR,INPUT);
}

void* servo(unsigned char num, unsigned char angle){
    // send angles to arduino
    int fd;
    if((fd = serialOpen("/dev/ttyACM0",9600)) < 0)return 0;
    //printf("serial test start ...\n"); 
    serialPutchar(fd,num);
    // delay(10);
    serialPutchar(fd,angle);
    // delay(10);
    // serialPuts(fd,"Hello World!!!\n");
    serialClose(fd);
}

void* roboticArm_off(){
    printf("Current state: roboticArm_off\n");
}

void* roboticArm_waiting(){
    printf("Current state: roboticArm_waiting\n");
}

void* roboticArm_home(){
    printf("Current state: roboticArm_home\n");
    // inverse kinematics
    int angles [4];
    memcpy(angles,home_angles,sizeof(angles));
    pthread_mutex_lock(&Resources);
    for (int i = 0;i<4;i++){
        servo(i,(char)(angles[i]));
        delay(20);
    }
    // update postion
    current_position = HOME_POSITION;
    pthread_mutex_unlock(&Resources);
}

void* roboticArm_pick(){
    pthread_cond_wait(&cond_v1,&_);
    printf("Current state: roboticArm_pick\n");
    // task space straight line 
    int task_traj [11][4];
    memcpy(task_traj,home_pick,sizeof(task_traj));
    pthread_mutex_lock(&Resources);
    for(int i=14;i>=0;i--){
		for(int j=3;j>=0;j--){
            servo(j,(char)(task_traj[i][j]));
            delay(20);
        }
    }
    // update postion
    current_position = PICK_POSITION;
    
    pthread_mutex_unlock(&Resources);
}

void* roboticArm_assemble(){
    printf("Current state: roboticArm_assemble\n");
    // task space straight line 
    pthread_mutex_lock(&Resources);
    switch (assemble_done)
    {
    case 0:
        // 0 blocks
        for(int i=14;i>=0;i--){
			for(int j=3;j>=0;j--){
                servo(j,(char)(home_assemble_0[i][j]));
                delay(20);
            }
        }
        break;
    case 1:
        // 1 blocks
        for(int i=14;i>=0;i--){
			for(int j=3;j>=0;j--){
                servo(j,(char)(home_assemble_1[i][j]));
                delay(20);
            }
        }
        break;
    case 2:
        // 2 blocks
        for (int i = 0;i<11;i++){
            for(int j = 0;j<4;j++){
                servo(j,(char)(home_assemble_2[i][j]));
                delay(20);
            }
        }
        break;
    case 3:
        // 3 blocks
        for(int i=14;i>=0;i--){
			for(int j=3;j>=0;j--){
                servo(j,(char)(home_assemble_3[i][j]));
                delay(20);
            }
        }
        break;
    case 4:
        // 4 blocks
        for(int i=14;i>=0;i--){
			for(int j=3;j>=0;j--){
                servo(j,(char)(home_assemble_4[i][j]));
                delay(20);
            }
        }
        break;
    case 5:
        // 5 blocks
        for(int i=14;i>=0;i--){
			for(int j=3;j>=0;j--){
                servo(j,(char)(home_assemble_5[i][j]));
                delay(20);
            }
        }
        break;
    default:
        break;
    }
    // update postion
    current_position = ASSEMBLE_POSITION;
    assemble_done +=1;
    pthread_cond_signal(&cond_v);
    pthread_mutex_unlock(&Resources);
}

void* gripper_on(){
    printf("Current state: gripper_on\n");
    servo(4,ANGLE_GRIPPER_ON);
}

void* gripper_off(){
    printf("Current state: gripper_off\n");
    servo(4,ANGLE_GRIPPER_OFF);
}
