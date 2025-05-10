/***************************************************************
   Motor driver function definitions - by James Nugen
   *************************************************************/

#define M1_C1 0
#define M1_C2 1
#define M2_C1 2
#define M2_C2 3

#define PWM_freq 5000

#define LEFT_MOTOR_FORWARD   25
#define LEFT_MOTOR_BACKWARD  26
#define RIGHT_MOTOR_FORWARD  32
#define RIGHT_MOTOR_BACKWARD 33

void initMotorController();
void setMotorSpeeds(int leftSpeed, int rightSpeed);
