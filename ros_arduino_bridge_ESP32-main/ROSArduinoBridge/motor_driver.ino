#include <Arduino.h>

/***************************************************************
   Motor driver definitions

   ESP32 to L298N Motor Driver Connections:
   - GPIO 25 -> LEFT_MOTOR_FORWARD  -> IN1 of L298N
   - GPIO 26 -> LEFT_MOTOR_BACKWARD -> IN2 of L298N
   - GPIO 32 -> RIGHT_MOTOR_FORWARD -> IN3 of L298N
   - GPIO 33 -> RIGHT_MOTOR_BACKWARD-> IN4 of L298N
   - L298N ENA -> Connect to 5V (or GPIO 14 for variable speed)
   - L298N ENB -> Connect to 5V (or GPIO 27 for variable speed)
   - L298N VCC -> 7-12V power supply
   - L298N GND -> ESP32 GND
   
   ESP32 to Encoder Connections:
   - GPIO 34 -> LEFT_ENCODER_A  -> Channel A of left encoder
   - GPIO 35 -> LEFT_ENCODER_B  -> Channel B of left encoder
   - GPIO 36 -> RIGHT_ENCODER_A -> Channel A of right encoder
   - GPIO 39 -> RIGHT_ENCODER_B -> Channel B of right encoder
   - Encoder VCC to 3.3V
   - Encoder GND to ESP32 GND

   Motor to L298N Connections:
   - Left Motor +  -> OUT1 of L298N
   - Left Motor -  -> OUT2 of L298N
   - Right Motor + -> OUT3 of L298N
   - Right Motor - -> OUT4 of L298N

   Add these definitions to your main sketch (ROSArduinoBridge.ino):
   #define LEFT_MOTOR_FORWARD     25
   #define LEFT_MOTOR_BACKWARD    26
   #define RIGHT_MOTOR_FORWARD    32
   #define RIGHT_MOTOR_BACKWARD   33
   #define LEFT_ENCODER_A         34
   #define LEFT_ENCODER_B         35
   #define RIGHT_ENCODER_A        36
   #define RIGHT_ENCODER_B        39
   #define PWM_freq              1000  // PWM frequency in Hz

   *************************************************************/

void initMotorController() {
  // Set motor control pins as outputs
  pinMode(RIGHT_MOTOR_BACKWARD, OUTPUT);
  pinMode(LEFT_MOTOR_BACKWARD, OUTPUT);
  pinMode(RIGHT_MOTOR_FORWARD, OUTPUT);
  pinMode(LEFT_MOTOR_FORWARD, OUTPUT);

  // Initialize PWM for motor control
  ledcAttach(LEFT_MOTOR_BACKWARD, PWM_freq, 8);  // IN2 of left motor
  ledcAttach(LEFT_MOTOR_FORWARD, PWM_freq, 8);   // IN1 of left motor
  ledcAttach(RIGHT_MOTOR_BACKWARD, PWM_freq, 8); // IN4 of right motor
  ledcAttach(RIGHT_MOTOR_FORWARD, PWM_freq, 8);  // IN3 of right motor
}

void setMotorSpeeds(int leftSpeed, int rightSpeed) {
  // Constrain speed values to prevent issues
  leftSpeed = constrain(leftSpeed, -255, 255);
  rightSpeed = constrain(rightSpeed, -255, 255);

  // Apply a deadband to prevent small PWM values that might not move motors
  if (abs(leftSpeed) < 20) leftSpeed = 0;
  if (abs(rightSpeed) < 20) rightSpeed = 0;

  // Left motor control with error handling
  try {
    if (leftSpeed > 0) {
      ledcWrite(LEFT_MOTOR_FORWARD, leftSpeed);
      ledcWrite(LEFT_MOTOR_BACKWARD, 0);
    }
    else if (leftSpeed < 0) {
      ledcWrite(LEFT_MOTOR_BACKWARD, -leftSpeed);
      ledcWrite(LEFT_MOTOR_FORWARD, 0);
    } else {
      ledcWrite(LEFT_MOTOR_BACKWARD, 0);
      ledcWrite(LEFT_MOTOR_FORWARD, 0);
    }
  } catch (...) {
    // Recover from any errors
    ledcWrite(LEFT_MOTOR_BACKWARD, 0);
    ledcWrite(LEFT_MOTOR_FORWARD, 0);
  }

  // Right motor control with error handling
  try {
    if (rightSpeed > 0) {
      ledcWrite(RIGHT_MOTOR_FORWARD, rightSpeed);
      ledcWrite(RIGHT_MOTOR_BACKWARD, 0);
    }
    else if (rightSpeed < 0) {
      ledcWrite(RIGHT_MOTOR_BACKWARD, -rightSpeed);
      ledcWrite(RIGHT_MOTOR_FORWARD, 0);
    } else {
      ledcWrite(RIGHT_MOTOR_BACKWARD, 0);
      ledcWrite(RIGHT_MOTOR_FORWARD, 0);
    }
  } catch (...) {
    // Recover from any errors
    ledcWrite(RIGHT_MOTOR_BACKWARD, 0);
    ledcWrite(RIGHT_MOTOR_FORWARD, 0);
  }
}
