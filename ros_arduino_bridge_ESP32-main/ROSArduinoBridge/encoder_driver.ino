/* *************************************************************
   Encoder definitions

   Add an "#ifdef" block to this file to include support for
   a particular encoder board or library. Then add the appropriate
   #define near the top of the main ROSArduinoBridge.ino file.

   ************************************************************ */

#include <ESP32Encoder.h>
ESP32Encoder enc0;
ESP32Encoder enc1;

// Variables to detect stuck encoders
static long last_left_count = 0;
static long last_right_count = 0;
static unsigned long last_encoder_check_time = 0;
static int stuck_count = 0;

void initEncoder(){
  // Initialize encoders with internal pull-up resistors enabled
  enc0.attachFullQuad(LEFT_ENC_PIN_A, LEFT_ENC_PIN_B);
  enc1.attachFullQuad(RIGHT_ENC_PIN_A, RIGHT_ENC_PIN_B);
  
  // Enable internal pull-up resistors for both encoders
  pinMode(LEFT_ENC_PIN_A, INPUT_PULLUP);
  pinMode(LEFT_ENC_PIN_B, INPUT_PULLUP);
  pinMode(RIGHT_ENC_PIN_A, INPUT_PULLUP);
  pinMode(RIGHT_ENC_PIN_B, INPUT_PULLUP);
  
  resetEncoders();
}

/* Wrap the encoder reading function with sync detection */
long readEncoder(int i) {
  long current_left = enc0.getCount();
  long current_right = enc1.getCount();
  
  // Check for stuck encoders (only if moving)
  unsigned long current_time = millis();
  if (current_time - last_encoder_check_time > 500) { // Check every 500ms
    last_encoder_check_time = current_time;
    
    if (moving) { // Only check if we're supposed to be moving
      if (current_left == last_left_count && current_right == last_right_count) {
        stuck_count++;
        if (stuck_count > 3) { // After ~1.5 seconds of no change
          // Try to reset encoder instances
          enc0.clearCount();
          enc1.clearCount();
          
          // Re-attach encoders (force hardware reinit)
          enc0.attachFullQuad(LEFT_ENC_PIN_A, LEFT_ENC_PIN_B);
          enc1.attachFullQuad(RIGHT_ENC_PIN_A, RIGHT_ENC_PIN_B);
          
          stuck_count = 0;
        }
      } else {
        // We got new values, reset stuck counter
        stuck_count = 0;
      }
    }
    
    last_left_count = current_left;
    last_right_count = current_right;
  }
  
  if (i == LEFT) return current_left;
  else return -current_right;  // Invert right encoder for correct odometry
}

/* Wrap the encoder reset function */
void resetEncoder(int i) {
  if (i == LEFT) {
    enc0.setCount(0);
  } else {
    enc1.setCount(0);
  }
  
  // Reset sync check values
  last_left_count = 0;
  last_right_count = 0;
  stuck_count = 0;
}

/* Reset both encoders */
void resetEncoders() {
  enc0.setCount(0);
  enc1.setCount(0);
  
  // Reset sync check values
  last_left_count = 0;
  last_right_count = 0;
  stuck_count = 0;
}
