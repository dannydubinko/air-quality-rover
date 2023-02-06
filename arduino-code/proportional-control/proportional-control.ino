/**
 * @file ws1.ino
 * @author Joshua Marshall (joshua.marshall@queensu.ca)
 * @author Sabrina Button [updated] (sabrina.button@queensu.ca)
 * @author Daniel Dubinko [updated] (daniel.dubinko@icloud.com)
 * @author Luke Major [Updated] (luke.major@queensu.ca)
 * @brief Arduino program to drive one wheel motor through a motor driver.
 * @version 2.0 
 * @date 2023-01-16
 *
 * @copyright Copyright (c) 2021-2022
 *
 */

int EA = 6;  // Right Wheels PWM pin (must be a PWM pin).
int I1 = 3;   // Right Wheels direction digital pin 1
int I2 = 5;   // Right Wheels direction digital pin 2

int EB = 11;  // Left Wheels PWM pin (must be a PWM pin)
int I3 = 4;   // Left Wheels direction digital pin 1
int I4 = 2;   // Left Wheels direction digital pin 2

int left_Power_Input= 0;
int right_Power_Input = 0;

// Encoder ticks per (motor) revolution (TPR)
const int TPR_left = 3083;   //3083
const int TPR_right = 3100;  //3100

// Wheel radius [m]
const double RHO = 0.0625;

// Proportional Constants
const double k_P_L = 200;
const double k_P_R = 200;
const double k_I_L = 100;
const double k_I_R = 100;

double error_sum_right = 0;
double error_sum_left = 0;

// Sampling interval for measurements in milliseconds
const int T = 50;
// Counters for milliseconds during interval
long t_now = 0;
long t_last = 0;

// Encoder digital pins
const byte LEFTWHEEL_A = 8;
const byte LEFTWHEEL_B = 9;
const byte RIGHTWHEEL_A = 10;
const byte RIGHTWHEEL_B = 11;
const long L = 0.2775;

// Counter to keep track of encoder ticks [integer]
volatile long encoder_ticks_left = 0;
volatile long encoder_ticks_right = 0;

// Counter to keep track of the last number of ticks [integer]
long encoder_ticks_last_left = 0;
long encoder_ticks_last_right = 0;

// This function is called when SIGNAL_A goes HIGH
void decodeEncoderTicksRight() {
  if (digitalRead(RIGHTWHEEL_B) == LOW) {
    // SIGNAL_A leads SIGNAL_B, so count one way
    encoder_ticks_right--;
  } else {
    // SIGNAL_B leads SIGNAL_A, so count the other way
    encoder_ticks_right++;
  }
}

// This function is called when SIGNAL_B goes HIGH
void decodeEncoderTicksLeft() {
  if (digitalRead(LEFTWHEEL_B) == LOW) {
    // SIGNAL_A leads SIGNAL_B, so count one way
    encoder_ticks_left--;
  } else {
    // SIGNAL_B leads SIGNAL_A, so count the other way
    encoder_ticks_left++;
  }
}

void setup() {

  // Open the serial port at 9600 bps
  Serial.begin(9600);

  // Set the pin modes for the encoders
  pinMode(LEFTWHEEL_A, INPUT);
  pinMode(LEFTWHEEL_B, INPUT);
  pinMode(RIGHTWHEEL_A, INPUT);
  pinMode(RIGHTWHEEL_B, INPUT);

  // Configure digital pins for output
  // RIGHT SIDE:
  pinMode(EA, OUTPUT);
  pinMode(I1, OUTPUT);
  pinMode(I2, OUTPUT);

  // LEFT SIDE:
  pinMode(EB, OUTPUT);
  pinMode(I3, OUTPUT);
  pinMode(I4, OUTPUT);

  // Every time SIGNAL_A goes HIGH, this is a pulse
  attachInterrupt(digitalPinToInterrupt(LEFTWHEEL_A), decodeEncoderTicksLeft, RISING);
  attachInterrupt(digitalPinToInterrupt(RIGHTWHEEL_A), decodeEncoderTicksRight, RISING);

  // Print a message
  Serial.print("Program initialized.");
  Serial.print("\n");
}

void loop() {

  int desired_Vehicle_Speed = 2;

  int desired_Omega = 1 / L * (1);

  do{
    // buffers to collect data from control function
    int error[2];
    int speed[2];

    PI_controller(desired_Vehicle_Speed, desired_Omega, error, speed);

    drive_forward(speed[0], speed[1]);

    Serial.print(speed[0]);

  } while (1);
  
} 

// Adjust the value to be written to motors according to the error between the expected velocity and true velocity according to encoders
// written by Sabrina (2023/02/01)u
void PI_controller(double velocity_target, double omega_target, int *error_buffer, int *speed_buffer){
  // compute difference between current expected velocity and the feedback from the encoder (true velocity)
  double error_right = (velocity_target + (1/2) * L * omega_target) - encoderFeedback_Right_Velocity();
  double error_left = (velocity_target - (1/2) * L * omega_target) - encoderFeedback_Left_Velocity();

  // compute the integral sum of Vdesired - Vtrue by adding the current error to the sum of all previous errors.
  // NOTE**: for each new movement, integral sum should be reset
  error_sum_right += error_right;
  error_sum_left += error_left;

  // compute velocities according to standard PI function
  double velocity_Right = k_P_R*error_right + k_I_R*error_sum_right;
  double velocity_Left = k_P_L*error_left + k_I_L*error_sum_left;

  // anti-windup (if the velocity exceeds 255, disregard integral sum)
  if (abs(velocity_Left) > 255 ){
    velocity_Left -= k_I_L*error_sum_left;
  }
  if (abs(velocity_Right) > 255 ){
    velocity_Right-= k_I_R*error_sum_right;
  }

  // write speeds back to buffer array as output
  error_buffer[0] = (int)error_right;
  error_buffer[1] = (int)error_left;
  speed_buffer[0] = (int)velocity_Right;
  speed_buffer[1] = (int)velocity_Left;
};

double encoderFeedback_Left_Velocity() {

  t_now = millis();
  int roverSpeed;
  if (t_now - t_last >= T) {
    // Estimate the rotational speed [rad/s]
    int omega_Left = 2.0 * PI * ((double)encoder_ticks_left / (double)TPR_left) * 1000.0 / (double)(t_now - t_last);
  
    int left_Velocity = RHO * omega_Left;

    // Record the current time [ms]
    t_last = t_now;

    // Reset the encoder ticks counter
    encoder_ticks_left = 0;

    return left_Velocity;

  }
}

double encoderFeedback_Right_Velocity() {

  t_now = millis();
  int roverSpeed;
  if (t_now - t_last >= T) {
    // Estimate the rotational speed [rad/s]
    int omega_Right = 2.0 * PI * ((double)encoder_ticks_right / (double)TPR_right) * 1000.0 / (double)(t_now - t_last);
    int right_Velocity = RHO * omega_Right;

    // Record the current time [ms]
    t_last = t_now;

    // Reset the encoder ticks counter
    encoder_ticks_right = 0;

    return right_Velocity;
  }
}

void drive_forward(int right_PWM_Input, int left_PWM_Input) {
  // both sides drive forward.
  motor_ctrl(right_PWM_Input, left_PWM_Input, LOW, HIGH, LOW, HIGH);
}

void motor_ctrl(int right_PWM_Input, int left_PWM_Input, bool i1, bool i2, bool i3, bool i4) {
  // Input selected direction
  digitalWrite(I1, i1);
  digitalWrite(I2, i2);
  digitalWrite(I3, i3);
  digitalWrite(I4, i4);

  // PWM control for each side of robot to the motor driver
  analogWrite(EA, right_PWM_Input);  // right side
  analogWrite(EB, left_PWM_Input);  // left side
}
