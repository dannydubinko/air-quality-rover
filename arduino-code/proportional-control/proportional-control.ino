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

int EA = 19;  // Right Wheels PWM pin (must be a PWM pin).
int I1 = 3;   // Right Wheels direction digital pin 1
int I2 = 5;   // Right Wheels direction digital pin 2

int EB = 18;  // Left Wheels PWM pin (must be a PWM pin)
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
const double k_I_L = 1;
const double k_I_R = 1;

// Variable to store estimated angular rate of left wheel [rad/s]
double desired_Vehicle_Speed = 0;
double desired_Omega = 0;
double omega_Left = 0.0;
double omega_Right = 0.0;
double left_Velocity = 0.0;
double right_Velocity = 0.0;
double integral_term_prev = 0;

// Sampling interval for measurements in milliseconds
const int T = 1000;
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

  int desired_Vehicle_Speed = 200;

  int desired_Omega = 1 / L * (1);

  do{

    double e_prop_L = compute_proportional(-1, desired_Vehicle_Speed, desired_Omega, feedback_Left_Velocity);
    double e_int_L = compute_integral(-1, desired_Vehicle_Speed, desired_Omega, feedback_Left_Velocity);
    int u_Left = PI_controller(e_prop_L, e_int_L, k_P_L, k_I_L, 0);
     double e_prop_R = compute_proportional(1, desired_Vehicle_Speed, desired_Omega, feedback_Right_Velocity);
    double e_int_R = compute_integral(1, desired_Vehicle_Speed, desired_Omega, feedback_Right_Velocity);
    int u_Right = PI_controller(e_prop_R, e_int_R, k_P_R, k_I_R, 0);
  fwd( u_Left, u_Right );
  delay(10);

  } while (1);
  
} 

int PI_controller(double proportional_term, double integral_term, double k_P, double k_I, bool anti_windup){
  short u;
  if(!anti_windup){
    u = (short)(k_P * proportional_term + k_I * integral_term);
  } else{
    // If anti-windup is on, do not consider the integral_term
    u = (short)(k_P * proportional_term);
  }
 
  if (u > 255){
    u = 255;
    // Call function again with anti-windup on.
    u = PI_controller(proportional_term, integral_term,k_P, k_I, 1);
  } else if (u < -255){
    u = -255;
    // Call function again with anti-windup on.
    u = PI_controller(proportional_term, integral_term,k_P, k_I, 1);
  }
  return (int)u;
}

// Compute the proportional term, vL,d - vL[i]
double compute_proportional(int coeff, double v_d, double omega_d, double feedback_func()){
  // int coeff accounts for left vs right movement; coeff is -1 for left and +1 for right.
  double v_des = v_d + coeff* (1/2) * L * omega_d;
  return v_des - feedback_func();
}

// Compute the integral term as the sum of the current proportional term plus all previous terms.
double compute_integral(int coeff, double v_d, double omega_d, double feedback_func()){
  // int coeff accounts for left vs right movement; coeff is -1 for left and +1 for right.
  double v_des = v_d + coeff* (1/2) * L * omega_d;
  integral_term_prev += v_des - feedback_func();
  return integral_term_prev;
}

double feedback_Left_Velocity() {

  t_now = millis();
  int roverSpeed;
  if (t_now - t_last >= T) {
    // Estimate the rotational speed [rad/s]
    omega_Left = 2.0 * PI * ((double)encoder_ticks_left / (double)TPR_left) * 1000.0 / (double)(t_now - t_last);
  
    left_Velocity = RHO * omega_Left;

    // Record the current time [ms]
    t_last = t_now;

    // Reset the encoder ticks counter
    encoder_ticks_left = 0;

    return left_Velocity;

  }
}

double feedback_Right_Velocity() {

  t_now = millis();
  int roverSpeed;
  if (t_now - t_last >= T) {
    // Estimate the rotational speed [rad/s]
    omega_Right = 2.0 * PI * ((double)encoder_ticks_right / (double)TPR_right) * 1000.0 / (double)(t_now - t_last);
    right_Velocity = RHO * omega_Right;

    // Record the current time [ms]
    t_last = t_now;

    // Reset the encoder ticks counter
    encoder_ticks_right = 0;

    return right_Velocity;
  }
}

void fwd(int right_PWM_Input, int left_PWM_Input) {
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



/********** Achived functions ***********
double left_PI_Speed_Control(double desired_Vehicle_Speed, double desired_Omega) {

  double desired_Left_Velocity = desired_Vehicle_Speed - (1 / 2) * L * desired_Omega;

  double left_PWM_Input = right_Proportional_Constant * (desired_Left_Velocity - feedback_Left_Velocity());

  return left_PWM_Input;

}

double right_PI_Speed_Control(double desired_Velocity, double desired_Omega) {

  double desired_Right_Velocity = desired_Velocity + (1 / 2) * L * desired_Omega;

  double right_PWM_Input = left_Proportional_Constant * (desired_Right_Velocity - feedback_Right_Velocity());

  return right_PWM_Input;

}*/