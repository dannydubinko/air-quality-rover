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
int I1 = 3;  // Right Wheels direction digital pin 1
int I2 = 5;  // Right Wheels direction digital pin 2

int EB = 12;  // Left Wheels PWM pin (must be a PWM pin)
int I3 = 4;   // Left Wheels direction digital pin 1
int I4 = 2;   // Left Wheels direction digital pin 2

int left_Power_Input = 0;
int right_Power_Input = 0;

// Encoder ticks per (motor) revolution (TPR)
const int TPR_left = 3083;   //3083
const int TPR_right = 3100;  //3100

// Wheel radius [m]
const double RHO = 0.0625;

// Proportional Constant
double right_Proportional_Constant = 50;
double left_Proportional_Constant = 50;

// Variable to store estimated angular rate of left wheel [rad/s]
double desired_Vehicle_Speed = 0;
double desired_Omega = 0;
double omega_Left = 0.0;
double omega_Right = 0.0;
double left_Velocity = 0.0;
double right_Velocity = 0.0;

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

// Counter to keep track of encoder ticks [integer]
volatile long encoder_ticks_left = 0;
volatile long encoder_ticks_right = 0;

// Counter to keep track of the last number of ticks [integer]
long encoder_ticks_last_left = 0;
long encoder_ticks_last_right = 0;

// This function is called when SIGNAL_A goes HIGH
void decodeEncoderTicksRight() {
  if (digitalRead(RIGHTWHEEL_B) == HIGH) {
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

  int desired_Vehicle_Speed = 3;

  int desired_Omega = 0;


  t_now = millis();
  int roverSpeed;
  if (t_now - t_last >= T) {
    // Estimate the rotational speed [rad/s]
    omega_Left = 2.0 * PI * ((double)encoder_ticks_left / (double)TPR_left) * 1000.0 / (double)(t_now - t_last);
    omega_Right = 2.0 * PI * ((double)encoder_ticks_right / (double)TPR_right) * 1000.0 / (double)(t_now - t_last);

    right_Velocity = RHO * omega_Right;
    left_Velocity = RHO * omega_Left;

    double desired_Left_Velocity = desired_Vehicle_Speed - (0.5) * 0.2775 * desired_Omega;
    double desired_Right_Velocity = desired_Vehicle_Speed + (0.5) * 0.2775 * desired_Omega;

    int left_Power_Input = (int)(right_Proportional_Constant * (desired_Left_Velocity - left_Velocity));
    int right_Power_Input = (int)(left_Proportional_Constant * (desired_Right_Velocity - right_Velocity));

    // Record the current time [ms]
    t_last = t_now;

    // Reset the encoder ticks counter
    encoder_ticks_left = 0;

    // Reset the encoder ticks counter
    encoder_ticks_right = 0;

    Serial.print(left_Power_Input);
    Serial.print(' ');
    Serial.print(right_Power_Input);
    Serial.print('\n');

    fwd(right_Power_Input, left_Power_Input);
  }
}

/*double left_PI_Speed_Control(double desired_Vehicle_Speed, double desired_Omega, double left_Velocity) {

  double desired_Left_Velocity = desired_Vehicle_Speed - (0.5) * 0.2775 * desired_Omega;

  int left_Power_Input = (int)(right_Proportional_Constant * (desired_Left_Velocity - left_Velocity));

  Serial.print(left_Power_Input);
  Serial.print(' ');

  return left_Power_Input;
}

int right_PI_Speed_Control(double desired_Velocity, double desired_Omega, double right_Velocity) {

  double desired_Right_Velocity = desired_Velocity + (0.5) * 0.2775 * desired_Omega;

  int right_Power_Input = (int)(left_Proportional_Constant * (desired_Right_Velocity - right_Velocity));

  Serial.print(right_Power_Input);
  Serial.print('\n');

  return right_Power_Input;
}

/*double compute_Left_Velocity() {

  t_now = millis();
  int roverSpeed;
  if (t_now - t_last_left >= T) {
    // Estimate the rotational speed [rad/s]
    omega_Left = 2.0 * PI * ((double)encoder_ticks_left / (double)TPR_left) * 1000.0 / (double)(t_now - t_last_left);
  
    left_Velocity = RHO * omega_Left;



    // Record the current time [ms]
    t_last_left = t_now;

    // Reset the encoder ticks counter
    encoder_ticks_left = 0;

    return left_Velocity;

  }
}

double compute_Right_Velocity() {

  t_now = millis();
  int roverSpeed;
  if (t_now - t_last_right >= T) {
    // Estimate the rotational speed [rad/s]
    omega_Right = 2.0 * PI * ((double)encoder_ticks_right / (double)TPR_right) * 1000.0 / (double)(t_now - t_last_right);
    right_Velocity = RHO * omega_Right;

    // Record the current time [ms]
    t_last_right = t_now;

    // Reset the encoder ticks counter
    encoder_ticks_right = 0;

    return right_Velocity;
  }
}*/

void fwd(int right_Power_Input, int left_Power_Input) {
  // both sides drive forward.
  motor_ctrl(right_Power_Input, left_Power_Input, LOW, HIGH, LOW, HIGH);
}

void motor_ctrl(int right_Power_Input, int left_Power_Input, bool i1, bool i2, bool i3, bool i4) {
  // Input selected direction
  digitalWrite(I1, i1);
  digitalWrite(I2, i2);
  digitalWrite(I3, i3);
  digitalWrite(I4, i4);

  // PWM control for each side of robot to the motor driver
  analogWrite(EA, right_Power_Input);  // right side
  analogWrite(EB, left_Power_Input);   // left side
}
