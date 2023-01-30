/**
 * @file motor-control-speed.ino
 * @author Joshua Marshall (joshua.marshall@queensu.ca)
 * @author Sabrina Button [updated] (sabrina.button@queensu.ca)
 * @author Daniel Dubinko [updated] (daniel.dubinko@icloud.com)
 * @brief Arduino program to drive one wheel motor through a motor driver.
 * @version 2.0 
 * @date 2023-01-16
 *
 * @copyright Copyright (c) 2021-2022
 *
 */
#include <Arduino_LSM6DS3.h>

int EA = 19;  // Right Wheels PWM pin (must be a PWM pin).
int I1 = 3;   // Right Wheels direction digital pin 1
int I2 = 5;   // Right Wheels direction digital pin 2

int EB = 18;  // Left Wheels PWM pin (must be a PWM pin)
int I3 = 4;   // Left Wheels direction digital pin 1
int I4 = 2;   // Left Wheels direction digital pin 2

byte uR = 0;
byte uL = 0;

// Encoder ticks per (motor) revolution (TPR)
const int TPR_left = 3083;   //3083
const int TPR_right = 3100;  //3100

// Wheel radius [m]
const double RHO = 0.0625;
// Variable to store estimated angular rate of left wheel [rad/s]
double omega_L = 0.0;
double omega_R = 0.0;
double vL = 0.0;
double vR = 0.0;

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
  int u = 255;
  // Get the elapsed time [ms]
  rover_speed_calculation();
  // Write to the output pins
  fwd(u);
}

void fwd(int u) {
  // both sides drive forward.
  motor_ctrl(u, u, LOW, HIGH, LOW, HIGH);
  
}

void bck(int u) {
  // both sides drive backward
  motor_ctrl(u, u, HIGH, LOW, HIGH, LOW);
}

void ccw_turn(int u) {
  // have the left side drive backward
  // right side drive forward
  motor_ctrl(u, u, LOW, HIGH, HIGH, LOW);
}

void cw_turn(int u) {
  // have the right side drive backward
  // left side drive forward
  motor_ctrl(u, u, HIGH, LOW, LOW, HIGH);
}

void ccw_circle() {
  // the duty cycle of the left side should be less than right
  // to make a radial turn
  motor_ctrl(255, 100, LOW, HIGH, HIGH, LOW);
}

void cw_circle() {
  // the duty cycle of the right side should be less than left
  // to make a radial turn
  motor_ctrl(100, 255, LOW, HIGH, HIGH, LOW);
}


void figure_eight() {
  // Approach: do one full cw circle, then one full ccw circle immediately after
  int now = millis();

  while (millis() - now < 1000) {
    ccw_circle();
  }

  now = millis();
  while (millis() - now < 1000) {
    cw_circle();
  }
}

void right_wheels(int uR) {
  // Input selected direction
  digitalWrite(I1, LOW);
  digitalWrite(I2, HIGH);

  // PWM control for each side of robot to the motor driver
  analogWrite(EA, uR);  // right side
}

void motor_ctrl(int uA, int uB, bool i1, bool i2, bool i3, bool i4) {
  // Input selected direction
  digitalWrite(I1, i1);
  digitalWrite(I2, i2);
  digitalWrite(I3, i3);
  digitalWrite(I4, i4);

  // PWM control for each side of robot to the motor driver
  analogWrite(EA, uA);  // right side
  analogWrite(EB, uB);  // left side
}

// Compute vehicle speed [m/s]
double compute_vehicle_speed(double v1, double v2) {
  double v;
  v1 = abs(v1);
  v2 = abs(v2);
  v = 0.5 * (v1 + v2);
  return v;
}

// Compute vehicle turning rate [rad/s]
double compute_vehicle_rate(double v1, double v2) {
  double omega;
  omega = 1.0 * (v1 + v2);
  return omega;
}

void rover_speed_calculation(){
  t_now = millis();
  int roverSpeed;
  if (t_now - t_last >= T) {
    // Estimate the rotational speed [rad/s]
    omega_L = 2.0 * PI * ((double)encoder_ticks_left / (double)TPR_left) * 1000.0 / (double)(t_now - t_last);
    omega_R = 2.0 * PI * ((double)encoder_ticks_right / (double)TPR_right) * 1000.0 / (double)(t_now - t_last);

    vL = RHO * omega_L;
    vR = RHO * omega_R;

    // Print some stuff to the serial monitor
    Serial.print("Rover Speed: ");
    Serial.print(compute_vehicle_speed(vL, vR));
    Serial.print("\n");

    

    // Record the current time [ms]
    t_last = t_now;

    // Reset the encoder ticks counter
    encoder_ticks_left = 0;
    encoder_ticks_right = 0;
    
  }
}

void turning_rate_calculation(){

}