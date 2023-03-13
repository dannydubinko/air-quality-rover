/**
 * @file speed-controller.ino
 * @author Joshua Marshall (joshua.marshall@queensu.ca), Thomas Sears (thomas.sears@queensu.ca)
 * @brief Arduino program to control the speed and turning rate of a vehicle.
 * @version 2.2
 * @date 2022-12-17
 *
 * @copyright Copyright (c) 2023
 *
 */
// LIBRARIES
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <String.h>

// OLED SCREEN SETUP
#define SCREEN_WIDTH 128     // OLED display width, in pixels
#define SCREEN_HEIGHT 32     // OLED display height, in pixels
#define OLED_RESET -1        // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C  ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

/* PIN CONNECTIONS */

// Left wheel PWM control
int EB = 5;  // Wheel PWM pin (must be a PWM pin)
int I3 = 3;  // Wheel direction digital pin 1
int I4 = 4;  // Wheel direction digital pin 2

// Right wheel PWM control
int EA = 6;  // Wheel PWM pin (must be a PWM pin)
int I1 = 8;  // Wheel direction digital pin 1
int I2 = 7;  // Wheel direction digital pin 2

// Left wheel encoder digital pins
const byte SIGNAL_AL = 11;  // green wire
const byte SIGNAL_BL = 12;  // yellow wire

// Right wheel encoder digital pins
const byte SIGNAL_AR = 9;   // green wire
const byte SIGNAL_BR = 10;  // yellow wire

/* USEFUL CONSTANTS */

// Encoder ticks per (motor) revolution (TPR)
const int TPR = 3000;

// Wheel radius [m]
const double RHO = 0.0625;

// Vehicle track [m]
const double ELL = 0.2775;

// Sampling interval for measurements in milliseconds
const int T = 100;

// Controller gains (use the same values for both wheels)
const double KP = 200.0;  // Proportional gain
const double KI = 100.0;  // Integral gain

/* VARIABLE DECLARATIONS */

// Motor PWM command variables [0-255]
short u_L = 0;
short u_R = 0;

// Counter to keep track of encoder ticks [integer]
volatile long encoder_ticks_L = 0;
volatile long encoder_ticks_R = 0;

// Variables to store estimated angular rates of wheels [rad/s]
double omega_L = 0.0;
double omega_R = 0.0;

// Variables to store estimated wheel speeds [m/s]
double v_L = 0.0;
double v_R = 0.0;

// Variables to store vehicle speed and turning rate
double v = 0.0;      // [m/s]
double omega = 0.0;  // [rad/s]

// Variables to store desired vehicle speed and turning rate
double v_d = 0.0;      // [m/s]
double omega_d = 0.0;  // [rad/s]

// Variable to store desired wheel speeds [m/s]
double v_Ld = 0.0;
double v_Rd = 0.0;

// Counters for milliseconds during interval
long t_now = 0;
long t_last = 0;

// Variables to store errors for controller
double e_L = 0.0;
double e_R = 0.0;
double e_Lint = 0.0;
double e_Rint = 0.0;

/* HELPER FUNCTIONS */

// This function applies PWM inputs (u_L and u_R) to the right and left wheels
void driveVehicle(short u_L, short u_R) {
  // LEFT WHEEL
  if (u_L < 0)  // If the controller calculated a negative input...
  {
    digitalWrite(I3, HIGH);  // Drive backward (left wheels)
    digitalWrite(I4, LOW);   // Drive backward (left wheels)

    analogWrite(EB, -u_L);  // Write left motors command
  } else                    // the controller calculated a positive input
  {
    digitalWrite(I3, LOW);   // Drive forward (left wheels)
    digitalWrite(I4, HIGH);  // Drive forward (left wheels)

    analogWrite(EB, u_L);  // Write left motors command
  }

  // RIGHT WHEEL
  if (u_R < 0)  // If the controller calculated a negative input...
  {
    digitalWrite(I1, LOW);   // Drive backward (right wheels)
    digitalWrite(I2, HIGH);  // Drive backward (right wheels)

    analogWrite(EA, -u_R);  // Write right motors command
  } else                    // the controller calculated a positive input
  {
    digitalWrite(I1, HIGH);  // Drive forward (right wheels)
    digitalWrite(I2, LOW);   // Drive forward (right wheels)

    analogWrite(EA, u_R);  // Write right motors command
  }
}

// This function is called when SIGNAL_AL (left encoder) goes HIGH
void decodeEncoderTicks_L() {
  if (digitalRead(SIGNAL_BL) == LOW) {
    // SIGNAL_A leads SIGNAL_B, so count one way
    encoder_ticks_L--;
  } else {
    // SIGNAL_B leads SIGNAL_A, so count the other way
    encoder_ticks_L++;
  }
}

// This function is called when SIGNAL_AR (right encoder) goes HIGH
void decodeEncoderTicks_R() {
  if (digitalRead(SIGNAL_BR) == LOW) {
    // SIGNAL_A leads SIGNAL_B, so count one way
    encoder_ticks_R++;
  } else {
    // SIGNAL_B leads SIGNAL_A, so count the other way
    encoder_ticks_R--;
  }
}

// Compute the wheel rate from elapsed time and encoder ticks [rad/s]
double compute_wheel_rate(long encoder_ticks, double delta_t) {
  double omega;
  omega = 2.0 * PI * ((double)encoder_ticks / (double)TPR) * 1000.0 / delta_t;
  return omega;
}

// Compute wheel speed [m/s]
double compute_wheel_speed(double omega_wheel) {
  double v_wheel;
  v_wheel = omega_wheel * RHO;
  return v_wheel;
}

// Compute vehicle speed [m/s]
double compute_vehicle_speed(double v_L, double v_R) {
  double v;
  v = 0.5 * (v_L + v_R);
  return v;
}

// Compute vehicle turning rate [rad/s]
double compute_vehicle_rate(double v_L, double v_R) {
  double omega;
  omega = 1.0 / ELL * (v_R - v_L);
  return omega;
}

// Compute v_L from v and omega
double compute_L_wheel_speed(double v, double omega) {
  double v_wheel = 0.0;
  v_wheel = v - ELL / 2.0 * omega;
  return v_wheel;
}

// Compute v_R from v and omega
double compute_R_wheel_speed(double v, double omega) {
  double v_wheel = 0.0;
  v_wheel = v + ELL / 2.0 * omega;
  return v_wheel;
}

// Wheel speed PI controller function
short PI_controller(double e_now, double e_int, double k_P, double k_I) {
  short u;
  u = (short)(k_P * e_now + k_I * e_int);

  // Saturation (i.e., maximum input) detection
  if (u > 255) {
    u = 255;
  } else if (u < -255) {
    u = -255;
  }
  return u;
}

/* SETUP FUNCTION */

void setup() {
  // Open the serial port at 9600 bps
  Serial.begin(9600);

  //check if oled monitor is on
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    for (;;)
      ;  // Don't proceed, loop forever
  }

  // Configure digital pins for output
  pinMode(EA, OUTPUT);
  pinMode(I1, OUTPUT);
  pinMode(I2, OUTPUT);

  // Configure digital pins for output
  pinMode(EB, OUTPUT);
  pinMode(I3, OUTPUT);
  pinMode(I4, OUTPUT);

  // Set the pin modes for the encoders
  pinMode(SIGNAL_AL, INPUT);
  pinMode(SIGNAL_BL, INPUT);
  pinMode(SIGNAL_AR, INPUT);
  pinMode(SIGNAL_BR, INPUT);

  // Send 0 PWM commands
  analogWrite(EA, 0);
  analogWrite(EB, 0);

  // Send brake signals to motor driver
  digitalWrite(I1, LOW);
  digitalWrite(I2, LOW);
  digitalWrite(I3, LOW);
  digitalWrite(I4, LOW);

  // Every time the pin goes high, this is a pulse; enable the interrupts
  attachInterrupt(digitalPinToInterrupt(SIGNAL_AL), decodeEncoderTicks_L,
                  RISING);
  attachInterrupt(digitalPinToInterrupt(SIGNAL_AR), decodeEncoderTicks_R,
                  RISING);

  // Pause for a second (perhaps unnecessary)
  delay(1000);

  // Print a message
  Serial.print("Program initialized.");
  Serial.print("\n");
}

/* MAIN PROGRAM LOOP */

void loop() {
  //SERIAL COMMUNICATION CODE
  // if (Serial.available() > 0) {
  //   String message = Serial.readStringUntil('\n');
  // }

  // //capture values for speed from message array
  // String left_speed = message.substring(1, a.indexOf(","));
  // String right_speed = message.substring(5, a.indexOf("]"));

  // left_speed = left_speed.toInt();
  // right_speed = right_speed.toInt();

  //left_wheel_ctrl(left_speed)
  //right_wheel_ctrl(right_speed)

  //PID CONTROLLER CODE
  // Get the elapsed time [ms]
  t_now = millis();

  // Perform control update every T milliseconds
  if (t_now - t_last >= T) {

    // Set the desired vehicle speed and turning rate
    v_d = 0.0;      // [m/s]
    omega_d = 1.0;  // [rad/s]

    // Estimate the rotational speed of each wheel [rad/s]
    omega_L = compute_wheel_rate(encoder_ticks_L, (double)(t_now - t_last));
    omega_R = compute_wheel_rate(encoder_ticks_R, (double)(t_now - t_last));

    // Compute the speed of each wheel [m/s]
    v_L = compute_wheel_speed(omega_L);
    v_R = compute_wheel_speed(omega_R);

    // Compute the speed of the vehicle [m/s]
    v = compute_vehicle_speed(v_L, v_R);

    // Compute the turning rate of the vehicle [rad/s]
    omega = compute_vehicle_rate(v_L, v_R);

    // Record the current time [ms]
    t_last = t_now;

    // Reset the encoder ticks counter
    encoder_ticks_L = 0;
    encoder_ticks_R = 0;

    // Compute the desired wheel speeds from v_d and omega_d
    v_Ld = compute_L_wheel_speed(v_d, omega_d);
    v_Rd = compute_R_wheel_speed(v_d, omega_d);

    // Compute errors
    e_L = v_Ld - v_L;
    e_R = v_Rd - v_R;

    // Integrate errors with anti-windup
    if (abs(u_L) < 255) {
      e_Lint += e_L;
    }
    if (abs(u_R) < 255) {
      e_Rint += e_R;
    }

    // Compute control signals using PI controller
    u_L = PI_controller(e_L, e_Lint, KP, KI);
    u_R = PI_controller(e_R, e_Rint, KP, KI);

    // Drive the vehicle
    driveVehicle(u_L, u_R);

    // Print some stuff to the serial monitor (or plotter)
    Serial.print("Vehicle_speed_[m/s]:");
    Serial.print(v);
    Serial.print(",");
    Serial.print("Turning_rate_[rad/s]:");
    Serial.print(omega);
    Serial.print(",");
    Serial.print("u_L:");
    Serial.print(u_L);
    Serial.print(",");
    Serial.print("u_R:");
    Serial.print(u_R);
    Serial.print("\n");
  }
}

void left_wheel_ctrl(int uB) {
  // Input selected direction
  digitalWrite(I3, HIGH);
  digitalWrite(I4, LOW);

  // PWM control for each side of robot to the motor driver
  analogWrite(EB, uB);  // left side
}

void right_wheel_ctrl(int uA) {
  // Input selected direction
  digitalWrite(I1, HIGH);
  digitalWrite(I2, LOW);

  // PWM control for each side of robot to the motor driver
  analogWrite(EA, uA);  // right side
}
