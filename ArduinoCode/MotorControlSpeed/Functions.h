#ifndef RoverFunctions
#define RoverFunctions

#include <Arduino.h>

double compute_vehicle_speed(double v1, double v2);

void fwd(int u);

void bck(int u);

void ccw_turn(int u);

void cw_turn(int u);

void ccw_circle();

void cw_circle();

void figure_eight();

void right_wheels(int uR);

void motor_ctrl(int uA, int uB, bool i1, bool i2, bool i3, bool i4);

#endif