#pragma once
#include "main.h"


extern Controller controller;
extern pros::Imu inertialSensor;
extern bool upIsBlue;
extern void setIntakeSpeed(double speed);
extern void printDistance ();
extern void odomSetPos (double front, double side);
extern void odomReset (double front, double Lside);
extern void frontDistPIDSlew(double Dist, double tol, double speed, double angle, double accel);
extern void frontDistPIDSlewCustom(double Dist, double tol, double speed, double angle, double accel, double kP = 19, double kI = 0.49, double kD =11.7192, double kI_Windup = 1542);