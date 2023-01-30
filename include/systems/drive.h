#pragma once
#include "main.h"

extern std::shared_ptr<ChassisModel> chassisDriveModel;
extern std::shared_ptr<AsyncMotionProfileController> profileController;
extern std::shared_ptr<OdomChassisController> Basechassis;

extern bool gyroIsReset;

extern void doTheThing(std::vector<std::pair<double, double>>& v);

extern void drivePIDSlewAngleWithBackup(double distance, double tol, double speed, double accel, double angle, int dRange);

extern void odomrun (void*);
extern void gyroTurn4(double angle, double tol);
extern void gyroTurn4Custom(double angle, double tol, double kP, double kI, double kD, double kI_Windup);
extern void drivePID3(double distance, double tol, double speed);
extern void drivePIDexitSlow(double distance, double exitSpeed, double speed);
extern void drivePIDSlew(double distance, double tol, double speed, double accel);
extern void drivePIDSlewExitSlow(double distance, double speed, double accel, double angle, double exitSpeed);
extern void drivePIDSlewAngle(double distance, double tol, double speed, double accel, double angle);
extern void lockChassis();
extern void followPath();
extern void unlockChassis();
extern void Drivecontrol();
extern void printEncoder();
