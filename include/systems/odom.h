#pragma once
#include "main.h"

namespace odom {
    extern double bPosX, bPosY;
    extern double distanceToCenter;
    extern void update (double dEnc, double dSideEnc, double dGyroRad);
};

extern void odomMoveTo5 (int x, int y, double speed, double driveTol, double gyroTol, double accel);
extern void odomMoveto5B(int x, int y, double voltage, double driveTol, double gyroTol, double accel);
extern void odomMoveTo5NS(int x, int y, double voltage, double exitSpeed_, double gyroTol, double accel);