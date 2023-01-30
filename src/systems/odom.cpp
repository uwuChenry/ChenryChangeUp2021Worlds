#include "main.h"

namespace odom {
    double bPosX = 0, bPosY = 0, bHeadingRad = 0;
    double distanceToCenter = 221.4;
    void update (double dEnc, double dSideEnc, double dGyroRad);
};


void odomMoveto4(int x, int y, double voltage, double driveTol, double gyroTol){
    double distance = sqrt((odom::bPosX - x)*(odom::bPosX - x) + (odom::bPosY - y)*(odom::bPosY - y));
    double angle = atan2((x-odom::bPosX),(y-odom::bPosY));
    angle = convertToDeg(angle);
    gyroTurn4(angle, gyroTol);
    delay(150);
    drivePID3(distance, driveTol, voltage);
}

void odomMoveTo5 (int x, int y, double speed, double driveTol, double gyroTol, double accel){
    double distance = sqrt((odom::bPosX - x)*(odom::bPosX - x) + (odom::bPosY - y)*(odom::bPosY - y));
    double angle = atan2((x-odom::bPosX),(y-odom::bPosY));
    angle = convertToDeg(angle);
    gyroTurn4(angle, gyroTol);
    drivePIDSlew(distance, driveTol, speed, accel);
}

void odomMoveTo4NS (int x, int y, double voltage, double exitVoltage, double gyroTol){
    double distance = sqrt((odom::bPosX - x)*(odom::bPosX - x) + (odom::bPosY - y)*(odom::bPosY - y));
    double angle = atan2((x-odom::bPosX),(y-odom::bPosY));
    angle = convertToDeg(angle);
    gyroTurn4(angle, gyroTol);
    delay(100);
    drivePIDexitSlow(distance, exitVoltage, voltage);
}

void odomMoveTo4B(int x, int y, double voltage, double driveTol, double gyroTol){
    double distance = -1 * sqrt((odom::bPosX - x)*(odom::bPosX - x) + (odom::bPosY - y)*(odom::bPosY - y));
    double angle = atan2((x-odom::bPosX),(y-odom::bPosY));
    angle = convertToDeg(angle) - 180;
    gyroTurn4(angle, gyroTol);
    delay(100);
    drivePID3(distance, driveTol, voltage);
}

void odomMoveTo5NS(int x, int y, double voltage, double exitSpeed_, double gyroTol, double accel){
    double distance = sqrt((odom::bPosX - x)*(odom::bPosX - x) + (odom::bPosY - y)*(odom::bPosY - y));
    double angle = atan2((x-odom::bPosX),(y-odom::bPosY));
    angle = convertToDeg(angle);
    gyroTurn4(angle, gyroTol);
    delay(100);
    drivePIDSlewExitSlow(distance, voltage, accel, angle, exitSpeed_);
}

void odomMoveto5B(int x, int y, double voltage, double driveTol, double gyroTol, double accel){
    double distance = -1 * sqrt((odom::bPosX - x)*(odom::bPosX - x) + (odom::bPosY - y)*(odom::bPosY - y));
    double angle = atan2((x-odom::bPosX),(y-odom::bPosY));
    angle = convertToDeg(angle) - 180;
    gyroTurn4(angle, gyroTol);
    drivePIDSlew(distance, driveTol, voltage, accel);
}

void odom::update (double dEnc, double dSideEnc, double dGyroRad) { 
    double dX;
    double dY;
    if (dGyroRad == 0){
        dX = dSideEnc;
        dY = dEnc;
    }
    else {
        double r = dEnc / dGyroRad;
        dX = r - (r * cos (dGyroRad));
        dY = r * sin (dGyroRad);

        double rS = dSideEnc / dGyroRad;
        rS -= sgn(rS) * distanceToCenter;
        dX += rS * sin (dGyroRad);
        dY += rS - (rS * cos (dGyroRad));
    }
    bPosX += dX * cos (bHeadingRad) + dY * sin (bHeadingRad);
    bPosY += dY * cos (bHeadingRad) - dX * sin (bHeadingRad);
    bHeadingRad += dGyroRad;
}

