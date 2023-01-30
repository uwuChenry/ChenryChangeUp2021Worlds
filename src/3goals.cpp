#include "main.h"


void auton3 (){
    profileController->removePath("2BallStart");
    profileController->removePath("2Ball2nd");
    profileController->removePath("curve");
    setIntakeSpeed(100);
    manualControl = false;
    drivePIDSlew(mmToEnc(400), 10, 10000, 400);
    gyroTurn4(-135, 0.9);
    drivePIDexitSlow(mmToEnc(700), 4000, 9000);
    delay(300);
    chassisDriveModel->stop();
    setIntakeSpeed(0);
    autonShoot2(4);
    setIntakeSpeed(-200);
    drivePIDSlew(mmToEnc(-500), 10, 10000, 300);
    odomMoveto5B(mmToEnc(930), mmToEnc(150), 11000, 20, 0.9, 100);
    gyroTurn4(-180, 1.2); 
    drivePIDexitSlow(mmToEnc(290), 4000, 9000);
    delay(200);
    chassisDriveModel->stop();
    autonShoot2(1);
    drivePIDSlew(mmToEnc(-1080), 20, 11000, 200);
    drivePID3(mmToEnc(100), 20, 9000);
    setIntakeSpeed(150);
    odomMoveTo5NS(mmToEnc(2500), mmToEnc(-220), 11000, 4000, 0.9, 100);
    manualControl = true;
    rollerLower.moveVelocity(200);
    rollerUpper.moveVelocity(200);
    delay(100);
    setIntakeSpeed(0);
    chassisDriveModel->stop();
    delay(500);
    setIntakeSpeed(-200);
    drivePID3(mmToEnc(-400), 10, 9000);
    setIntakeSpeed(0);
    rollerUpper.moveVelocity(0);    
    rollerLower.moveVelocity(0);
}