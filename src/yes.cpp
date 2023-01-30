#include "main.h"
#include "profiles/poggers.h"
#include "profiles/poggers2.h"
#include "profiles/poggers3.h"



void skillv5(){
    manualControl = false;
    setIntakeSpeed(200);
    doTheThing(poggers);
    frontDistPIDSlew(950, 15, 11000, 0, 150);
    gyroTurn4(45, 0.9);
    setIntakeSpeed(100);
    drivePIDSlewExitSlow(mmToEnc(800), 11000, 200, 45, 4000);
    delay(150);
    shootUntilBlue(200, 70, 2);
    chassisDriveModel->stop();
    delay(100);
    manualControl = true;
    rollerLower.moveVelocity(-200);
    rollerUpper.moveVelocity(-200);
    setIntakeSpeed(-130);
    drivePIDSlewAngle(mmToEnc(-500), 10, 11000, 150, 45);
    setIntakeSpeed(-200);
    delay(100);
    //back from 1st goal

    manualControl = false;
    setIntakeSpeed(200);
    odomMoveTo5(mmToEnc(-780), mmToEnc(-320), 11000, 15, 0.9, 150);
    odomMoveTo5NS(mmToEnc(50), mmToEnc(-280), 11000, 4000, 0.9, 150);
    setIntakeSpeed(130);
    delay(100);
    chassisDriveModel->stop();
    manualControl = true;
    rollerUpper.moveVelocity(200);
    rollerLower.moveVelocity(80);
    delay(650);
    rollerUpper.moveVelocity(200);
    delay(200);
    setIntakeSpeed(-100);
    rollerUpper.moveVelocity(0);
    rollerLower.moveVelocity(0);
    drivePIDSlew(mmToEnc(-180), 20, 11000, 200);
    setIntakeSpeed(-200);
    rollerUpper.moveVelocity(-200);
    rollerLower.moveVelocity(-200);
    gyroTurn4(45, 5);
    gyroTurn4(180, 1);
    //back from 2nd goal

    manualControl = false;
    setIntakeSpeed(200);
    doTheThing(poggers2);
    frontDistPIDSlewCustom(925, 25, 11000, -180, 150, 12, 0.8, 18, 2500);
    //frontDistPIDSlew(925, 20, 11000, -180, 120);
    gyroTurn4(135, 0.9);
    setIntakeSpeed(150);
    drivePIDSlewExitSlow(mmToEnc(800), 11000, 200, 135, 4000);
    delay(150);
    shootUntilBlue(200, 70, 2);
    chassisDriveModel->stop();
    delay(100);
    manualControl = true;
    rollerLower.moveVelocity(-200);
    rollerUpper.moveVelocity(-200);
    setIntakeSpeed(-80);
    drivePIDSlewAngle(mmToEnc(-500), 10, 11000, 150, 135);
    //back from 3rd goal

    manualControl = false;
    setIntakeSpeed(200);
    odomMoveTo5(mmToEnc(-1320), mmToEnc(-910), 11000, 15, 0.9, 150);
    //gyroTurn4(-180, 0.9);
    odomMoveTo5NS(mmToEnc(-1295), mmToEnc(-1700), 11000, 4000, 0.9, 150);
    setIntakeSpeed(130);
    delay(100);
    chassisDriveModel->stop();
    manualControl = true;
    rollerUpper.moveVelocity(200);
    rollerLower.moveVelocity(85);
    delay(650);
    rollerUpper.moveVelocity(200);
    setIntakeSpeed(-80);
    delay(300);
    rollerUpper.moveVelocity(-200);
    rollerLower.moveVelocity(-200);
    drivePIDSlewAngle(mmToEnc(-510), 15, 11000, 150, -180);
    gyroTurn4(-90, 1);
    //back from 4th goal

    manualControl = false;
    setIntakeSpeed(200);
    frontDistPIDSlewCustom(550, 25, 11000, -90, 150, 12, 0.8, 18, 2500);
    odomReset(550, 939);
    gyroTurn4(158, 1);
    drivePIDSlewAngleWithBackup(mmToEnc(620), 15, 11000, 150, 158, 0);
    //drivePIDSlewAngle(mmToEnc(620), 15, 11000, 150, 158);
    drivePIDSlewAngle(mmToEnc(-390), 15, 11000, 150, 158);
    gyroTurn4(-135, 0.9);
    drivePIDSlewExitSlow(mmToEnc(550), 11000, 150, -135, 4000);
    setIntakeSpeed(100);
    delay(150);
    shootUntilBlue(200, 80, 2);
    chassisDriveModel->stop();
    delay(100);
    manualControl = true;
    rollerLower.moveVelocity(-200);
    rollerUpper.moveVelocity(-200);
    setIntakeSpeed(-100);
    drivePIDSlewAngle(mmToEnc(-500), 15, 11000, 150, -135);
    //back from 5th goal

    manualControl = false;
    setIntakeSpeed(200);
    odomMoveTo5(mmToEnc(520), mmToEnc(900), 11000, 15, 0.9, 150);
    odomMoveTo5NS(mmToEnc(-350), mmToEnc(880), 11000, 4000, 0.9, 150);
    delay(100);
    chassisDriveModel->stop();
    autonShoot2(4);
    drivePIDSlewAngle(mmToEnc(-180), 15, 11000, 150, -90);
    manualControl = true;
    setIntakeSpeed(-200);
    rollerUpper.moveVelocity(-200);
    rollerLower.moveVelocity(-200);
    gyroTurn4(-135, 5);
    gyroTurn4(0, 1);
    //back from 6th goal

    manualControl = false;
    setIntakeSpeed(200);
    doTheThing(poggers2);
    frontDistPIDSlewCustom(900, 25, 11000, 0, 150, 12, 0.8, 18, 2500);
    //frontDistPIDSlew(925, 20, 11000, -180, 120);
    gyroTurn4(-45, 0.9);
    setIntakeSpeed(150);
    drivePIDSlewExitSlow(mmToEnc(800), 11000, 200, -45, 4000);
    delay(150);
    shootUntilBlue(200, 70, 2);
    chassisDriveModel->stop();
    delay(100);
    manualControl = true;
    rollerLower.moveVelocity(-200);
    rollerUpper.moveVelocity(-200);
    setIntakeSpeed(-80);
    drivePIDSlewAngle(mmToEnc(-500), 10, 11000, 150, -45);
    //back from 7th goal

    manualControl = false;
    setIntakeSpeed(200);
    odomMoveTo5(mmToEnc(1050), mmToEnc(2170), 11000, 15, 0.9, 150);
    gyroTurn4(0, 0.9);
    drivePIDSlewExitSlow(mmToEnc(290), 11000, 150, 0, 2000);
    autonShoot2(4);
    chassisDriveModel->stop();
    manualControl = true;
    drivePIDSlewAngle(mmToEnc(-250), 15, 11000, 150, 0);
    //back from 8th goal

    rollerLower.moveVelocity(-200);
    rollerUpper.moveVelocity(-200);
    setIntakeSpeed(-200);
    gyroTurn4(45, 0.9);
    manualControl = false;
    setIntakeSpeed(200);
    odomMoveTo5(mmToEnc(1050), mmToEnc(1450), 11000, 15, 0.9, 150);
    gyroTurn4Custom(168, 1.5, 250, 12, 1800, 2600);
    setIntakeSpeed(-200);
    chassisDriveModel->tank(0.7, 1);
    delay(500);
    chassisDriveModel->tank(.5, .5);
    delay(200);
    autonShoot2(4);
    setIntakeSpeed(-200);
    delay(200);
    chassisDriveModel->tank(-.5, -.5);
    delay(500);
    chassisDriveModel->tank(.8, .8);
    delay(600);
    chassisDriveModel->tank(-.5, -.5);
    delay(700);
    chassisDriveModel->tank(.8, .8);
    delay(600);
    chassisDriveModel->tank(-.5, -.5);
    delay(500);
    chassisDriveModel->stop();
    //drivePID3(mmToEnc(-500), 15, 9000);
    setIntakeSpeed(0);
    manualControl = true;
    rollerUpper.moveVelocity(0);
    rollerLower.moveVelocity(0);
    //yes */
}