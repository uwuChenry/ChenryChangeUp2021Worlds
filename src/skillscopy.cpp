#include "main.h"

void skills2() {

    manualControl = false;
    setIntakeSpeed(200);
    drivePID3(1100, 8, 9000);
    gyroTurn4Custom(-135, 1, 250, 9.13, 1800, 2600);
    delay(100);
    setIntakeSpeed(100);
    drivePIDexitSlow(mmToEnc(830), 4000, 9000);
    delay(300);
    chassisDriveModel->stop();
    autonShoot2(2);
    manualControl = true;
    rollerLower.moveVelocity(-200);
    rollerUpper.moveVelocity(-200);
    setIntakeSpeed(-100);
    drivePID3(mmToEnc(-400), 8, 9000);
    // backing away from 1st goal


    setIntakeSpeed(200);
    manualControl = false;
    odomMoveTo5(400, 2550, 11000, 8, 0.8, 150);
    odomMoveTo5NS(-830, 2540, 11000, 4000, 0.8, 150);
    setIntakeSpeed(100);
    delay(300);
    chassisDriveModel->stop();
    manualControl = true;
    rollerUpper.moveVelocity(200);
    rollerLower.moveVelocity(50);
    delay(650);
    setIntakeSpeed(0);
    rollerUpper.moveVelocity(0);
    rollerLower.moveVelocity(0);
    drivePID3(mmToEnc(-500), 8, 9000);
    setIntakeSpeed(-200);
    rollerUpper.moveVelocity(-200);
    rollerLower.moveVelocity(-200);
    gyroTurn4(-135, 5);
    
    //backing away from 2nd goal


    setIntakeSpeed(200);
    manualControl = false;
    odomMoveTo5(-40, 4500, 11000, 8, 0.9, 150);
    drivePID3(-500, 8, 9000);
    odomMoveTo5NS(-900, 4920, 11000, 4000, 0.8, 150);
    delay(300);
    chassisDriveModel->stop();
    autonShoot2(1);
    delay(300);
    manualControl = true;
    setIntakeSpeed(-100);
    rollerUpper.moveVelocity(-200);
    rollerLower.moveVelocity(-200);
    drivePID3(mmToEnc(-500), 8, 9000);
    //move back from 3rd goal


    manualControl = false;
    setIntakeSpeed(200);
    odomMoveTo5(1480, 3550, 11000, 8, 0.8, 150);
    odomMoveTo5NS(1450, 4900, 11000, 4000, 0.9, 150);
    delay(300);
    chassisDriveModel->stop();
    autonShoot2(4);
    delay(50);
    manualControl = true;
    setIntakeSpeed(0);
    rollerUpper.moveVelocity(0);
    rollerLower.moveVelocity(0);
    drivePID3(mmToEnc(-700), 8, 9000);
    setIntakeSpeed(-200);
    rollerUpper.moveVelocity(-200);
    rollerLower.moveVelocity(-200);
    gyroTurn4(-45, 5);
    //move back from 4th goal

    
    manualControl = false;
    setIntakeSpeed(200);
    odomMoveTo5NS(3650, 5010, 11000, 5000, 1, 150);
    delay(300);
    chassisDriveModel->stop();
    autonShoot2(4);
    delay(350);
    manualControl = true;
    setIntakeSpeed(-200);
    rollerUpper.moveVelocity(-200);
    rollerLower.moveVelocity(-200);
    drivePID3(mmToEnc(-400), 8, 9000);
    //move back from 5th goal


    manualControl = false;
    setIntakeSpeed(200);
    odomMoveTo5(2505, 2690, 11000, 8, 0.9, 150);
    odomMoveTo5NS(4000, 2690, 11000, 4000, 0.8, 150);
    delay(300);
    chassisDriveModel->stop();
    manualControl = true;
    rollerUpper.moveVelocity(200);
    rollerLower.moveVelocity(50);
    delay(700);
    setIntakeSpeed(-200);
    rollerUpper.moveVelocity(0);
    rollerLower.moveVelocity(0);
    drivePID3(mmToEnc(-500), 8, 9000);
    setIntakeSpeed(-200);
    rollerUpper.moveVelocity(-200);
    rollerLower.moveVelocity(-200);
    gyroTurn4(45, 5);
    //move back from 6th goal

    setIntakeSpeed(200);
    manualControl = false;
    odomMoveTo5(3090, 700, 11000, 8, 0.8, 150);
    gyroTurn4(180, 1.5);
    printDistance();
    odomSetPos(480, 775);
    drivePID3(-400, 10, 7000);
    gyroTurn4(90, 0.8);
    drivePID3(950, 10, 9000);
    drivePID3(-800, 10, 9000);
    odomMoveTo5NS(4100, 405, 11000, 5000, 0.8, 150);
    delay(400);
    chassisDriveModel->stop();
    autonShoot2(1);
    delay(400);
    setIntakeSpeed(-100);
    rollerLower.moveVelocity(-100);
    drivePID3(mmToEnc(-300), 8, 9000);
    setIntakeSpeed(-50);
    //move back from 7th goal

    odomMoveTo5(1850, 800, 11000, 8, 0.9, 150);
    manualControl = false;
    setIntakeSpeed(200);
    odomMoveTo5NS(1850, 200, 11000, 4000, 0.9, 150);
    delay(300);
    autonShoot2(4);
    setIntakeSpeed(0);
    drivePID3(mmToEnc(-250), 8, 9000);
    //omve back from 8th goal

    manualControl = true;
    setIntakeSpeed(-200);
    rollerLower.moveVelocity(-200);
    rollerUpper.moveVelocity(-200);
    gyroTurn4(-135, 5);
    manualControl = false;


    setIntakeSpeed(200);
    odomMoveTo5NS(1750, 2000, 9000, 5000, 0.8, 150);
    gyroTurn4Custom(-8, 1.5, 250, 12, 1800, 2600);
    chassisDriveModel->tank(0.5, 1);
    delay(500);
    chassisDriveModel->tank(.5, .5);
    delay(200);
    autonShoot2(4);
    drivePID3(mmToEnc(-500), 8, 9000);
}