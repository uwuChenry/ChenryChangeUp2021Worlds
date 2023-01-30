#include "main.h"

void generatePaths(){
    profileController->generatePath({{0_cm, 0_cm, 0_deg}, {120_cm, 50_cm, 0_deg}}, "2BallStart");
    profileController->generatePath({{0_cm, 0_cm, 0_deg}, {48_cm, 0_cm, 0_deg}, {146_cm, -30_cm, 0_deg}}, "2Ball2nd");
    profileController->generatePath({{0_cm, 0_cm, 0_deg}, {48_cm, 0_cm, 0_deg}, {148_cm, -30_cm, 0_deg}, {158_cm, -30_cm, 0_deg}}, "2Ball3rd");
    //profileController->generatePath({{0_cm, 0_cm, 0_deg}, {80_cm, 30_cm, 45_deg}}, "curve");
}



void skillv4(){
    setIntakeSpeed(200);
    manualControl=false;
    profileController->setTarget("2BallStart", false);
    profileController->waitUntilSettled();
    profileController->removePath("2BallStart");
    drivePIDSlew(mmToEnc(-620), 15, 11000, 200);
    gyroTurn4(45, 0.9);
    setIntakeSpeed(100);
    drivePIDSlewExitSlow(mmToEnc(800), 11000, 200, 45, 4000);
    delay(200);
    chassisDriveModel->stop();
    autonShoot2(2);
    manualControl = true;
    rollerLower.moveVelocity(-200);
    rollerUpper.moveVelocity(-200);
    setIntakeSpeed(-80);
    drivePIDSlew(mmToEnc(-700), 15, 11000, 200);
    //back away from 1st

    manualControl = false;
    setIntakeSpeed(200);
    odomMoveTo5(mmToEnc(-780), mmToEnc(-320), 11000, 15, 0.9, 120);
    gyroTurn4(90, 0.9);
    drivePIDSlewExitSlow(mmToEnc(800), 11000, 200, 90, 4000);
    setIntakeSpeed(130);
    delay(300);
    chassisDriveModel->stop();
    manualControl = true;
    rollerUpper.moveVelocity(200);
    rollerLower.moveVelocity(70);
    delay(650);
    rollerUpper.moveVelocity(200);
    delay(100);
    setIntakeSpeed(-100);
    rollerUpper.moveVelocity(0);
    rollerLower.moveVelocity(0);
    drivePIDSlew(mmToEnc(-180), 20, 11000, 200);
    setIntakeSpeed(-200);
    rollerUpper.moveVelocity(-200);
    rollerLower.moveVelocity(-200);
    gyroTurn4(45, 5);
    gyroTurn4(180, 1);
    //finish 2nd goal

    manualControl = false;
    setIntakeSpeed(200);
    profileController->setTarget("2Ball2nd");
    profileController->waitUntilSettled();
    drivePIDSlew(mmToEnc(-520), 15, 11000, 200);
    printDistance();
    odomReset(816, 839);
    gyroTurn4(135, 0.9);
    setIntakeSpeed(150);
    drivePIDSlewExitSlow(mmToEnc(800), 11000, 200, 135, 4000);
    delay(200);
    chassisDriveModel->stop();
    autonShoot2(2);
    manualControl = true;
    rollerLower.moveVelocity(-200);
    rollerUpper.moveVelocity(-200);
    setIntakeSpeed(-80);
    drivePIDSlew(mmToEnc(-700), 15, 11000, 200);
    //finish 3rd goal

    manualControl = false;
    setIntakeSpeed(200);
    odomMoveTo5(mmToEnc(-935), mmToEnc(285), 11000, 15, 0.9, 200);
    gyroTurn4(180, 0.9);
    drivePIDSlewExitSlow(mmToEnc(800), 11000, 200, 180, 4000);
    setIntakeSpeed(130);
    delay(300);
    chassisDriveModel->stop();
    manualControl = true;
    rollerUpper.moveVelocity(200);
    rollerLower.moveVelocity(50);
    delay(650);
    rollerUpper.moveVelocity(200);
    delay(100);
    setIntakeSpeed(-100);
    rollerUpper.moveVelocity(0);
    rollerLower.moveVelocity(0);
    drivePIDSlew(mmToEnc(-500), 15, 11000, 200);
    setIntakeSpeed(-200);
    rollerUpper.moveVelocity(-200);
    rollerLower.moveVelocity(-200);
    gyroTurn4(135, 5);
    gyroTurn4(-90, 1);
    //finish 4th goal

    manualControl = false;
    setIntakeSpeed(200);
    drivePIDSlew(mmToEnc(1200), 15, 11000, 200);
    odomReset(472, 762);
    drivePIDSlew(mmToEnc(-320), 20, 11000, 200);
    gyroTurn4(180, 1);
    drivePIDSlew(mmToEnc(600), 10, 11000, 200);
    drivePIDSlew(mmToEnc(-590), 10, 11000, 200);
    gyroTurn4(-135, 0.9);
    drivePIDSlewExitSlow(mmToEnc(850), 11000, 200, -135, 4000);
    delay(200);
    chassisDriveModel->stop();
    autonShoot2(2);
    manualControl = true;
    rollerLower.moveVelocity(-200);
    rollerUpper.moveVelocity(-200);
    setIntakeSpeed(-80);
    drivePIDSlew(mmToEnc(-700), 15, 11000, 200);
    //finish 5th goal

    manualControl = false;
    setIntakeSpeed(200);
    odomMoveTo5(mmToEnc(520), mmToEnc(885), 11000, 15, 0.9, 200);
    gyroTurn4(-90, 0.9);
    drivePIDSlewExitSlow(mmToEnc(800), 11000, 200, -90, 4000);
    setIntakeSpeed(130);
    delay(300);
    chassisDriveModel->stop();
    manualControl = true;
    rollerUpper.moveVelocity(200);
    rollerLower.moveVelocity(50);
    delay(450);
    rollerUpper.moveVelocity(200);
    delay(50);
    setIntakeSpeed(-100);
    rollerUpper.moveVelocity(0);
    rollerLower.moveVelocity(0);
    drivePIDSlew(mmToEnc(-180), 20, 11000, 200);
    setIntakeSpeed(-200);
    rollerUpper.moveVelocity(-200);
    rollerLower.moveVelocity(-200);
    gyroTurn4(-135, 5);
    gyroTurn4(0, 0.9);
    //finish 6th goal

    manualControl = false;
    setIntakeSpeed(200);
    profileController->setTarget("2Ball3rd");
    profileController->waitUntilSettled();
    profileController->removePath("2Ball2nd");
    profileController->removePath("2Ball3rd");
    drivePIDSlewAngle(mmToEnc(-540), 10, 11000, 200, 0);
    //drivePIDSlew(mmToEnc(-520), 10, 11000, 200);
    printDistance();
    odomReset(748, 695);
    gyroTurn4(-45, 0.9);
    setIntakeSpeed(100);
    drivePIDSlewExitSlow(mmToEnc(800), 11000, 200, -45, 4000);
    delay(200);
    chassisDriveModel->stop();
    autonShoot2(2);
    setIntakeSpeed(-100);
    manualControl = true;
    rollerUpper.moveVelocity(-200);
    rollerLower.moveVelocity(-200);
    drivePIDSlew(mmToEnc(-700), 15, 11000, 200);
    //finish 7th

    setIntakeSpeed(200);
    manualControl = false;
    odomMoveTo5(mmToEnc(800), mmToEnc(320), 11000, 15, 0.9, 200);
    gyroTurn4(0, 0.9);
    setIntakeSpeed(200);
    /*
    drivePIDexitSlow(mmToEnc(280), 4000, 9000);
    delay(100);
    chassisDriveModel->stop();
    autonShoot2(4);
    delay(100);
    manualControl = true;
    setIntakeSpeed(-100);
    rollerLower.moveVelocity(-200);
    rollerUpper.moveVelocity(-200);
    drivePIDSlew(mmToEnc(-200), 15, 11000, 200);
    //finish 8th goal

    setIntakeSpeed(200);
    manualControl = false;
    odomMoveTo5(mmToEnc(830), mmToEnc(-200), 11000, 15, 0.9, 200);*/
}