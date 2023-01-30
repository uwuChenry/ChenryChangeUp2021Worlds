#include "main.h"

Controller controller;
pros::Imu inertialSensor(1);
pros::Optical opticalUp (14);
pros::Optical opticalDown (15);
pros::Distance distBack (17);
pros::Distance distSide (18);
pros::Distance distFront (12);

int opticalBallValue = 150;

bool upIsBlue;
double intakeSpeed_ = 0;

void printDistance (){
    pros::lcd::print(6, "%d distFront", distFront.get());
    pros::lcd::print(7, "%d distSide", distSide.get());
}
void odomSetPos (double front, double side){
    double mmDistFront = distFront.get();
    double mmDistSide = distSide.get();
    double displacementF = mmDistFront - front;
    double displacementS = mmDistSide - side;
    double dispFInch = mmToEnc(displacementF);
    double dispSInch = mmToEnc(displacementS) * -1;
    odom::bPosX = dispSInch + 3400;
    odom::bPosY = dispFInch + 700;

}

void odomReset (double front, double Lside){
    double mmDistFront = distFront.get();
    double mmDistSide = distSide.get();
    double displacementF = mmDistFront - front;
    double displacementS = mmDistSide - Lside;
    double dispFInch = mmToEnc(displacementF);
    double dispSInch = mmToEnc(displacementS) * -1;
    odom::bPosX = dispSInch;
    odom::bPosY = dispFInch;

}

void frontDistPIDSlew(double Dist, double tol, double speed, double angle, double accel){
    double step, lastError, derivative, error, motorSpeed, integral = 0;
    int counter;
    double speedCapped = 5000;
    double kP, kI, kD, kI_Windup, akP = 0.01;
    
    kP = 19; kI = 0.49; kD = 11.7192; kI_Windup = 1542;

    while (counter <= 8){
        if (distFront.get() == 0){
            error = 1000;
        }
        else{
            error = distFront.get() - Dist;
        }
        derivative = error - lastError;
        integral += error;
        double aError = constrainAngle(angle - inertialSensor.get_heading());
        double aOutput = aError * akP;

        if (abs(error) > 81) integral = 0;
        if (integral > kI_Windup) integral = kI_Windup; 
        else if (integral < -kI_Windup) integral = -kI_Windup;

        step = (error*kP) + (integral * kI) + (derivative*kD);

        speedCapped += accel;
       
        if (speedCapped > speed) speedCapped = speed;
        else if (speedCapped < -speed) speedCapped = -speed; 

        if (step > speedCapped) motorSpeed = speedCapped;
        else if (step < -speedCapped) motorSpeed = -speedCapped;
        else motorSpeed = step;
        
        lastError = error;

        chassisDriveModel->tank(((motorSpeed/12000) + aOutput), ((motorSpeed/12000) - aOutput));

        pros::lcd::print(2, "%.2f step", step);
        pros::lcd::print(1, "%.2f motorspeed", motorSpeed);
        pros::lcd::print(4, "%.2f error", error);
        pros::lcd::print(5, "%.2f integral", integral);
        
        if (fabs(error) <= tol) {
            counter++;
            integral = 0;
        }
        else counter = 0;
        delay(10);
    }
    chassisDriveModel->stop();
}

void frontDistPIDSlewCustom(double Dist, double tol, double speed, double angle, double accel, double kP, double kI, double kD, double kI_Windup){
    double step, lastError, derivative, error, motorSpeed, integral = 0;
    int counter;
    double speedCapped = 5000;
    double akP = 0.01;

    while (counter <= 8){
        if (distFront.get() == 0){
            error = 1000;
        }
        else{
            error = distFront.get() - Dist;
        }
        derivative = error - lastError;
        integral += error;
        double aError = constrainAngle(angle - inertialSensor.get_heading());
        double aOutput = aError * akP;

        if (abs(error) > 150) integral = 0;
        if (integral > kI_Windup) integral = kI_Windup; 
        else if (integral < -kI_Windup) integral = -kI_Windup;

        step = (error*kP) + (integral * kI) + (derivative*kD);

        speedCapped += accel;
       
        if (speedCapped > speed) speedCapped = speed;
        else if (speedCapped < -speed) speedCapped = -speed; 

        if (step > speedCapped) motorSpeed = speedCapped;
        else if (step < -speedCapped) motorSpeed = -speedCapped;
        else motorSpeed = step;
        
        lastError = error;

        chassisDriveModel->tank(((motorSpeed/12000) + aOutput), ((motorSpeed/12000) - aOutput));

        pros::lcd::print(2, "%.2f step", step);
        pros::lcd::print(1, "%.2f motorspeed", motorSpeed);
        pros::lcd::print(4, "%.2f error", error);
        pros::lcd::print(5, "%.2f integral", integral);
        
        if (fabs(error) <= tol) {
            counter++;
            integral = 0;
        }
        else counter = 0;
        delay(10);
    }
    chassisDriveModel->stop();
}

void setIntakeSpeed(double speed){
    intakeSpeed_ = speed;
}

void intakeAutonControl (void*) {
    int counter = 0;
    while (1){
        double averageTorque = ((intakeL.getTorque() + intakeR.getTorque()) / 2);
        if (counter > 20) {
        intakeR.moveVelocity(-200);
        intakeL.moveVelocity(0);
        delay(30);
        counter = 0;
        }
        if (fabs(intakeSpeed_) != 0 && averageTorque > 0.6){
            counter += 1;
        }
        else {
            intakeL.moveVelocity(intakeSpeed_);
            intakeR.moveVelocity(intakeSpeed_);
            counter = 0;
        }
        delay(10);
    }
    
}

void checkForBalls (void*)
{
    while (1){
        if (opticalDown.get_proximity() > opticalBallValue){
            downHasBall = true;
        }
        else{
            downHasBall = false;
        }

        if (opticalUp.get_proximity() > opticalBallValue){
            upHasBall = true;
        }
        else{
            upHasBall = false;

        }
        delay(10);
    }
}
Task ballscheck (checkForBalls);
Task intakeDoubleCheck (intakeAutonControl);

