#include "main.h"
#include <vector>

ADIEncoder enc ('E', 'F', true);
ADIEncoder encSide ('C', 'D', false);
MotorGroup LeftDriveMotors({-20, -8});
MotorGroup RightDriveMotors({7, 3});

bool gyroIsReset = false;



std::shared_ptr<OdomChassisController> Basechassis = ChassisControllerBuilder()
.withMotors(LeftDriveMotors, RightDriveMotors)
.withDimensions(AbstractMotor::gearset::green, {{5.8_in, 11.81_in}, imev5GreenTPR})
.withOdometry()
.buildOdometry();

std::shared_ptr<AsyncMotionProfileController> profileController = AsyncMotionProfileControllerBuilder()
.withLimits({1, 1.5, 10})
.withOutput(Basechassis)
.buildMotionProfileController(); 

std::shared_ptr<ChassisModel> chassisDriveModel = Basechassis->getModel();


void doTheThing(std::vector<std::pair<double, double>>& v) {
    for (auto& step : v) {
        auto [left, right] = step;
        left = left * 12 * 60 / (2 * M_PI) / 1.625 * 5 / 9;
        right = right * 12 * 60 / (2 * M_PI) / 1.625 * 5 / 9;
        LeftDriveMotors.moveVelocity(left);
        RightDriveMotors.moveVelocity(right);
        delay(10);
    }
    LeftDriveMotors.moveVoltage(0);
    RightDriveMotors.moveVoltage(0);
}

void printMotorEncoders(){
    pros::lcd::print(1, "%.2f left", LeftDriveMotors.getPosition());
    pros::lcd::print(2, "%.2f right", RightDriveMotors.getPosition());
}

void resetMotorEncoders(){
    LeftDriveMotors.tarePosition();
    RightDriveMotors.tarePosition();
}

void lockChassis (){
    LeftDriveMotors.setBrakeMode(AbstractMotor::brakeMode::brake);
	RightDriveMotors.setBrakeMode(AbstractMotor::brakeMode::brake);
}

void unlockChassis () {
    LeftDriveMotors.setBrakeMode(AbstractMotor::brakeMode::coast);
	RightDriveMotors.setBrakeMode(AbstractMotor::brakeMode::coast);
}

void slowlySetMotor (double speed) {
    double motorSpeed = (speed) / 200;
    chassisDriveModel->left(motorSpeed);
    chassisDriveModel->right(motorSpeed);
}


void drivePIDSlew(double distance, double tol, double speed, double accel){
    double step, lastError, derivative, error, motorSpeed, integral = 0;
    int counter;
    double speedCapped = 5000;
    double startingEnc = enc.get();
    double kP, kI, kD, kI_Windup, akP = 0.03;
    double initGyro = inertialSensor.get_heading();
    
    if (distance >= 0){//forwards
        if (abs(distance) < 800){
            kP = 12.5; kI = 0.49; kD = 19; kI_Windup = 2500; speedCapped = 4000;
            pros::lcd::print(1, "small");
        }
        else{
            kP = 15.5; kI = 0.52; kD = 38; kI_Windup = 2500; speedCapped = 4000; 
            pros::lcd::print(1, "long");
        }
    }
    else if (distance < 0){// backwards
        if (abs(distance) < 800){
            kP = 12.5; kI = 0.49; kD = 19; kI_Windup = 2500; 
            pros::lcd::print(1, "im doing it pogn't");
        }
        if (abs(distance) < 350){
            kP = 25; kI = 0.49; kD = 38; kI_Windup = 2500; 
            pros::lcd::print(1, "im doing it pog");
        }
        else{
            kP = 15.5; kI = 0.52; kD = 19; kI_Windup = 2500; 
            pros::lcd::print(1, "this is the one im using");
        }
    }

    while (counter <= 8){
        error = (distance + startingEnc) - enc.get();
        derivative = error - lastError;
        integral += error;
        double aError = constrainAngle (initGyro - inertialSensor.get_heading());
        double aOutput = aError * akP;

        if (abs(error) > 100){
            integral = 0;
        }
        if (integral > kI_Windup){
            integral = kI_Windup;
        }
        else if (integral < -kI_Windup){
            integral = -kI_Windup;
        }

        step = (error*kP) + (integral * kI) + (derivative*kD);
        lastError = error;

        speedCapped += accel;

        if (speedCapped > speed){
            speedCapped = speed;
        }
        else if (speedCapped < -speed){
            speedCapped = -speed;
        }

        if (step > speedCapped){
            motorSpeed = speedCapped;
        }
        else if (step < -speedCapped){
            motorSpeed = -speedCapped;
        }
        else {
            motorSpeed = step;
        }
        
        chassisDriveModel->tank(((motorSpeed/12000) + aOutput), ((motorSpeed/12000) - aOutput));

        if (fabs(error) <= tol) {
            counter++;
            integral = 0;
        }
        else {
            counter = 0;
        }
        delay(10);
        pros::lcd::print(7, "%.2f errpr", error);
    }
    chassisDriveModel->stop();   
}

void drivePIDSlewAngle(double distance, double tol, double speed, double accel, double angle){
    double step, lastError, derivative, error, motorSpeed, integral = 0;
    int counter;
    double speedCapped = 5000;
    double startingEnc = enc.get();
    double kP, kI, kD, kI_Windup, akP = 0.03;
    double initGyro = angle;
    
    if (distance >= 0){//forwards
        if (abs(distance) < 800){
            kP = 12.5; kI = 0.49; kD = 19; kI_Windup = 2500; speedCapped = 4000;
            pros::lcd::print(1, "small");
        }
        else{
            kP = 15.5; kI = 0.52; kD = 38; kI_Windup = 2500; speedCapped = 4000; 
            pros::lcd::print(1, "long");
        }
    }
    else if (distance < 0){// backwards
        if (abs(distance) < 800){
            kP = 12.5; kI = 0.49; kD = 19; kI_Windup = 2500; 
            pros::lcd::print(1, "im doing it pogn't");
        }
        if (abs(distance) < 350){
            kP = 25; kI = 0.49; kD = 38; kI_Windup = 2500; 
            pros::lcd::print(1, "im doing it pog");
        }
        else{
            kP = 15.5; kI = 0.52; kD = 19; kI_Windup = 2500; 
            pros::lcd::print(1, "this is the one im using");
        }
    }

    while (counter <= 8){
        error = (distance + startingEnc) - enc.get();
        derivative = error - lastError;
        integral += error;
        double aError = constrainAngle (initGyro - inertialSensor.get_heading());
        double aOutput = aError * akP;

        if (abs(error) > 100){
            integral = 0;
        }
        if (integral > kI_Windup){
            integral = kI_Windup;
        }
        else if (integral < -kI_Windup){
            integral = -kI_Windup;
        }

        step = (error*kP) + (integral * kI) + (derivative*kD);
        lastError = error;

        speedCapped += accel;

        if (speedCapped > speed){
            speedCapped = speed;
        }
        else if (speedCapped < -speed){
            speedCapped = -speed;
        }

        if (step > speedCapped){
            motorSpeed = speedCapped;
        }
        else if (step < -speedCapped){
            motorSpeed = -speedCapped;
        }
        else {
            motorSpeed = step;
        }
        
        chassisDriveModel->tank(((motorSpeed/12000) + aOutput), ((motorSpeed/12000) - aOutput));

        if (fabs(error) <= tol) {
            counter++;
            integral = 0;
        }
        else {
            counter = 0;
        }
        delay(10);
        pros::lcd::print(7, "%.2f errpr", error);
    }
    chassisDriveModel->stop();   
}

void drivePIDSlewAngleWithBackup(double distance, double tol, double speed, double accel, double angle, int dRange){
    double step, lastError, derivative, error, motorSpeed, integral = 0;
    int counter, dCounter = 0;
    double speedCapped = 5000;
    double startingEnc = enc.get();
    double kP, kI, kD, kI_Windup, akP = 0.03;
    double initGyro = angle;
    
    if (distance >= 0){//forwards
        if (abs(distance) < 800){
            kP = 12.5; kI = 0.49; kD = 19; kI_Windup = 2500; speedCapped = 4000;
            pros::lcd::print(1, "small");
        }
        else{
            kP = 15.5; kI = 0.52; kD = 38; kI_Windup = 2500; speedCapped = 4000; 
            pros::lcd::print(1, "long");
        }
    }
    else if (distance < 0){// backwards
        if (abs(distance) < 800){
            kP = 12.5; kI = 0.49; kD = 19; kI_Windup = 2500; 
            pros::lcd::print(1, "im doing it pogn't");
        }
        if (abs(distance) < 350){
            kP = 25; kI = 0.49; kD = 38; kI_Windup = 2500; 
            pros::lcd::print(1, "im doing it pog");
        }
        else{
            kP = 15.5; kI = 0.52; kD = 19; kI_Windup = 2500; 
            pros::lcd::print(1, "this is the one im using");
        }
    }

    while (counter <= 8 || dCounter <=10){
        error = (distance + startingEnc) - enc.get();
        derivative = error - lastError;
        integral += error;
        double aError = constrainAngle (initGyro - inertialSensor.get_heading());
        double aOutput = aError * akP;

        if (abs(error) > 100){
            integral = 0;
        }
        if (integral > kI_Windup){
            integral = kI_Windup;
        }
        else if (integral < -kI_Windup){
            integral = -kI_Windup;
        }

        step = (error*kP) + (integral * kI) + (derivative*kD);
        lastError = error;

        speedCapped += accel;

        if (speedCapped > speed){
            speedCapped = speed;
        }
        else if (speedCapped < -speed){
            speedCapped = -speed;
        }

        if (step > speedCapped){
            motorSpeed = speedCapped;
        }
        else if (step < -speedCapped){
            motorSpeed = -speedCapped;
        }
        else {
            motorSpeed = step;
        }
        
        chassisDriveModel->tank(((motorSpeed/12000) + aOutput), ((motorSpeed/12000) - aOutput));

        if (fabs(error) <= tol) {
            counter++;
            integral = 0;
        }
        else {
            counter = 0;
        }

        if (fabs(derivative) <= 2) {
            dCounter++;
        }
        if (fabs(derivative) > 2){
            dCounter = 0;
        }
        if (dCounter > 10) break;
        delay(10);
        pros::lcd::print(7, "%.2f errpr", error);
        printf("%.2f derivative\n", derivative);
        printf("%d dcounter\n", dCounter);
    }
    chassisDriveModel->stop();   
}



void drivePIDSlewExitSlow(double distance, double speed, double accel, double angle, double exitSpeed){
    double step, lastError, derivative, error, motorSpeed, integral = 0;
    int counter;
    double speedCapped;
    double startingEnc = enc.get();
    double kP, kI, kD, kI_Windup, akP = 0.07;
    double initGyro = angle;
    
    if (distance >= 0){//forwards
        if (abs(distance) < 800){
            kP = 12.5; kI = 0.49; kD = 19; kI_Windup = 2500; speedCapped = 4000;
        }
        else{
            kP = 12.5; kI = 0.52; kD = 19; kI_Windup = 2500; speedCapped = 4000; 
        }
    }
    else if (distance < 0){// backwards
        if (abs(distance) < 800){
            kP = 12.5; kI = 0.49; kD = 19; kI_Windup = 2500; 
        }
        else{
            kP = 12.5; kI = 0.52; kD = 19; kI_Windup = 2500; 
        }
    }

    while (counter <= 8){
        error = (distance + startingEnc) - enc.get();
        derivative = error - lastError;
        integral += error;
        double aError = constrainAngle (initGyro - inertialSensor.get_heading());
        double aOutput = aError * akP;

        if (abs(error) > 100){
            integral = 0;
        }
        if (integral > kI_Windup){
            integral = kI_Windup;
        }
        else if (integral < -kI_Windup){
            integral = -kI_Windup;
        }

        step = (error*kP) + (integral * kI) + (derivative*kD);
        lastError = error;

        speedCapped += accel;

        if (speedCapped > speed){
            speedCapped = speed;
        }
        else if (speedCapped < -speed){
            speedCapped = -speed;
        }

        if (step > speedCapped){
            motorSpeed = speedCapped;
        }
        else if (step < -speedCapped){
            motorSpeed = -speedCapped;
        }
        else {
            motorSpeed = step;
        }
        
        chassisDriveModel->tank(((motorSpeed/12000) + aOutput), ((motorSpeed/12000) - aOutput));

        if (fabs(step) <= exitSpeed){
            counter++;
        }
        else {
            counter = 0;
        }
        delay(10);
    }
    chassisDriveModel->stop();   
}

void drivePIDslewWithGyro(double distance, double tol, double speed, double accel, double gyroangle){
    double step, lastError, derivative, error, motorSpeed, integral = 0;
    int counter;
    double speedCapped;
    double startingEnc = enc.get();
    double kP, kI, kD, kI_Windup, akP = 0.07;
    double initGyro = gyroangle;
    
    if (distance >= 0){//forwards
        if (abs(distance) < 800){
            kP = 12.5; kI = 0.49; kD = 19; kI_Windup = 2500; speedCapped = 4000;
        }
        else{
            kP = 12.5; kI = 0.52; kD = 19; kI_Windup = 2500; speedCapped = 4000; 
        }
    }
    else if (distance < 0){// backwards
        if (abs(distance) < 800){
            kP = 12.5; kI = 0.49; kD = 19; kI_Windup = 2500; 
        }
        else{
            kP = 12.5; kI = 0.52; kD = 19; kI_Windup = 2500; 
        }
    }

    while (counter <= 8){
        error = (distance + startingEnc) - enc.get();
        derivative = error - lastError;
        integral += error;
        double aError = constrainAngle (initGyro - inertialSensor.get_heading());
        double aOutput = aError * akP;

        if (abs(error) > 100){
            integral = 0;
        }
        if (integral > kI_Windup){
            integral = kI_Windup;
        }
        else if (integral < -kI_Windup){
            integral = -kI_Windup;
        }

        step = (error*kP) + (integral * kI) + (derivative*kD);
        lastError = error;

        speedCapped += accel;

        if (speedCapped > speed){
            speedCapped = speed;
        }
        else if (speedCapped < -speed){
            speedCapped = -speed;
        }

        if (step > speedCapped){
            motorSpeed = speedCapped;
        }
        else if (step < -speedCapped){
            motorSpeed = -speedCapped;
        }
        else {
            motorSpeed = step;
        }
        
        chassisDriveModel->tank(((motorSpeed/12000) + aOutput), ((motorSpeed/12000) - aOutput));

        if (fabs(error) <= tol) {
            counter++;
            integral = 0;
        }
        else {
            counter = 0;
        }
        delay(10);
    }
    chassisDriveModel->stop();   
}


void drivePID3(double distance, double tol, double speed){
    double step, lastError, derivative, error, motorSpeed, integral = 0;
    int counter;
    double startingEnc = enc.get();
    double kP, kI, kD, kI_Windup, akP = 0.01;
    double initGyro = inertialSensor.get_heading();

    if (distance >= 0){//forwards
        if (abs(distance) < 800){
            kP = 12.5; kI = 0.49; kD = 19; kI_Windup = 2500;
        }
        else{
            kP = 12.5; kI = 0.52; kD = 19; kI_Windup = 2500;
        }
    }
    else if (distance < 0){// backwards
        if (abs(distance) < 800){
            kP = 12.5; kI = 0.49; kD = 19; kI_Windup = 2500;
        }
        else{
            kP = 12.5; kI = 0.52; kD = 19; kI_Windup = 2500;
        }
    }

    while (counter <= 8){
        error = (distance + startingEnc) - enc.get();
        derivative = error - lastError;
        integral += error;
        double aError = constrainAngle (initGyro - inertialSensor.get_heading());
        double aOutput = aError * akP;

        if (abs(error) > 100){
            integral = 0;
        }
        if (integral > kI_Windup){
            integral = kI_Windup;
        }
        else if (integral < -kI_Windup){
            integral = -kI_Windup;
        }
        step = (error*kP) + (integral * kI) + (derivative*kD);
        if (step > speed){
            motorSpeed = speed;
        }
        else if (step < -speed){
            motorSpeed = -speed;
        }
        else {
            motorSpeed = step;
        }
        
        lastError = error;
        chassisDriveModel->tank(((motorSpeed/12000) + aOutput), ((motorSpeed/12000) - aOutput));
        pros::lcd::print(2, "%.2f step", step);
        pros::lcd::print(1, "%.2f motorspeed", motorSpeed);
        pros::lcd::print(3, "%.2f distance", enc.get());
        pros::lcd::print(4, "%.2f error", error);
        pros::lcd::print(5, "%.2f integral", integral);
        if (fabs(error) <= tol) {
            counter++;
            integral = 0;
        }
        else {
            counter = 0;
        }
        delay(10);
    }
    chassisDriveModel->stop();
}

void drivePIDexitSlow(double distance, double exitSpeed, double speed){
double step, lastError, derivative, error, motorSpeed, integral = 0;
    int counter;
    double startingEnc = enc.get();
    double kP, kI, kD, kI_Windup, akP = 0.01;
    double initGyro = inertialSensor.get_heading();

    if (distance >= 0){//forwards
        if (abs(distance) < 800){
            kP = 16; kI = .7; kD = 92; kI_Windup = 2600;
        }
        else{
            kP = 16; kI = .8; kD = 72; kI_Windup = 2600;
        }
    }
    else if (distance < 0){// backwards
        if (abs(distance) < 800){
            kP = 16; kI = .7; kD = 92; kI_Windup = 2600;
        }
        else{
            kP = 16; kI = 0.8; kD = 72; kI_Windup = 2600;
        }
    }

    while (counter <= 2){
        error = (distance + startingEnc) - enc.get();
        derivative = error - lastError;
        integral += error;
        double aError = constrainAngle (initGyro - inertialSensor.get_heading());
        double aOutput = aError * akP;
        if (abs(error) > 100){
            integral = 0;
        }
        if (integral > kI_Windup){
            integral = kI_Windup;
        }
        else if (integral < -kI_Windup){
            integral = -kI_Windup;
        }
        step = (error*kP) + (integral * kI) + (derivative*kD);
        if (step > speed){
            motorSpeed = speed;
        }
        else if (step < -speed){
            motorSpeed = -speed;
        }
        else {
            motorSpeed = step;
        }
        
        lastError = error;
        chassisDriveModel->tank(((motorSpeed/12000) + aOutput), ((motorSpeed/12000) - aOutput));
        delay(10);
        if (fabs(step) <= exitSpeed){
            counter++;
        }
        else {
            counter = 0;
        }
    }
}

void drivePIDwithTime(double distance, double speed, double time){
    int init = pros::millis();
    while (1){
        double now = pros::millis();
        double timeSpent = now - init;
        if (abs(timeSpent) > time) break;
    }
}

void gyroTurn4(double angle, double tol){
    int atTargetCounter = 0;
    double lastError, derivative, error, mappedError, output;
    double integral = 0;
    double kP = 0, kD = 0, kI = 0, kI_Windup = 0;
    double initAngle = constrainAngle(inertialSensor.get_heading());
    double angleDiff = fabs(constrainAngle(angle) - initAngle);

    if (angleDiff < 39){ //small turn
        kP = 250; kI = 10; kD = 1800; kI_Windup = 2600;
    }
    else if (angleDiff < 155){ //medium turn
        kP = 250; kI = 12.07; kD = 1800; kI_Windup = 2600;
    }
    else {
        kP = 250; kI = 12.4; kD = 1800; kI_Windup = 2800;
    }
    
    while (atTargetCounter < 8)
    {
        error = angle-inertialSensor.get_heading();
        mappedError = constrainAngle(error);
        integral += mappedError;
        if (abs(mappedError)>15){
            integral = 0;
        }
        if (integral > kI_Windup){
            integral = kI_Windup;
        }
        else if (integral < -kI_Windup){
            integral = -kI_Windup;
        }
        derivative = constrainAngle(mappedError - lastError);
        output = ((mappedError*kP) + (integral * kI) + ((derivative)*kD))/12000;
        chassisDriveModel->tank(output, -output);
        lastError = mappedError;
        if (abs(mappedError) <= tol){
            atTargetCounter +=1;
            integral = 0;
        }
        else {
            atTargetCounter = 0;
        }
        
        pros::lcd::print(5, "%.2f mapped error", mappedError);
        pros::lcd::print(6, "%.2f output", output); 
        pros::lcd::print(7, "%.2f inertial sensor readings", inertialSensor.get_heading());
        delay(10);
    }
    chassisDriveModel->stop();
}

void gyroTurn4Custom(double angle, double tol, double kP, double kI, double kD, double kI_Windup){
    int atTargetCounter = 0;
    double lastError, derivative, error, mappedError, output;
    double integral = 0;
    while (atTargetCounter < 8)
    {
        error = angle-inertialSensor.get_heading();
        mappedError = constrainAngle(error);
        integral += mappedError;
        if (abs(mappedError)>15){
            integral = 0;
        }
        if (integral > kI_Windup){
            integral = kI_Windup;
        }
        else if (integral < -kI_Windup){
            integral = -kI_Windup;
        }
        derivative = constrainAngle(mappedError - lastError);
        output = ((mappedError*kP) + (integral * kI) + ((derivative)*kD))/12000;
        chassisDriveModel->tank(output, -output);
        lastError = mappedError;
        if (abs(mappedError) <= tol){
            atTargetCounter +=1;
            integral = 0;
        }
        else {
            atTargetCounter = 0;
        }
        
        pros::lcd::print(5, "%.2f mapped error", mappedError);
        pros::lcd::print(6, "%.2f output", output); 
        pros::lcd::print(7, "%.2f inertial sensor readings", inertialSensor.get_heading());
        delay(10);
    }
    chassisDriveModel->stop();
}

void Drivecontrol()
{
    chassisDriveModel->arcade(controller.getAnalog(ControllerAnalog::leftY), controller.getAnalog(ControllerAnalog::rightX));
}


void odomrun (void*) {
    int lEnc = 0;
    int lSideEnc = 0;
    double lGyro = 0;
    delay(5000);

    while (gyroIsReset){
        int encoder = enc.get();
        int encoderSide = encSide.get();
        double gyro = constrainAngle(inertialSensor.get_heading());
        int dEnc = encoder - lEnc;
        int dSideEnc = encoderSide - lSideEnc;
        double dGyro = gyro - lGyro;
        double dTheta = convertToRad(dGyro);
        odom::update(dEnc, dSideEnc, dTheta);
        lEnc = encoder;
        lSideEnc = encoderSide;
        lGyro = gyro;
        delay(20);
    }
}

void printEncoder (){
    pros::lcd::print(1, "%.2f straight", enc.get());
    pros::lcd::print(2, "%.2f side", encSide.get());
}

void driveEncoder (double speed, double encodervalue){
    double distanceTravelled;
    double startingEnc = enc.get();
    double motorSpeed = speed / 200;
    while ((startingEnc + encodervalue) > fabs(distanceTravelled))
    {
        distanceTravelled = enc.get();
        chassisDriveModel->left(motorSpeed);
        chassisDriveModel->right(motorSpeed);
        delay(10);
    }
    chassisDriveModel->stop();
}

void driveEncoderB (double speed, double targetDistance){
    double encValue = enc.get();
    double startingEnc = enc.get();
    double motorSpeed = speed / 200;
    double target = startingEnc - targetDistance;
    while (target < encValue){
        encValue = enc.get();
        chassisDriveModel->left(motorSpeed);
        chassisDriveModel->right(motorSpeed);
        delay(10);
    }
    chassisDriveModel->stop();
}

void driveEncoderNS (double speed, double encodervalue){
    double distanceTravelled;
    double startingEnc = enc.get();
    double motorSpeed = speed / 200;
    while ((startingEnc + encodervalue) > fabs(distanceTravelled))
    {
        distanceTravelled = enc.get();
        chassisDriveModel->left(motorSpeed);
        chassisDriveModel->right(motorSpeed);
        delay(10);
    }
}

