#include "main.h"

bool manualControl = true;
bool downHasBall;
bool upHasBall;
pros::Optical opticalMid (16);
int intakeSpeed;
int rollerSpeed;
int rollerLowerSpeed;
Motor intakeL (10);
Motor intakeR (-2);
Motor rollerLower(13);
Motor rollerUpper(11);


ControllerButton intakeUp = controller[ControllerDigital::R1];
ControllerButton shift = controller[ControllerDigital::L2];
ControllerButton shoot = controller[ControllerDigital::L1];
ControllerButton intakeDown = controller[ControllerDigital::R2];
ControllerButton shootSlow = controller[ControllerDigital::X];

void intakeControlv3() {
    if (shift.isPressed() || intakeDown.isPressed()){
        setIntakeSpeed(-200);
    }
    else if (intakeUp.isPressed()){
        if (downHasBall == true) {
            setIntakeSpeed(130);
        }
        else{
            setIntakeSpeed(200);
        }
    }
    else {
        setIntakeSpeed(0);
    }
}

void rollerAutonControl(void*){
    rollerUpper.setBrakeMode(AbstractMotor::brakeMode::brake);
    rollerLower.setBrakeMode(AbstractMotor::brakeMode::hold);
    while (true) {
        if (manualControl == false) {
                if (downHasBall == false && upHasBall == false){
                    rollerSpeed = 60;
                    rollerLowerSpeed = 200;
                }
                if (downHasBall == false && upHasBall == true){
                    rollerSpeed = 0;
                    rollerLowerSpeed = 80;
                }
                if (downHasBall == true && upHasBall == false){
                    rollerSpeed = 60;
                    rollerLowerSpeed = 200;
                }
                if (downHasBall == true && upHasBall == true){
                    rollerSpeed = 0;
                    rollerLowerSpeed = 0;
                }
            rollerUpper.moveVelocity(rollerSpeed);
            rollerLower.moveVelocity(rollerLowerSpeed);   
        }
        delay(10);
    }
}


void shootUntilBlue(double upspeed, double downspeed, int count){
    opticalMid.set_led_pwm(100);
    manualControl = true;
    int blueLowerValue = 100;
    int blueHigherValue = 260;  
    bool canExit = false;
    while (canExit == false)
    {
        rollerUpper.moveVelocity(upspeed);
        rollerLower.moveVelocity(downspeed);
        if (blueLowerValue <= opticalMid.get_hue() && opticalMid.get_hue() <= blueHigherValue){
            canExit = true;
        }
        delay(10);
    }
    pros::lcd::print(1, "yes");
    rollerLower.moveVelocity(0);
    rollerUpper.moveVelocity(0);
    opticalMid.set_led_pwm(0);
}

void autonShoot2(int ballAmount){
    manualControl = true;
    if (ballAmount == 1){
        while (upHasBall == true){
            rollerUpper.moveVelocity(200);
            delay(10);
        }
        delay(50);
        rollerUpper.moveVelocity(0);
        manualControl = false;
    }
    else if (ballAmount == 2){
        while (upHasBall == true ){
            rollerUpper.moveVelocity(190);
            rollerLower.moveVelocity(40);
            delay(10);
        }
        rollerLower.moveVelocity(0);
        rollerUpper.moveVelocity(0);
        delay(50);
        while (upHasBall == false){
            rollerUpper.moveVelocity(100);
            rollerLower.moveVelocity(200);
            delay(10);
        }
        while (upHasBall == true){
            rollerUpper.moveVelocity(190);
            rollerLower.moveVelocity(0);
            delay(10);
        }
        delay(80);
        rollerUpper.moveVelocity(0);
        
        manualControl = false;
    }
    else if (ballAmount == 3){
        rollerUpper.setBrakeMode(AbstractMotor::brakeMode::brake);
        while (upHasBall == true ){
            rollerUpper.moveVelocity(200);
            delay(10);
        }
        delay(50);
        rollerUpper.moveVelocity(0);
        while (upHasBall == false){
            rollerUpper.moveVelocity(70);
            rollerLower.moveVelocity(100);
            delay(10);
        }
        rollerLower.moveVelocity(0);
        rollerUpper.moveVelocity(0);
        delay(50);
        while (upHasBall == true){
            rollerUpper.moveVelocity(200);
            delay(10);
        }
        delay(50);
        rollerUpper.moveVelocity(0);

        while (upHasBall == false){
            rollerUpper.moveVelocity(70);
            rollerLower.moveVelocity(100);
            delay(10);
        }
        rollerLower.moveVelocity(0);
        rollerUpper.moveVelocity(0);
        delay(50);
        
        while (upHasBall == true){
            rollerUpper.moveVelocity(200);
            delay(10);
        }
        delay(50);
        rollerUpper.moveVelocity(0);

        manualControl = false;
    }
    else if (ballAmount == 4){
        while (upHasBall == false){
            rollerUpper.moveVelocity(50);
            rollerLower.moveVelocity(90);
            delay(10);
            pros::lcd::print(1, "going up");
        }
        rollerLower.moveVelocity(0);
        rollerUpper.moveVelocity(0);
        delay(100);
        while (upHasBall == true){
            rollerUpper.moveVelocity(200);
            delay(10);
            pros::lcd::print(1, "yes");
        }
        delay(100);
        rollerUpper.moveVelocity(0);
        
        manualControl = false;
    }

    else if (ballAmount == 5){
        while (upHasBall == false){
            rollerLower.moveVelocity(200);
            rollerUpper.moveVelocity(200);
            delay(10);
        }
        while (upHasBall == true) {
            rollerUpper.moveVelocity(200);
            delay(10);
        }
        delay(100);
        rollerUpper.moveVelocity(0);
        manualControl = false;
    }
}

void rollerControl(){
    rollerUpper.setBrakeMode(AbstractMotor::brakeMode::brake);
    rollerLower.setBrakeMode(AbstractMotor::brakeMode::hold);

        if (shootSlow.isPressed()){
            rollerSpeed = 200;
            rollerLowerSpeed = 100;
        }
        else if (shoot.isPressed()){
            rollerSpeed = 200;
            rollerLowerSpeed = 200;
        }
        else{
            if (shift.isPressed()){
                rollerSpeed = -200;
                rollerLowerSpeed = -200;
            }

            else if (intakeUp.isPressed()){
                if (downHasBall == false && upHasBall == false){
                    rollerSpeed = 70;
                    rollerLowerSpeed = 200;
                }
                if (downHasBall == false && upHasBall == true){
                    rollerSpeed = 0;
                    rollerLowerSpeed = 80;
                }
                if (downHasBall == true && upHasBall == false){
                    rollerSpeed = 70;
                    rollerLowerSpeed = 200;
                }
                if (downHasBall == true && upHasBall == true){
                    rollerSpeed = 0;
                    rollerLowerSpeed = 0;
                }
            }
            else{
                rollerSpeed = 0;
                rollerLowerSpeed = 0;
            }
        }
        rollerLower.moveVelocity(rollerLowerSpeed);
        rollerUpper.moveVelocity(rollerSpeed);
        delay(10);
}

Task autonroller (rollerAutonControl);