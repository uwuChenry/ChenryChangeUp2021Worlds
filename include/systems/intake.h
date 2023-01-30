#pragma once
#include "main.h"
extern void rollerControl();
extern void autonShoot2(int ballAmount);
extern void intakeControlv3();
extern bool manualControl;
extern Motor intakeL;
extern Motor intakeR;
extern Motor rollerLower;
extern Motor rollerUpper;
extern bool downHasBall;
extern bool upHasBall;
extern void shootUntilBlue(double upspeed, double downspeed, int count);