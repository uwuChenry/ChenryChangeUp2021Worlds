#include "main.h"

double convertToRad (double angle) {
    return (angle*M_PI/180);
}
double convertToDeg (double angle) {
    return (angle * 180 / M_PI);
}
double constrainAngle(double angle){
    return atan2(sin(angle / 180.0 * M_PI), cos(angle / 180.0 * M_PI))*180/M_PI;
}
double encToInch (double encvalue){
    return (encvalue/360 * 2.783 * M_PI);
}
double inchToEnc (double inch) {
    return (inch * 360 / 2.783 / M_PI);
}
double mmToEnc (double x) {
    return (inchToEnc(x/25.4));
}
int sgn (double x) {
    if (x > 0) return 1;
    if (x < 0) return -1;
    return 0;
}

