#ifndef KINEMATICS_H_
#define KINEMATICS_H_



const double PI = 3.14159265358979323846;


const double kBodyLong = 82; //mm  x方向
const double kBodyWidth = 405;   //mm  z方向
const double kBodyHigh = 1131.9101101572;    //mm  y方向

auto inverse(double* leg_in_ground, double* body_in_ground, double* input)->int;

#endif
