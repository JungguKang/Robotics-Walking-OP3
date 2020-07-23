#ifndef BALANCEFEEDBACK_H
#define BALANCEFEEDBACK_H

#include <cmath>
#include <string>
#include <ros/ros.h>
using namespace std;

enum Euler{
    YAW,
    PITCH,
    ROLL,
};



struct Quaternion {
    double w, x, y, z;
};

struct EulerAngles {
    double roll, pitch, yaw;
};

struct Accelometer {
    double x, y, z;
};


void matMul(double a[][4], double b[][4], double ret[][4]);
void matMulOneD(double a[][4], double b[][1]);
void rotate(string direction, double angle, double ret[][4]);
void translate(string direction, double length, double ret[][4]);
void toAnkleDistance(string side);
Accelometer calVelocity(Accelometer acc);
void saveAngles(string side, string joint_name, double angle);
double balanceFeedback(string direction, double ang, string side);
void ToEulerAngles(Quaternion q);
EulerAngles getAngle();


#endif