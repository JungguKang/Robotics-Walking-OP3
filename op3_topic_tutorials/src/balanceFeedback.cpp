#include "balanceFeedback.hpp"
#include <iostream>

// globals for linear velocity
double abs_vel_x;
double abs_vel_y;
double abs_vel_z;
ros::Time curr_time;
ros::Time prev_time;

double dist_x;
double dist_y;

// Angles for balance feedback
EulerAngles globalAngle;
EulerAngles hipAngle;
EulerAngles kneeAngle;
EulerAngles ankleAngle;


double balanceFeedback(string direction, double ang, string side){

    double result, cv, cd, vel, dist;
    //cd = 0.1;
    //cv = 0.01;
    //cd = 0.15;
    //cd = 0.18;
    //cd = 0.26;
    //cd = 0.07;
    cd = 0.1;
    cv = 0.03;

    if (direction == "x"){
        
        if(side == "l"){    // 6/30 modified by 경험
            dist = dist_x;
            vel = abs_vel_x;
        }
        else{
            dist = dist_x;
            vel = abs_vel_x;
        }
        /*
        cout << "x feedback" << endl;
        cout << "distance: " << dist_x << endl;
        //cout << "distance: " << dist << endl;
        //cout << "velocity: " << vel << endl;
        cout << "ang0    : " << ang << endl;
        cout << "result  : " << ang + cd * dist + cv * vel << endl;
        //cout << endl << endl;
        */
    }
    else if(direction == "y"){
        
        if(side == "l"){    // 6/30 modified by 경험
            dist = -dist_y;
            vel = -abs_vel_y;
        }
        else{
            dist = -dist_y;
            vel = -abs_vel_y;
        }
        /*
        cout << "y feedback" << endl;
        cout << "distance: " << dist_y << endl;
        //cout << "velocity: " << vel << endl;
        cout << "ang0    : " << ang << endl;
        cout << "result  : " << ang + cd * dist + cv * vel << endl;
        cout << endl << endl;
        */
    }

    result = ang + cd * dist + cv * vel;

    //cout << "distance: " << dist << endl;
    //cout << "velocity: " << vel << endl;
    //cout << "ang0    : " << ang << endl;
    //cout << "result  : " << result << endl;
    //cout << endl << endl;
    return result;
    //return ang;
}

//void toAnkleDistance(string side, EulerAngles hipAngle, EulerAngles kneeAngle, EulerAngles globalAngle, double* ret){
void toAnkleDistance(string side){
    // side == 'l' left
    // side == 'r' right
    //(cm)

    // Y 축에 대한 distance 구할 때 문제가 있음 -> 다시 할것
    
    double upperLegLength = 11.0;
    double lowerLegLength = 11.0;
    double hipToCOM_z = 9.0; //2.9;
    double hipToCOM_y = 5.0;

    //cout << "hip roll: " << hipAngle.roll << endl;

    if(side == "r"){ // right
        //hipToCOM = -hipToCOM;
        ankleAngle.roll = -ankleAngle.roll;
        kneeAngle.pitch = -kneeAngle.pitch;
        //kneeAngle.yaw = -kneeAngle.yaw;
        //kneeAngle.roll = -kneeAngle.roll;

        hipAngle.pitch = -hipAngle.pitch;
        hipAngle.yaw = -hipAngle.yaw;
        //hipAngle.roll = -hipAngle.roll;                   //****6/30 modified ->왜 이게 맞는지는 모름

    }
    else{
        //hipAngle.roll = -hipAngle.roll;                   //****6/30 modified ->왜 이게 맞는지는 모름
        ankleAngle.roll = -ankleAngle.roll;
        kneeAngle.pitch = -kneeAngle.pitch;
        hipAngle.pitch = -hipAngle.pitch;
        hipToCOM_y = -5.0;
    }
    /*
    cout << "hip yaw : " << hipAngle.yaw <<endl;
    cout << "hip pitch : " << hipAngle.pitch <<endl;
    cout << "hip roll : " << hipAngle.roll <<endl<<endl;
    cout << "knee pitch : " << kneeAngle.pitch <<endl<< endl;
    cout << "global yaw : " << globalAngle.yaw <<endl;
    cout << "global pitch : " << globalAngle.pitch <<endl;
    cout << "global roll : " << globalAngle.roll <<endl;
    */
    double point[4][1] = {
        {0},
        {0},
        {0},
        {1}
    };


    double target[4][4] = {
        {1.0, 0.0, 0.0, 0.0},
        {0.0, 1.0, 0.0, 0.0},
        {0.0, 0.0, 1.0, 0.0},
        {0.0, 0.0, 0.0, 1.0}
    };


    //rotate("yaw", globalAngle.yaw, target);
    //rotate("pitch", globalAngle.pitch, target);
    //rotate("roll", globalAngle.roll, target);


/*
    rotate("yaw", hipAngle.yaw, target);
    rotate("pitch", hipAngle.pitch, target);
    rotate("roll", hipAngle.roll, target);

    translate("z", -upperLegLength, target);

    //rotate("yaw", kneeAngle.yaw, target);
    rotate("pitch", kneeAngle.pitch, target);
    //rotate("roll", kneeAngle.roll, target);

    translate("z", -lowerLegLength, target);

    matMulOneD(target, point);

    dist_x = -point[0][0];          //************ 6/30 modified ************
    dist_y = -point[1][0];          //distance should be ankle to hip, not hip to ankle
*/
    rotate("pitch", ankleAngle.pitch, target);
    rotate("roll", ankleAngle.roll, target);

    translate("z", lowerLegLength, target);

    rotate("pitch", kneeAngle.pitch, target);

    translate("z", upperLegLength, target);
    
    rotate("yaw", hipAngle.yaw, target);
    rotate("pitch", hipAngle.pitch, target);
    rotate("roll", hipAngle.roll, target);

    translate("z", hipToCOM_z, target);
    translate("y", hipToCOM_y, target);
    //translate("x", -1, target);

    //rotate("yaw", kneeAngle.yaw, target);
    
    //rotate("roll", kneeAngle.roll, target);

    matMulOneD(target, point);

    dist_x = point[0][0];          //************ 6/30 modified ************
    dist_y = point[1][0];          //distance should be ankle to hip, not hip to ankle
    /*
    for(int i = 0; i < 3; i ++){
        distance[i] = point[i][0];
        //ret[i] = point[i][0];
    }*/
}

Accelometer calVelocity(Accelometer acc){
    double gravity[4][1] = {
        {0},
        {0},
        {9.80665},
        {1}
    };


    double target[4][4] = {
        {1.0, 0.0, 0.0, 0.0},
        {0.0, 1.0, 0.0, 0.0},
        {0.0, 0.0, 1.0, 0.0},
        {0.0, 0.0, 0.0, 1.0}
    };
    // if Yaw first -> z axis(gravity) will not rotate
    rotate("roll", -globalAngle.roll, target);
    rotate("pitch", -globalAngle.pitch, target);
    rotate("yaw", -globalAngle.yaw, target);

    matMulOneD(target, gravity);
    /*
    cout << "original acc" << endl;
    cout << "x: " << acc.x << endl;
    cout << "y: " << acc.y << endl;
    cout << "z: " << acc.z << endl;


    cout << "gravity acc" << endl;
    cout << "x: " << gravity[0][0] << endl;
    cout << "y: " << gravity[0][1] << endl;
    cout << "z: " << gravity[0][2] << endl;
    */
    acc.x = acc.x - gravity[0][0];
    acc.y = acc.y - gravity[1][0];
    acc.z = acc.z - gravity[2][0];
    
    //cout << "calculated acc" << endl;
    //cout << "x: " << acc.x << endl;
    //cout << "y: " << acc.y << endl;
    //cout << "z: " << acc.z << endl << endl;
    
    curr_time = ros::Time::now();

    abs_vel_x = abs_vel_x + (curr_time.toSec() - prev_time.toSec()) * acc.x;
    abs_vel_y = abs_vel_y + (curr_time.toSec() - prev_time.toSec()) * acc.y;
    abs_vel_z = abs_vel_z + (curr_time.toSec() - prev_time.toSec()) * acc.z;

    //cout << "x_vel: " << abs_vel_x << endl << "y_vel: " << abs_vel_y << endl;
    //cout << endl << endl;
    
    prev_time = curr_time;

    return acc;
}

void saveAngles(string side, string joint_name, double angle){
    if(side == "l"){

        if(joint_name == "l_hip_roll"){     // swing hip lat
            hipAngle.roll = angle;
        }
        else if(joint_name == "l_hip_yaw"){     // swing hip lat
            hipAngle.yaw = angle;
        }
        else if(joint_name == "l_hip_pitch"){     // swing hip lat
            hipAngle.pitch = angle;
        }
        else if(joint_name == "l_knee"){         // swing knee
            kneeAngle.pitch = angle;
        }
        else if(joint_name == "l_ank_pitch"){         // swing knee
            ankleAngle.pitch = angle;
        }
        else if(joint_name == "l_ank_roll"){         // swing knee
            ankleAngle.roll = angle;
        }
    }
    else{

        if(joint_name == "r_hip_roll"){     // swing hip lat
            hipAngle.roll = angle;
        }
        else if(joint_name == "r_hip_yaw"){     // swing hip lat
            hipAngle.yaw = angle;
        }
        else if(joint_name == "r_hip_pitch"){     // swing hip lat
            hipAngle.pitch = angle;
        }
        else if(joint_name == "r_knee"){         // swing knee
            kneeAngle.pitch = angle;
        }
        else if(joint_name == "r_ank_pitch"){         // swing knee
            ankleAngle.pitch = angle;
        }
        else if(joint_name == "r_ank_roll"){         // swing knee
            ankleAngle.roll = angle;
        }
    }
}

// a = a * b
void matMul(double a[][4], double b[][4]){
    double ret[4][4] =  {
        {0.0, 0.0, 0.0, 0.0},
        {0.0, 0.0, 0.0, 0.0},
        {0.0, 0.0, 0.0, 0.0},
        {0.0, 0.0, 0.0, 0.0}
    };

    for(int i = 0; i < 4; ++i)
        for(int j = 0; j < 4; ++j)
            for(int k = 0; k < 4; ++k){
                ret[i][j] += a[i][k] * b[k][j];
            }
    

    copy(&ret[0][0], &ret[0][0] + 4 * 4, &a[0][0]);
    
}

// b = a * b (b: 4*1)
void matMulOneD(double a[][4], double b[][1]){
    double ret[4][1] =  {
        {0.0},
        {0.0},
        {0.0},
        {0.0}
    };

    for(int i = 0; i < 4; ++i)
        for(int j = 0; j < 1; ++j)
            for(int k = 0; k < 4; ++k){
                ret[i][j] += a[i][k] * b[k][j];
            }
    copy(&ret[0][0], &ret[0][0] + 4 * 1, &b[0][0]);
    
}

void rotate(string direction, double angle, double target[][4]){
    // x (roll)
    // y (pitch)
    // z (yaw)

    if(direction == "roll" || direction == "x"){
        double rot[4][4] = {
            {1,          0,           0, 0},
            {0, cos(angle), -sin(angle), 0},
            {0, sin(angle),  cos(angle), 0},
            {0,          0,           0, 1}
        };

        matMul(target, rot);
    }

    else if(direction == "pitch" || direction == "y"){
        double rot[4][4] = {
            { cos(angle),  0, sin(angle), 0},
            {          0,  1,          0, 0},
            {-sin(angle),  0, cos(angle), 0},
            {          0,  0,          0, 1}
        };

        matMul(target, rot);
    }

    else if(direction == "yaw" || direction == "z"){
        double rot[4][4] = {
            {cos(angle), -sin(angle), 0, 0},
            {sin(angle),  cos(angle), 0, 0},
            {         0,           0, 1, 0},
            {         0,           0, 0, 1}
        };

        matMul(target, rot);
    }
}

void translate(string direction, double length, double target[][4]){
    double trl[4][4] = {
        {1.0, 0.0, 0.0, 0.0},
        {0.0, 1.0, 0.0, 0.0},
        {0.0, 0.0, 1.0, 0.0},
        {0.0, 0.0, 0.0, 1.0}
    };

    if(direction == "x"){
        trl[0][3] = length;
    }
    else if(direction == "y"){
        trl[1][3] = length;
    }
    else if(direction == "z"){
        trl[2][3] = length;
    }

    matMul(target, trl);
}

void ToEulerAngles(Quaternion q) {
    EulerAngles angles;

    // roll (x-axis rotation)
    double sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
    double cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
    angles.roll = std::atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = 2 * (q.w * q.y - q.z * q.x);
    if (std::abs(sinp) >= 1)
        angles.pitch = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        angles.pitch = std::asin(sinp);

    // yaw (z-axis rotation)
    double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    angles.yaw = std::atan2(siny_cosp, cosy_cosp);

    globalAngle = angles;
}

EulerAngles getAngle(){
    return globalAngle;
}