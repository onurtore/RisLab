/*
Onur Berk Töre 
Simulation for a rigid body rotation

Yeditepe University RIS Lab
*/

#include <iostream>
#include <stdlib.h> //abs
#include <math.h> //fmod,sin,cos etc ...
#include <string>
#include <fstream> 
#define SSIGN(a) ((a)>=0?(1.0):(-1.0))

using namespace std;


float old_counter = 0;
float new_counter = 0;
float counter     = 0;

const float PI =  3.1415926535897932384626433832795f;
const float BALL_WIDTH = 8.0;
const float BALL_DEPTH = 9.0;
const float BALL_MASS = 4.0;
const float DELTA_T = 0.001f;
const float COEFFICIENT_FRICTION_ROT_STATIC = 0.20f;
const float COEFFICIENT_FRICTION_ROT        = 0.19f;



void b_calculate_angle(float b_force[3]);
float b_calculateAngleVel(float b_force[3]);
float b_calculateAngleAcc(float b_force[3]);
float b_calculateMoment(float b_force[3]);
float b_calculateInertia();

float a_angle     = 0;



float b_angle     = 0;
float b_angle_vel = 0;
float b_angle_acc = 0;
float b_inertia   = b_calculateInertia();
float b_moment    = 0;

void writeTofile(string filename,string output){

    ofstream out;
    out.open(filename,ios::app);

    out << output;

    out.close();

}


int main(){
    cout << "Enter first cube angle : (With degree 0-360)";
    cin >> a_angle;

    cout <<"Enter second cube angle : (With degree 0-360)";
    cin >> b_angle;

    old_counter = b_angle;
    new_counter = b_angle;

    while( 1/*abs(a_angle - b_angle) > 10 Threshold*/ )
    {
        counter++;
        //string output = to_string(b_angle);
        new_counter =  b_angle;
        
        if(abs ( new_counter - old_counter) > 1){
            old_counter = new_counter;
            cout << counter << '\t' << b_angle << '\n' ;
        }

        float cal_b_angle_vel = (a_angle - b_angle ) / 1000;
        float cal_b_angle_acc = cal_b_angle_vel / 1000;
        float cal_b_moment    = b_inertia * cal_b_angle_acc;

        if(b_angle == 0){
            b_angle = 0.001;
        }

        float b_force[3] = {0};

        b_force[0] = ( (cal_b_moment) / sin(b_angle) * BALL_WIDTH * 0.5);
        b_force[1] = 0;
        b_force[2] = ( (cal_b_moment) / cos(b_angle) * BALL_WIDTH * 0.5);
        
        b_force[0] *= 600;
        b_force[1] *= 600;
        b_force[2] *= 600;

       // output += '\t' +  to_string(b_force[0]) + '\t' + to_string(b_force[2]);
        
        b_calculate_angle(b_force);
        
        //output += '\t' + to_string(b_angle) + '\n';
        //writeTofile("cool_rotation.txt",output);//CPU Consuming
    }
}


float b_calculateInertia(){

    float width = BALL_WIDTH; //Get from haptic board game
    float depth = BALL_DEPTH;
    float mass =  BALL_MASS;

    return ( mass / 12 ) * ( width * width + depth * depth);

}


void b_calculate_angle(float b_force[3]){

    float angleV = b_calculateAngleVel(b_force);
    b_angle = b_angle + (DELTA_T * angleV) + b_calculateAngleAcc(b_force) *0.5 * pow(DELTA_T,2);

}


float b_calculateAngleVel(float b_force[3]){


    b_angle_vel = b_angle_vel + (DELTA_T * (b_calculateAngleAcc(b_force)));
    return b_angle_vel;

}


float b_calculateAngleAcc(float b_force[3]){

    b_angle_acc = b_calculateMoment(b_force) / b_inertia;
    return b_angle_acc;
}

float b_calculateMoment(float b_force[3]){

    //Tork = moment 
    float tork = 0;
    float tork_resistance = 0;
    tork = b_force[2] * BALL_WIDTH * 0.5 * cos(b_angle) + b_force[0] * BALL_WIDTH * 0.5 * sin(b_angle);

    //Rotational friction
    if(abs(b_angle_vel) == 0.0f){
        tork_resistance = b_inertia * COEFFICIENT_FRICTION_ROT_STATIC * SSIGN(tork);
        tork -= tork_resistance;
    }
    
    else if (abs(b_angle_vel) > 0.0f){
        tork_resistance = b_inertia * COEFFICIENT_FRICTION_ROT * SSIGN(b_angle_vel);
        
        tork -= tork_resistance;
    }

    return tork;
}
