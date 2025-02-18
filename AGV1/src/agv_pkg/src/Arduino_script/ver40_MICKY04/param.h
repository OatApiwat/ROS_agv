#include <ros.h>
#include <math.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int32MultiArray.h>
#include <geometry_msgs/Twist.h>
#include <stdlib.h>

#include <rosserial_arduino/Test.h>
#include <DFRobot_WT61PC.h>
#include <SoftwareSerial.h>

// Motor control --> [0]->front-l , [1]->front-r ,[2]->rear-l ,[3]->rear-r
const uint8_t DIR[] = {7, 42, 5, 38};
const uint8_t PWM[] = {6, 39, 4, 37};
const uint8_t EA[] = {11, 35, 12, 34};
const uint8_t EB[] = {10, 36, 13, 33};

// ENCODER
long int prv_enc_count[4];
long int enc[4];

// Time parameter
unsigned long prv_time[2];
float d_time;

//Kinemetic
const float lx = 0.1925;             // m
const float ly = 0.1600;             // m
const float wheel_diameter = 0.126;  // m
const float TPM[] = {400 / (M_PI * wheel_diameter), 360 / (M_PI * wheel_diameter)}; // tick/meter;

// CMD_vel
float vel_t[4], vel_t_prv[4];
uint8_t dir[4];
float Cvel_x, Cvel_y, Cvel_z, Cvel_x0, Cvel_y0, Cvel_z0, Cvel_x1, Cvel_y1, Cvel_z1, vel_x, vel_y, vel_x_act, vel_y_act;
float distance[4];
float vel_act[4];
float vel_flt[4];
float vel_prv[4];

// PID Control
float prv_err[4];
float err_vel[4];
float min_vel;
float mul_vel[4];
float err[4];
float err_I[4];
float I_err[4];
float D_err[4];
// const float Kp[4] = {1151.175493022718, 1151.175493022718, 1151.175493022718, 1151.175493022718};
// const float Ki[4] = {4195.473172624382, 4195.473172624382, 4742.465180481310 , 4742.465180481310};
// const float Kd[4] = {4.281063296298211, 4.281063296298211, 3.495517660151497, 3.495517660151497};
float Kp[4] = {349.60837567743, 346.79847127127, 363.15511898071, 368.32404603444};
float Ki[4] = {2667.17417816143, 2520.76331962240, 2166.22498272451, 2627.31374228841};
float Kd[4] = {2.1782916243661, 2.6365843374487, 2.5632685763864, 2.7318698596063};
float pwm_be_sat[4];
float pwm_sat[4];
int pwm_value[4];

// LED
const uint8_t LED[] = {19, 20, 21}; // R,G,B
//PWM
const int freq = 15000;
const int pwmChannel[4] = {0, 1, 2, 3};
const int resolution = 10;

//Position callback
float dis_linear;
float dis_ang;
float con_linearX;
float con_linearY;
float con_ang;
bool park_state;
bool park_stateZ;
//IMU_sensor
float ofs_IMU, prv_IMU, cur_IMU, th, prv_th, w_th;
int count_IMU, iter_IMU;

// Odom
float d_XTravel, d_YTravel;
float x, y;
//float forOdom[6];