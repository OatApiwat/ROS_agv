#ifndef CONFIG_H
#define CONFIG_H

#include <ros.h>
#include <math.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Int8.h>
#include <geometry_msgs/Twist.h>
#include <stdlib.h>

#include <rosserial_arduino/Test.h>
#include <DFRobot_WT61PC.h>
#include <SoftwareSerial.h>
#include <INA226_WE.h>

#define I2C_INA226 0x40

//----------------------------------Config PIN--------------------------------------//
//Pin for Encoder and Motor
const uint8_t pin_EA[2] = {11, 9};
const uint8_t pin_EB[2] = {10, 46};
const uint8_t pin_DIR[2] = {16, 40};
const uint8_t pin_PWM[2] = {15, 39};
//Pin for I2C read INA226
const uint8_t pin_SDA = 41;
const uint8_t pin_SCL = 42;
//Pin for IR
const uint8_t pin_IR1 = 5;
const uint8_t pin_IR2 = 6;
//Pin for IMU
const uint8_t pin_IMU_TX = 12;
const uint8_t pin_IMU_RX = 13;
//Pin for RGB
const uint8_t pin_RGB[3] = {45,48,47};


//-------------------------------Config Parameter-----------------------------------//
//Parameter for Encoder 1024 TPR
long cur_enc[2] = {0,0};
long prv_enc[2] = {0,0};
//Parameter for Kinemetic
const float lx = 0.47;
const float wheel_dia = 0.15;
const int TPR = 1024;
float TPM = TPR / (M_PI*wheel_dia);
//Parameter for CMD_vel and vel target
float vel_tar[2],Cvel_x,Cvel_y,Cvel_z;
//Parameter for actual
float distance[2] = {0,0};
float cur_vel_act[2] = {0,0};
float prv_vel_act[2] = {0,0};
float cur_vel_flt[2] = {0,0};
float vel_x,vel_y,d_XTravel,d_YTravel,x,y;
//Paremeter for Time
unsigned long prv_time0,prv_time1,prv_time2,prv_time3,d_time0,d_time1,d_time2,d_time3;;
//Parameter for PWM
const uint16_t freq = 10000;
const uint8_t pwmChannel[2]={0,1};
const uint8_t resolution = 10;
int16_t pwm_value[2] = {0,0};
//Parameter for PID
float cur_err[2] = {0,0};
float prv_err[2] = {0,0};
float err_I[2] = {0,0};
float I_err[2] = {0,0};
float D_err[2] = {0,0};
const float Kp[2] = {346.79847127127, 346.79847127127};
const float Ki[2] = {2520.76331962240, 2520.76331962240};
const float Kd[2] = {2.6365843374487, 2.6365843374487};
float pwm_be_sat[2]={0,0};
float pwm_sat[2]={0,0};
int8_t dir[2];
//Parameter for IMU sensor
float ofs_IMU, cur_IMU, prv_IMU, cur_th, prv_th, w_th;
int count_IMU,iter_IMU;
//Parameter for Battery
float batt = 0;
//Parameter for curent position
float cur_pose_linear,cur_pose_ang;
//Parameter for emergency system
int8_t allow_run = 1;//0 is not allow run, 1 is allow run

#endif

