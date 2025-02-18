#include "param.h"
#include "FINDSIGN.h"
using rosserial_arduino::Test;

SoftwareSerial mySerial(17, 18);
DFRobot_WT61PC sensor(&mySerial);

ros::NodeHandle nh;

// Pub data for odom and tf
std_msgs::Float32MultiArray forOdom_msg;
ros::Publisher FORODOM_("forOdom_data", &forOdom_msg);

// Pub encoder
std_msgs::Int32MultiArray enc_msg;
ros::Publisher ENC_("enc_data", &enc_msg);

// Pub Target
std_msgs::Float32MultiArray target_msg;
ros::Publisher TARGET_("target_data", &target_msg);


void CMD_vel_callback(const geometry_msgs::Twist &CVel_msg) {
  Cvel_x0 = CVel_msg.linear.x;
  Cvel_y0 = CVel_msg.linear.y;
  Cvel_z0 = CVel_msg.angular.z;
  calculate_vel_t();
}

void Position_callback(const std_msgs::Float32MultiArray &Pose_msg) {
  dis_linear = Pose_msg.data[3];
  dis_ang = Pose_msg.data[4] * 180 / M_PI;
  if (abs(dis_linear) < 0.1) {
    con_linearX = 0.08;
    con_linearY = 0.08;
    park_state = true;
  } else if (abs(dis_linear) < 0.3) {
    con_linearX = 0.1;
    con_linearY = 0.1;
    park_state = true;
  } else {
    con_linearX = 0.25;
    con_linearY = 0.25;
    park_state = false;
  }
  if (abs(dis_ang) < 5) {
    con_ang = 0.08;
    park_stateZ = true;
  }else if (abs(dis_ang) < 45)
  {
    con_ang = 0.1;
    park_stateZ = true;
  }else 
  {
    con_ang = 0.25;
    park_stateZ = false;
  }
}



ros::Subscriber<geometry_msgs::Twist> SUB_CMDvel("/cmd_vel", &CMD_vel_callback);
ros::Subscriber<std_msgs::Float32MultiArray> SUB_pose("position", &Position_callback);
// Service Clear encoder result
void srv_clear_data(const Test::Request &req, Test::Response &res);

// Service
ros::ServiceServer<Test::Request, Test::Response> clear_data("clear_data", &srv_clear_data);


//SETUP FUNCTION
void gpio_define() {
  for (int i = 0; i < 4; i++) {
    pinMode(EA[i], INPUT_PULLUP);
    pinMode(EB[i], INPUT_PULLUP);
    pinMode(DIR[i], OUTPUT);
    pinMode(PWM[i], OUTPUT);
    ledcSetup(pwmChannel[i], freq, resolution);
    ledcAttachPin(PWM[i], pwmChannel[i]);
    pwm_value[i] = 0;
    pwm_be_sat[i] = 0;
    vel_t_prv[i] = 0;
    vel_t[i] = 0;
    enc[i] = 0;
  }
  for (int i = 0; i < 3; i++) {
    pinMode(LED[i], OUTPUT);
  }

  attachInterrupt(digitalPinToInterrupt(EA[0]), enc_count0, CHANGE);
  attachInterrupt(digitalPinToInterrupt(EA[1]), enc_count1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(EA[2]), enc_count2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(EA[3]), enc_count3, CHANGE);
  dis_linear = 100;
  dis_ang = 100;
  con_linearX = 0.25;
  con_linearY = 0.25;
  con_ang = 0.25;
  park_state = false;
  park_stateZ = false;
}
void IMU_define() {
  //Use software serial port mySerial as communication seiral port
  mySerial.begin(9600);
  sensor.modifyFrequency(FREQUENCY_100HZ);
  bool start_IMU = true;
  while (start_IMU) {
    if (sensor.available()) {
      ofs_IMU = sensor.Angle.Z;
      prv_IMU = sensor.Angle.Z;
      start_IMU = false;
    }
    if (iter_IMU >= 1000) {
      start_IMU = false;
    }
    iter_IMU++;
    delay(5);
  }
  count_IMU = 0;
}
//CONTROL FUNCTION
void calculate_vel_t() {
  if (park_state) {
    Cvel_x1 = ((Cvel_x0 + 0.25) * (2 * con_linearX) / (2 * 0.25)) - con_linearX;
    Cvel_y1 = ((Cvel_y0 + 0.25) * (2 * con_linearY) / (2 * 0.25)) - con_linearY;
  } else {
    Cvel_x1 = Cvel_x0;
    Cvel_y1 = Cvel_y0;
  }
  if (park_state && park_stateZ) {
    Cvel_z1 = ((Cvel_z0 + 0.25) * (2 * con_ang) / (2 * 0.25)) - con_ang;
  } else {
    Cvel_z1 = Cvel_z0;
  }
  Cvel_x = constrain(Cvel_x1, -0.25, 0.25);
  Cvel_y = constrain(Cvel_y1, -0.25, 0.25);
  Cvel_z = constrain(Cvel_z1, -0.25, 0.25);
  vel_t[0] = (Cvel_x - Cvel_y - (lx + ly) * Cvel_z);
  vel_t[1] = (Cvel_x + Cvel_y + (lx + ly) * Cvel_z);
  vel_t[2] = (Cvel_x + Cvel_y - (lx + ly) * Cvel_z);
  vel_t[3] = (Cvel_x - Cvel_y + (lx + ly) * Cvel_z);
}
void calculate_vel_act() {
  for (int i = 0; i < 2; i++) {  // for encoder type 400tpr
    distance[i] = float(enc[i] - prv_enc_count[i]) / TPM[0];
    vel_act[i] = 1000 * (distance[i] / d_time);
    vel_flt[i] = 0.9 * vel_flt[i] + 0.1 * vel_act[i];
    if (vel_act[i] == 0) {
      vel_flt[i] = 0;
    }
    prv_enc_count[i] = enc[i];
  }
  for (int i = 2; i < 4; i++) {  // for encoder type 360tpr
    distance[i] = float(enc[i] - prv_enc_count[i]) / TPM[1];
    vel_act[i] = 1000 * (distance[i] / d_time);
    vel_flt[i] = 0.9 * vel_flt[i] + 0.1 * vel_act[i];
    if (vel_act[i] == 0) {
      vel_flt[i] = 0;
    }
    prv_enc_count[i] = enc[i];
  }
  // forward kinematic
  vel_x = (vel_flt[0] + vel_flt[1] + vel_flt[2] + vel_flt[3]) / 4;
  vel_y = (-vel_flt[0] + vel_flt[1] + vel_flt[2] - vel_flt[3]) / 4;
  d_XTravel = (distance[0] + distance[1] + distance[2] + distance[3]) / 4;
  d_YTravel = (-distance[0] + distance[1] + distance[2] - distance[3]) / 4;
  x += d_XTravel * cos(th * M_PI / 180) - d_YTravel * sin(th * M_PI / 180);
  y += d_YTravel * cos(th * M_PI / 180) + d_XTravel * sin(th * M_PI / 180);
}
void pid_control() {
  for (int i = 0; i < 4; i++) {
    err[i] = vel_t[i] - vel_flt[i];
    D_err[i] = 1000 * (err[i] - prv_err[i]) / d_time;
    prv_err[i] = err[i];
    //Anti Windup-error
    if ((pwm_be_sat[i] > pwm_sat[i]) && (findSign(err[i]) == findSign(pwm_be_sat[i]))) {
      err_I[i] = 0;
    } else {
      err_I[i] = err[i];
    }

    I_err[i] = I_err[i] + err_I[i] * d_time / 1000;

    pwm_be_sat[i] = round(4*(Kp[i] * err[i] + Ki[i] * I_err[i] + Kd[i] * D_err[i]));
    //Saturation
    if (pwm_be_sat[i] >= 1023) {
      pwm_sat[i] = 1023;
    } else if (pwm_be_sat[i] <= -1023) {
      pwm_sat[i] = -1023;
    } else {
      pwm_sat[i] = round(pwm_be_sat[i]);
    }
    pwm_value[i] = abs(pwm_sat[i]);  //abs(pwm_sat[i])
    //Set Direction
    if (vel_t[i] > 0) {
      dir[i] = 0;
      if (pwm_sat[i] < 0) {
        pwm_value[i] = 0;
      }
    } else if (vel_t[i] < 0) {
      dir[i] = 1;
      if (pwm_sat[i] > 0) {
        pwm_value[i] = 0;
      }
    } else {
      pwm_value[i] = 0;
    }
  }
}
void agv_run() {
  for (int i = 0; i < 4; i++) {
    digitalWrite(DIR[i], dir[i]);
    ledcWrite(pwmChannel[i], pwm_value[i]);
  }
}
void clear() {
  //  Clear enc_data
  if (vel_act[0] == 0 && vel_act[1] == 0 && vel_act[2] == 0 && vel_act[3] == 0 && vel_t[0] == 0 && vel_t[1] == 0 && vel_t[2] == 0 && vel_t[3] == 0) {
    for (int i = 0; i < 4; i++) {
      prv_enc_count[i] = 0;
      enc[i] = 0;
      pwm_value[i] = 0;
      vel_flt[i] = 0;
    }
  }
}
void IMU_DFrobot() {
  if (sensor.available()) {
    cur_IMU = sensor.Angle.Z;
    if (prv_IMU - cur_IMU > 300) {
      count_IMU++;
    } else if (cur_IMU - prv_IMU > 300) {
      count_IMU--;
    }
    prv_IMU = cur_IMU;
    th = 360 * count_IMU + cur_IMU - ofs_IMU;
    w_th = 1000 * (th - prv_th) / d_time;
    prv_th = th;
  }
}
//ROS FUNCTION
void forOdom_publish() {
  float forOdom[6] = { x, y, th, vel_x, vel_y, w_th };
  forOdom_msg.data = forOdom;
  forOdom_msg.data_length = 6;
  FORODOM_.publish(&forOdom_msg);
}
void enc_publish() {
  int enc_p[4] = { enc[0], enc[1], enc[2], enc[3] };
  enc_msg.data = enc_p;
  enc_msg.data_length = 4;
  ENC_.publish(&enc_msg);
}
void target_publish() {
  float target_p[9] = { vel_t[0], vel_t[1], vel_t[2], vel_t[3], pwm_value[0], pwm_value[1], pwm_value[2], pwm_value[3], int(park_state) };
  target_msg.data = target_p;
  target_msg.data_length = 9;
  TARGET_.publish(&target_msg);
}
void srv_clear_data(const Test::Request &req, Test::Response &res) {
  bool start_IMU = true;
  while (start_IMU) {
    if (sensor.available()) {
      ofs_IMU = sensor.Angle.Z;
      prv_IMU = sensor.Angle.Z;
      start_IMU = false;
    }
    if (iter_IMU >= 1000) {
      start_IMU = false;
    }
    iter_IMU++;
    delay(5);
  }
  count_IMU = 0;
  x = 0;
  y = 0;
  for (int i = 0; i < 4; i++) {
    pwm_value[i] = 0;
    enc[i] = 0;
    prv_enc_count[i] = 0;
    pwm_be_sat[i] = 0;
    vel_t_prv[i] = 0;
    vel_t[i] = 0;
  }
  res.output = "Data has been cleared ";
}
//LED
void led_show(char color) {
  switch (color) {
    case 'r':
      analogWrite(LED[0], 0);
      analogWrite(LED[1], 255);
      analogWrite(LED[2], 255);
      break;
    case 'g':
      analogWrite(LED[0], 255);
      analogWrite(LED[1], 0);
      analogWrite(LED[2], 255);
      break;
    case 'b':
      analogWrite(LED[0], 255);
      analogWrite(LED[1], 255);
      analogWrite(LED[2], 0);
      break;
    case 'y':
      analogWrite(LED[0], 0);
      analogWrite(LED[1], 0);
      analogWrite(LED[2], 255);
      break;
    default:
      analogWrite(LED[0], 0);
      analogWrite(LED[1], 0);
      analogWrite(LED[2], 0);
      break;
  }
}
void status_() {
  if (Cvel_x > 0) {
    led_show('g');
  } else if (Cvel_x < 0) {
    led_show('r');
  } else if (Cvel_x == 0 && Cvel_y == 0 && abs(Cvel_z) > 0) {
    led_show('y');
  } else if (Cvel_x == 0 && Cvel_y == 0 && Cvel_z == 0) {
    led_show('b');
  }
}


void setup() {
  nh.initNode();
  nh.getHardware()->setBaud(115200);

  nh.advertise(FORODOM_);
  nh.advertise(ENC_);
  nh.advertise(TARGET_);

  nh.advertiseService(clear_data);
  nh.subscribe(SUB_CMDvel);
  nh.subscribe(SUB_pose);
  gpio_define();
  IMU_define();
}

void loop() {
  d_time = millis() - prv_time[0];
  if (d_time >= 40) {
    prv_time[0] = millis();
    calculate_vel_act();
    pid_control();
    agv_run();
    status_();
    clear();
    IMU_DFrobot();
    enc_publish();
    target_publish();
  }
  forOdom_publish();
  nh.spinOnce();
}


void enc_count0() {
  if (digitalRead(EB[0]) == 0) {
    if (digitalRead(EA[0]) == 0) {
      enc[0]++;
    } else {
      enc[0]--;
    }
  }
}
void enc_count1() {
  if (digitalRead(EB[1]) == 0) {
    if (digitalRead(EA[1]) == 0) {
      enc[1]--;
    } else {
      enc[1]++;
    }
  }
}
void enc_count2() {
  if (digitalRead(EB[2]) == 0) {
    if (digitalRead(EA[2]) == 0) {
      enc[2]++;
    } else {
      enc[2]--;
    }
  }
}
void enc_count3() {
  if (digitalRead(EB[3]) == 0) {
    if (digitalRead(EA[3]) == 0) {
      enc[3]--;
    } else {
      enc[3]++;
    }
  }
}