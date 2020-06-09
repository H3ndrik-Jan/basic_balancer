/*
   Very simple 2 wheel balancing robot example. It is written for an arduino nano.
   Connected to the arduino nano is an L293D motor driver. It's pinout can be derived
   from the defines in this code. Connected to the L293D are two small DC-motors. The
   well-known ones with the 'yellow-gearbox', to be 'exact' ;-).

   An MPU6050 and the MPU6050-light library is used to get an aproximation of the
   angle of the sensor. (https://github.com/rfetick/MPU6050_light)

   This code is just a very simple 'proof of concept' and I wrote it to learn more
   about- and practice with- PID. The code works fine for letting the balancer stay
   upright, but the robot can't be remotely controlled.

   Written by Hendrik-Jan, 06-2020

*/
#include "Wire.h"
#include <MPU6050_light.h>

//Pinout (to L293D)
#define LEFT_ENABLE 5
#define RIGHT_ENABLE 10
#define LEFT_1 3
#define LEFT_2 6
#define RIGHT_1 11
#define RIGHT_2 9

//Optional compensation for mass inbalance
#define COMPENSATION 2

void motor_write(int16_t lspeed, int16_t rspeed);

double Ypos;
double lastYpos = 0;
double Kp = 20, Ki = 1.500, Kd = 900;
int16_t PIDLeft;
int16_t PIDRight;
double Pval;
double Ival;
double Dval;
uint16_t elapsedTime;
long int nowTime;
long int lastTime;

MPU6050 mpu(Wire);

void setup() {
  Wire.begin();
  mpu.begin();
  delay(1000);          //Add some time to have the balancer upright
  mpu.calcGyroOffsets();

  pinMode(LEFT_ENABLE, OUTPUT);
  pinMode(RIGHT_ENABLE, OUTPUT);
  pinMode(LEFT_1, OUTPUT);
  pinMode(LEFT_2, OUTPUT);
  pinMode(RIGHT_1, OUTPUT);
  pinMode(RIGHT_2, OUTPUT);
  digitalWrite(LEFT_1, 0);
  digitalWrite(LEFT_2, 0);
  digitalWrite(RIGHT_1, 0);
  digitalWrite(RIGHT_2, 0);
}

void loop() {
  mpu.update();
  Ypos = mpu.getAngleY();
  Ypos += COMPENSATION; //optional compensation
  Pval = Ypos * Kp;
  Ival = Ival + (Ki * Ypos);
  nowTime = millis();
  elapsedTime = nowTime - lastTime;
  lastTime = nowTime;
  Dval = Kd * ((Ypos - lastYpos) / elapsedTime);
  lastYpos = Ypos;

  PIDLeft = Pval + Ival + Dval;
  PIDRight = PIDLeft;

  // The following lines are to eliminate the deadzone of the motor. You might have to change 
  // the multiplier. You can also compensate for differences between left and right here.
  if (PIDLeft > 0) PIDLeft += ((250 - PIDLeft) * 0.27);
  else if (PIDLeft < 0)PIDLeft -= ((250 + PIDLeft) * 0.27);
  if (PIDRight > 0) PIDRight += ((250 - PIDRight) * 0.27);
  else if (PIDRight < 0)PIDRight -= ((250 + PIDRight) * 0.27);

  //Limit the variable
  if (PIDLeft > 255) PIDLeft = 255;
  else if (PIDLeft < -255) PIDLeft = -255;
  if (PIDRight > 255) PIDRight = 255;
  else if (PIDRight < -255) PIDRight = -255;

  motor_write(PIDLeft, PIDRight);
  delay(6);
}

void motor_write(int16_t lspeed, int16_t rspeed) {

  if (lspeed < 0) { //if lspeed<0, rspeed will be so as well anyways
    digitalWrite(LEFT_1, 1);
    digitalWrite(LEFT_2, 0);
    digitalWrite(RIGHT_1, 1);
    digitalWrite(RIGHT_2, 0);
    analogWrite(LEFT_ENABLE, -lspeed);
    analogWrite(RIGHT_ENABLE, -rspeed);
  }
  else if (lspeed > 0) {
    digitalWrite(LEFT_1, 0);
    digitalWrite(LEFT_2, 1);
    digitalWrite(RIGHT_1, 0);
    digitalWrite(RIGHT_2, 1);
    analogWrite(LEFT_ENABLE, lspeed);
    analogWrite(RIGHT_ENABLE, rspeed);
  }
}
