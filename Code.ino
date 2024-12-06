#include "PID_v1.h"

int Sensor[5] = {0, 0, 0, 0, 0};
int Error;
double Input, Output;
int Speed = 41;//(8;128)
double Setpoint = 0;
double Kp = 75, Ki = 0.8x, Kd = 180;
PID MyPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

void setup() {
  Serial.begin(9600);
  pinMode(10, OUTPUT);
  pinMode(9, OUTPUT);
  Input = 0;
  MyPID.SetSampleTime(1);
  MyPID.SetMode(AUTOMATIC);
  MyPID.SetOutputLimits(-Speed, Speed);
}

void loop() {
  Setpoint = 0;
  ReadSensorValues();
  //  Input = Error;
  //  MyPID.Compute();
  //  MotorControl(Output);
  if (Error == 5) {
    unsigned long Nowtime = millis();
    while (millis() - Nowtime <= 175) {
      analogWrite(10, 255);
      analogWrite(9, 255);
    }
  }
  else {
    Input = Error;
    MyPID.Compute();
    MotorControl(Output);
  }
//  analogWrite(10, 255);
//  analogWrite(9, 255);
}

void MotorControl(int Output) {
  int SpeedRight, SpeedLeft;
  int SpeedCal;
  SpeedCal = 255 / 2;

  if (Output > 1) {
    SpeedLeft = -SpeedCal;
    SpeedRight = Output;
  }
  else if (Output < -1) {
    SpeedRight = -SpeedCal;
    SpeedLeft = -Output;
  }
  else
    SpeedRight = SpeedLeft = 0;

  analogWrite(10, SpeedLeft + SpeedCal + Speed + 46);
  analogWrite(9, SpeedRight + SpeedCal + Speed + 46);
  Serial.print(SpeedLeft + SpeedCal + Speed + 50);
  Serial.println(SpeedRight + SpeedCal + Speed + 50);
}

void ReadSensorValues() {
  Sensor[0] = !digitalRead(12);//Left Sensor
  Sensor[1] = !digitalRead(11);
  Sensor[2] = !digitalRead(7);
  Sensor[3] = !digitalRead(6);
  Sensor[4] = !digitalRead(5);//Right Sensor

  //  Serial.print(Sensor[0]);
  //  Serial.print(Sensor[1]);
  //  Serial.print(Sensor[2]);
  //  Serial.print(Sensor[3]);
  //  Serial.println(Sensor[4]);

  if (Sensor[4] == 1)
    Error = 4;
  else if ((Sensor[3] == 1) && (Sensor[4] == 1))
    Error = 3;
  else if (Sensor[3] == 1)
    Error = 2;
  else if ((Sensor[2] == 1) && (Sensor[3] == 1))
    Error = 1;
  else if (Sensor[2] == 1)
    Error = 0;
  else if ((Sensor[1] == 1) && (Sensor[2] == 1))
    Error = -1;
  else if (Sensor[1] == 1)
    Error = -2;
  else if ((Sensor[0] == 1) && (Sensor[1] == 1))
    Error = -3;
  else if (Sensor[0] == 1)
    Error = -4;
  else if (((Sensor[0] == 1) && (Sensor[1] == 1) && (Sensor[2] == 1)) || ((Sensor[2] == 1) && (Sensor[3] == 1) && (Sensor[4] == 1)) || ((Sensor[1] == 1) && (Sensor[2] == 1) && (Sensor[3] == 1)))
    Error = 5;
  else if ((Sensor[0] == 1) && (Sensor[1] == 1) && (Sensor[2] == 1) && (Sensor[3] == 1) && (Sensor[4] == 1) || ((Sensor[0] == 1) && (Sensor[4] == 1)) || ((Sensor[1] == 1) && (Sensor[3] == 1)))
    Error = 5;
}
