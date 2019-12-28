#include <PID_v1.h>
int outMax = 255;
int outMin = -255;

/*
double setpoint = 90;

double PIDinput, PIDoutput;

double Kp = 100;
double Kd = 0;
double Ki = 0;
PID pid(&PIDinput, &PIDoutput, &setpoint, Kp, Ki, Kd, DIRECT);
*/

void PIDSetup() 
{
  pid.SetMode(AUTOMATIC);
  pid.SetSampleTime(10);
  pid.SetOutputLimits(outMin, outMax);
}

