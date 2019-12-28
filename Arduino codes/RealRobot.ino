#include <PID_v1.h>
double PIDinput, PIDoutput;
double setpoint = 95.8;

double Kp = 10;  //P = 70, D = 0, I = 0 is the best atm.
double Kd = 0;
double Ki = 0;
PID pid(&PIDinput, &PIDoutput, &setpoint, Kp, Ki, Kd, REVERSE);

void setup()
{
  Serial.begin(115200);
  MotorsSetup();
  GyroAccSetup();
  PIDSetup();
}

void loop() 
{
  PIDinput = GyroAccCompute();
  pid.Compute();
  //Serial.println(PIDoutput);
  MotorsMoveByPID(PIDoutput*0.5);
}
