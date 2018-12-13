//UNO to MC pins
/*All pins can produce PWM. pins 5 and 6 are not chosen 
because they use the same clock as other features
so generate higher-than-expected duty cycles when duty-cycle is low (or zero).*/

//A is left wheel, B is right wheel.
const int A1IN = 3;
const int A2IN = 9;
const int B2IN = 10;
const int B1IN = 11;

void MotorsSetup()
{
  pinMode(A1IN, OUTPUT);
  pinMode(A2IN, OUTPUT);
  pinMode(B2IN, OUTPUT);
  pinMode(B1IN, OUTPUT);
}

void MotorsMoveByPID(int output)
{
  if (output >= 0)
  {
    MotorsMoveForward(output);
  }
  else
  {
    MotorsMoveBackward(-output);
  }
}

void MotorsMoveForward(int PWMspeed)
{
  //Forward left wheel.
  analogWrite(A1IN, PWMspeed);
  analogWrite(A2IN, 0);
  
  //Reverse right wheel to go forward.
  analogWrite(B1IN, 0);    
  analogWrite(B2IN, PWMspeed); 
}

void MotorsMoveBackward(int PWMspeed)
{
  //Reverse left wheel.
  analogWrite(A1IN, 0);
  analogWrite(A2IN, PWMspeed);
  
  //Forward right wheel to reverse.
  analogWrite(B1IN, PWMspeed);    
  analogWrite(B2IN, 0);
}

void MotorsBrake()
{
  analogWrite(A1IN, 255);
  analogWrite(A2IN, 255);
  
  analogWrite(B1IN, 255);    
  analogWrite(B2IN, 255);
}

void MotorsCoast()
{
  analogWrite(A1IN, 0);
  analogWrite(A2IN, 0);
  
  analogWrite(B1IN, 0);
  analogWrite(B2IN, 0);
}


