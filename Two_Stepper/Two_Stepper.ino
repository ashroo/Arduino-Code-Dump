#include <AccelStepper.h>
#define dirPin PB5
#define stepPin PB9
#define dirPin2 PA15
#define stepPin2 PB8
#define motorInterfaceType 1
float p;
float y;
int z;
// Create a new instance of the AccelStepper class:
AccelStepper stepper_yaw = AccelStepper(motorInterfaceType, stepPin, dirPin);
AccelStepper stepper_pitch = AccelStepper(motorInterfaceType, stepPin2, dirPin2);

void setup() 
{
//  stepper.setEnablePin(8);
stepper_yaw.setMaxSpeed(5000);
stepper_yaw.setAcceleration(8000);
stepper_pitch.setMaxSpeed(1000);
stepper_pitch.setAcceleration(8000);
Serial.begin(115200);
p=0.0;
y=0.0;
}

void loop() {
stepper_pitch.moveTo(p);
stepper_pitch.run();
stepper_yaw.moveTo(y);
stepper_yaw.run();
if(Serial.available() != 0)
{
  z=Serial.parseInt();
  Serial.println(z);
  if(z==1)
  {
    while(Serial.available() == 0);
    p=Serial.parseInt();
    p=(p*6400)/360;
    Serial.println(p);
  }
  else if(z==2)
  {
    while(Serial.available() == 0);
    y=Serial.parseInt();
    y=(y*6400)/360;
    //y=y*11;
    Serial.println(y);
  }
}
}
