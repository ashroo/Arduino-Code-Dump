#include <AccelStepper.h>
#define dirPin 2
#define stepPin 3
#define motorInterfaceType 1
float i;
// Create a new instance of the AccelStepper class:
AccelStepper stepper = AccelStepper(motorInterfaceType, stepPin, dirPin);

void setup() 
{
//  stepper.setEnablePin(8);
stepper.setMaxSpeed(5000);
stepper.setAcceleration(8000);
Serial.begin(57600);
i=0;
}

void loop() {
stepper.moveTo(i);
//stepper.runToPosition();
stepper.run();
if(Serial.available() != 0)
{
  Serial.println("ran");
  i=Serial.parseInt();
  i=(i*6400)/360;
//  i=i*11;
//  i=i*6400;
}
//delay(1000);
}
