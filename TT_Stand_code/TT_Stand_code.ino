#include "HX711.h" //This library can be obtained here http://librarymanager/All#Avia_HX711

#define TORQUE_DOUT_PIN  PA8
#define TORQUE_SCK_PIN  PB15

#define THRUST_DOUT_PIN  PA7
#define THRUST_SCK_PIN  PA6


HX711 thrust;
HX711 torque;

float torque_cf = 2468.1077;
float thrust_cf = -693.554;

void setup() {
  Serial.begin(9600);

  thrust.begin(THRUST_DOUT_PIN, THRUST_SCK_PIN);
  thrust.set_scale(thrust_cf);
  thrust.tare(); 
  torque.begin(TORQUE_DOUT_PIN, TORQUE_SCK_PIN);
  torque.set_scale(torque_cf);
  torque.tare(); 
  Serial.println("----------Starting reading----------");
}

void loop() 
{
  Serial.print("Thrust=");
  Serial.print(thrust.get_units(), 2);
  Serial.print("g Torque=");
  Serial.print(torque.get_units(), 2);
  Serial.println(" g");
  if(Serial.available())
  {
    char temp = Serial.read();
    if(temp == 't' || temp == 'T')
    {
      thrust.set_scale(thrust_cf);
      thrust.tare();
      torque.set_scale(torque_cf);
      torque.tare();
      Serial.println("Tare done");
    }
  }
}
