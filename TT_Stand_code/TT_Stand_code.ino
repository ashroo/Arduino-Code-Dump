//Inclusion
#include "HX711.h" //This library can be obtained here http://librarymanager/All#Avia_HX711
#include <Wire.h>
#include <Adafruit_INA219.h>

//defination
#define TORQUE_DOUT_PIN  PA8
#define TORQUE_SCK_PIN  PB15
#define THRUST_DOUT_PIN  PA7
#define THRUST_SCK_PIN  PA6

//declaration
HX711 thrust;
HX711 torque;
Adafruit_INA219 ina219;
float torque_cf = 2468.1077;
float thrust_cf = -693.554;

void setup() {
  Serial.begin(115200);
  while (!Serial) 
  {
      delay(1);
  }
  thrust.begin(THRUST_DOUT_PIN, THRUST_SCK_PIN);
  thrust.set_scale(thrust_cf);
  thrust.tare(); 
  torque.begin(TORQUE_DOUT_PIN, TORQUE_SCK_PIN);
  torque.set_scale(torque_cf);
  torque.tare(); 
  if (! ina219.begin()) 
  {
    Serial.println("Failed to find INA219 chip");
    //while (1) { delay(10); }
  }
  Serial.println("----------Starting reading----------");
}

void loop() 
{
  float power_mW = 0;
  float busvoltage = 0;
  float current_mA = 0;

  busvoltage = ina219.getBusVoltage_V();
  current_mA = ina219.getCurrent_mA();
  power_mW = ina219.getPower_mW();
  
  Serial.print("Thrust=");
  Serial.print(thrust.get_units(), 2);
  Serial.print("g Torque=");
  Serial.print(torque.get_units(), 2);
  Serial.println(" g");

  Serial.print("Bus Voltage:   "); Serial.print(busvoltage); 
  Serial.print("V Current="); Serial.print(current_mA); 
  Serial.print("mA Power="); Serial.print(power_mW); 
  Serial.println("mW");
  Serial.println("");
  
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
