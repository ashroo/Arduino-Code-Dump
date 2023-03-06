#include <PWM.h>

const int b_low=7,b_high=8,pot=A0;
const int pot_setter=A1;

unsigned long prms,crms;
int val,val_set;

// the setup routine runs once when you press reset:
void setup() 
{
//  InitTimersSafe();
//  bool check=SetPinFrequencySafe(3, 50);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(b_low, INPUT_PULLUP);
  pinMode(b_high, INPUT_PULLUP);
  pinMode(pot,INPUT);
  pinMode(pot_setter,INPUT);
  pinMode(3, OUTPUT);
  Serial.begin(9600);
  digitalWrite(3,LOW);
//  Serial.println(check);
 crms=micros();
 prms=crms; 
}

// the loop routine runs over and over again forever:
void loop() 
{
//  button_low_state = digitalRead(b_low);
//  button_high_state = digitalRead(b_high);
crms=micros();
if(crms-prms >= 20000)
{
  prms=crms;
  if (digitalRead(b_low) == 1)
  {
//    digitalWrite(LED_BUILTIN, HIGH); 
//    pwmWrite(3, 13);
    digitalWrite(3,HIGH);
    delayMicroseconds(1000-0.15);
    digitalWrite(3,LOW);
  }
  else if (digitalRead(b_high) == 1)
  {
//    digitalWrite(LED_BUILTIN, HIGH);
//    pwmWrite(3, 25.5);
    digitalWrite(3,HIGH);
    delayMicroseconds(2000-0.15);
    digitalWrite(3,LOW);
  }
  else
  {
    int pot_set= analogRead(pot_setter);
    val_set= map(pot_set,0,1023,1000,2000);
    int potIn = analogRead(pot);
    val= (map(potIn, 102, 923, val_set, 1000));
//    Serial.print(pot_set);
//    Serial.print("----");
//    Serial.print(val_set);
//    Serial.print("----");
//    Serial.println(val);
//    pwmWrite(3, val/100);
    digitalWrite(3,HIGH);
    delayMicroseconds(val-0.15);
    digitalWrite(3,LOW);
//    digitalWrite(LED_BUILTIN, LOW);
  }
}
else if(crms-prms <0)
{
  prms=0;
}

}
