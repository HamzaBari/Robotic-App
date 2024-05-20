
#include "dht.h"
#define dht_apin A0 // Analog Pin sensor is connected to the Arduino.
#define dht_apin1 A1

dht DHT1;
dht DHT2;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  delay(500);//Delay to let system boot
  delay(1000);//Wait before accessing Sensor
}

void loop() {
  // put your main code here, to run repeatedly:
  DHT1.read11(dht_apin);
  DHT2.read11(dht_apin1);
    
  Serial.print("temperature 1 = ");
  Serial.print(DHT1.temperature); 
  Serial.println("C  ");
    
  Serial.print("temperature 2 = ");
  Serial.print(DHT2.temperature); 
  Serial.println("C  ");
    
  delay(5000);//Wait 5 seconds before accessing sensor again.

}
