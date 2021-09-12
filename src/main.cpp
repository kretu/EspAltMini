#include <Arduino.h>
#include <EEPROM.h>
#include <Wire.h>
#include <Adafruit_BMP280.h>
#include <ESP8266WebServer.h>
Adafruit_BMP280 bmp; // use I2C interface
Adafruit_Sensor *bmp_temp = bmp.getTemperatureSensor();
Adafruit_Sensor *bmp_pressure = bmp.getPressureSensor();


float startP;
float maxAlt;
float Alt;
int x=0;
char ssid[32];
unsigned long currentMillis;
unsigned long oldMillis;
unsigned long oldMillisSerial;

void setup() {
  float maxAltOld;
  EEPROM.begin(512);
  Serial.begin(9600);
  Serial.println("BMP280 Sensor event test");

  //if (!bmp.begin(BMP280_ADDRESS_ALT, BMP280_CHIPID)) {
  if (!bmp.begin(0x76)) {                           /* Example 0x76 I2C Adress of BMP280 */
    Serial.println("Could not find a valid BMP280 sensor, check wiring or "
                      "try a different address!");
    while (1) delay(10);
  }

  /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */

  bmp_temp->printSensorDetails();
  sensors_event_t pressure_event;
  bmp_pressure->getEvent(&pressure_event);
startP = pressure_event.pressure;
EEPROM.get(0, maxAltOld);
sprintf(ssid,"Wysokosc z pamieci %.2f m",maxAltOld);
WiFi.softAP(ssid);

}



void loop() {

      while(x<1){
        currentMillis = millis();
        if(currentMillis > oldMillis + 100){
          Alt = bmp.readAltitude(startP);
          if(Alt > maxAlt) {
            maxAlt = Alt;
          }
          if(maxAlt>Alt+10) {
            Serial.print("Osiagnieta wysokosc = ");
            Serial.print(maxAlt);
            Serial.println(" m ");
            EEPROM.put(0,maxAlt);
            EEPROM.commit();
            sprintf(ssid, "Wysokosc z pomiaru %.2f",maxAlt);
            WiFi.softAP(ssid);
            x++;
          }
          oldMillis = currentMillis;
        }
        if(currentMillis > oldMillisSerial + 1000){
          Serial.print("Approx altitude = ");
          Serial.print(Alt);
          Serial.println(" m ");
          Serial.print("Max altitude = ");
          Serial.print(maxAlt);
          Serial.println(" m ");
          Serial.println(ssid);
          oldMillisSerial = currentMillis;
        }
      } 
    currentMillis = millis();  
    if(currentMillis > oldMillisSerial + 1000){
        Serial.print("Zakonczono pomiar");
        Serial.print("Osiagnieta wysokosc = ");
        Serial.print(maxAlt);
        Serial.println(" m ");
        oldMillisSerial = currentMillis;
        }
    }