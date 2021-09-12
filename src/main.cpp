#include <Arduino.h>
#include <EEPROM.h>
#include <Wire.h>
#include <Adafruit_BMP280.h>
#include <ESP8266WebServer.h>
#include <string.h>
Adafruit_BMP280 bmp; // use I2C interface
Adafruit_Sensor *bmp_temp = bmp.getTemperatureSensor();
Adafruit_Sensor *bmp_pressure = bmp.getPressureSensor();


float startP;
float maxAlt;
float maxAltOld;
float Alt;
int x=0;
char ssid[32];
unsigned long currentMillis;
unsigned long oldMillis=0;


void setup() {
  EEPROM.begin(512);
  currentMillis = millis();
  Serial.begin(9600);
  Serial.println("BMP280 Sensor event test");

  //if (!bmp.begin(BMP280_ADDRESS_ALT, BMP280_CHIPID)) {
  if (!bmp.begin(0x76)) {
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
sprintf(ssid,"Wysokosc z pamieci %.3f m",maxAltOld);
WiFi.softAP(ssid);

}





void loop() {

      while(x<1){
      Serial.println(ssid);
      delay(100);
      Alt = bmp.readAltitude(startP);
        if(Alt > maxAlt){
        maxAlt = Alt;
        }
      if(maxAlt>Alt+2) {
        Serial.println(maxAlt);
        EEPROM.put(0,maxAlt);
        EEPROM.commit();
        sprintf(ssid, "Wysokosc z pomiaru %.2f",maxAlt);
        WiFi.softAP(ssid);
        x++;
        }
      currentMillis = millis();
      if(currentMillis > oldMillis + 1000){
        Serial.print("Approx altitude = ");
        Serial.print(Alt);
        Serial.println(" m ");
        Serial.print("Max altitude = ");
        Serial.print(maxAlt);
        Serial.println(" m ");
        Serial.print(maxAltOld);
        Serial.println(" m ");
        oldMillis = currentMillis;
        }
    }
    if(currentMillis > oldMillis + 1000){
        Serial.print("Zakonczono pomiar");
        Serial.print("Osiagnieta wysokosc = ");
        Serial.print(maxAlt);
        Serial.println(" m ");
         Serial.print(maxAltOld);
        oldMillis = currentMillis;
    }
Serial.println(ssid);
   delay(1000);
    }








