#include <Arduino.h>
#include <SoftwareSerial.h>
#include <TinyGPS++.h>
#include "lora.h"
#include "radio_node.h"

#define GPS_POWER_PIN 12

#define GPS_SS_TX 10
#define GPS_SS_RX 9

SoftwareSerial gpsSerial(GPS_SS_RX, GPS_SS_TX); // RX, TX
TinyGPSPlus gps;

void setup()
{
    Serial.begin(115200);

	pinMode(LED_BUILTIN, OUTPUT);
    pinMode(GPS_POWER_PIN, OUTPUT);
    digitalWrite(GPS_POWER_PIN, HIGH);

    gpsSerial.begin(9600);
}

uint32_t nextSerialTaskTs = 0;
#define TASK_SERIAL_RATE 1000

void loop()
{
    while (gpsSerial.available() > 0) {
        gps.encode(gpsSerial.read());
    }

    if (nextSerialTaskTs < millis()) {
        Serial.print("LAT=");  Serial.println(gps.location.lat(), 6);
        Serial.print("LONG="); Serial.println(gps.location.lng(), 6);
        Serial.print("ALT=");  Serial.println(gps.altitude.meters());
        Serial.print("Sats=");  Serial.println(gps.satellites.value());

        nextSerialTaskTs = millis() + TASK_SERIAL_RATE;
    }

    // if (gpsSerial.available()) {
        // Serial.write(gpsSerial.read());
    // }
    // Serial.println("dupa");
    // digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    // delay(500);
}
