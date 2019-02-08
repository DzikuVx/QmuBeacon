#include <Arduino.h>
#include <SoftwareSerial.h>
#include <TinyGPS++.h>
#include "lora.h"
#include "radio_node.h"

#define GPS_POWER_PIN 12

#define GPS_SS_TX 10
#define GPS_SS_RX 9

#ifdef ARDUINO_AVR_FEATHER32U4
    #define LORA_SS_PIN     8
    #define LORA_RST_PIN    4
    #define LORA_DI0_PIN    7

    #define BUTTON_0_PIN    9
    #define BUTTON_1_PIN    10
#else
    #error please select hardware
#endif

SoftwareSerial gpsSerial(GPS_SS_RX, GPS_SS_TX); // RX, TX
TinyGPSPlus gps;

RadioNode radioNode;
QspConfiguration_t qsp = {};
BeaconState_t beaconState = {};

uint8_t bindKey[4] = {0x05, 0x71, 0x64, 0xA3};

void setup()
{

    radioNode.init(LORA_SS_PIN, LORA_RST_PIN, LORA_DI0_PIN, onReceive);
    radioNode.reset();
    radioNode.canTransmit = true;

    randomSeed(analogRead(A4));

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
    bool transmitPayload = false;
    
    while (gpsSerial.available() > 0) {
        gps.encode(gpsSerial.read());
    }

    //Beacon is never hopping frequency
    radioNode.handleTxDoneState(false);

    radioNode.readAndDecode(
        &qsp,
        &beaconState,
        bindKey
    );

    if (transmitPayload)
    {
        radioNode.handleTx(&qsp, bindKey);
    }

    if (nextSerialTaskTs < millis()) {
        Serial.print("LAT=");  Serial.println(gps.location.lat(), 6);
        Serial.print("LONG="); Serial.println(gps.location.lng(), 6);
        Serial.print("ALT=");  Serial.println(gps.altitude.meters());
        Serial.print("Sats=");  Serial.println(gps.satellites.value());

        nextSerialTaskTs = millis() + TASK_SERIAL_RATE;
    }
}

void onReceive(int packetSize)
{
    /*
     * We can start reading only when radio is not reading.
     * If not reading, then we might start
     */
    if (radioNode.bytesToRead == NO_DATA_TO_READ) {
        if (packetSize >= MIN_PACKET_SIZE && packetSize <= MAX_PACKET_SIZE) {
            //We have a packet candidate that might contain a valid QSP packet
            radioNode.bytesToRead = packetSize;
        } else {
            /*
            That packet was not very interesting, just flush it, we have no use
            */
            LoRa.sleep();
            LoRa.receive();
            radioNode.radioState = RADIO_STATE_RX;
        }
    }
}