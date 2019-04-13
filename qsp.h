#include "Arduino.h"
#include "variables.h"

void qspComputeCrc(QspConfiguration_t *qsp, uint8_t dataByte);
void qspDecodeIncomingFrame(
    QspConfiguration_t *qsp, 
    uint8_t incomingByte, 
    BeaconState_t *beaconState,
    long beaconId
);
void qspClearPayload(QspConfiguration_t *qsp);
void qspEncodeFrame(QspConfiguration_t *qsp, uint8_t buffer[], uint8_t *size, uint8_t radioChannel);