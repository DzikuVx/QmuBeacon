#include "platform_node.h"
#include "utils.h"

PlatformNode::PlatformNode(void) {
    
}

/**
 * Return true if new bind key was generated
 */
void PlatformNode::seed(void) {
    uint8_t val;

    val = EEPROM.read(EEPROM_ADDRESS_BIND_KEY_SEEDED);

    if (val != 0xf1) {
        EEPROM.write(EEPROM_ADDRESS_BIND_0, random(1, 255)); //Yes, from 1 to 254
        EEPROM.write(EEPROM_ADDRESS_BIND_1, random(1, 255)); //Yes, from 1 to 254
        EEPROM.write(EEPROM_ADDRESS_BIND_2, random(1, 255)); //Yes, from 1 to 254
        EEPROM.write(EEPROM_ADDRESS_BIND_3, random(1, 255)); //Yes, from 1 to 254
        EEPROM.write(EEPROM_ADDRESS_BIND_KEY_SEEDED, 0xf1);
    } 
}

long PlatformNode::loadBeaconId(void) {
    long tmp;

    tmp = EEPROM.read(EEPROM_ADDRESS_BIND_0);
    tmp += ((long)EEPROM.read(EEPROM_ADDRESS_BIND_1)) << 8;
    tmp += ((long)EEPROM.read(EEPROM_ADDRESS_BIND_2)) << 16;
    tmp += ((long)EEPROM.read(EEPROM_ADDRESS_BIND_3)) << 24;

    beaconId = tmp;
    return tmp;
}

void PlatformNode::saveBeaconId(long key) {

    uint8_t tmp[4];

    int32ToBuf(tmp, 0, key);

    EEPROM.write(EEPROM_ADDRESS_BIND_0, tmp[0]);
    EEPROM.write(EEPROM_ADDRESS_BIND_1, tmp[1]);
    EEPROM.write(EEPROM_ADDRESS_BIND_2, tmp[2]);
    EEPROM.write(EEPROM_ADDRESS_BIND_3, tmp[3]);
    EEPROM.write(EEPROM_ADDRESS_BIND_KEY_SEEDED, 0xf1);
}
