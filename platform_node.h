#pragma once

#include "Arduino.h"
#include "radio_node.h"
#include <EEPROM.h>

#ifndef PLATFORM_NODE_H
#define PLATFORM_NODE_H
 
enum platformConfigMemoryLayout { 
    EEPROM_ADDRESS_BIND_KEY_SEEDED = 0x00, 
    EEPROM_ADDRESS_BIND_0, 
    EEPROM_ADDRESS_BIND_1, 
    EEPROM_ADDRESS_BIND_2, 
    EEPROM_ADDRESS_BIND_3,
    PLATFORM_CONFIG_LAST_BYTE 
}; 

class PlatformNode {

    public:
        PlatformNode(void);
        void seed(void);
        long loadBeaconId(void);
        void saveBeaconId(long key);
        long beaconId;
};

#endif
