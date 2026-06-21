#ifndef EEPROM_24C64_H
#define EEPROM_24C64_H

#include <stdbool.h>
#include <stdint.h>

#define EEPROM24C64_I2C_ADDRESS      0x50U
#define EEPROM24C64_SIZE_BYTES       8192U
#define EEPROM24C64_PAGE_SIZE        32U

bool EEPROM24C64_Init(void);
bool EEPROM24C64_IsPresent(void);
bool EEPROM24C64_Read(uint16_t address, uint8_t *data, uint16_t length);
bool EEPROM24C64_Write(uint16_t address, const uint8_t *data, uint16_t length);

#endif
