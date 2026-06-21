#include "eeprom_24c64.h"

#include "delay.h"
#include "i2c1.h"

#include <stddef.h>

#define EEPROM24C64_TIMEOUT_MS        100U
#define EEPROM24C64_WRITE_RETRIES     20U
#define EEPROM24C64_WRITE_POLL_MS     1U

static bool EEPROM24C64_Select(void)
{
    return (I2C_MasterSetSlaveAddress(&i2c1_instance,
                                      EEPROM24C64_I2C_ADDRESS,
                                      false) == STATUS_SUCCESS);
}

static bool EEPROM24C64_IsRangeValid(uint16_t address, uint16_t length)
{
    uint32_t end = (uint32_t)address + length;

    return (length > 0U) && (end <= EEPROM24C64_SIZE_BYTES);
}

static bool EEPROM24C64_WaitReady(void)
{
    uint8_t address_bytes[2] = { 0U, 0U };
    uint8_t retry;

    for (retry = 0U; retry < EEPROM24C64_WRITE_RETRIES; retry++)
    {
        if (!EEPROM24C64_Select())
        {
            return false;
        }

        if (I2C_MasterSendDataBlocking(&i2c1_instance,
                                       address_bytes,
                                       sizeof(address_bytes),
                                       true,
                                       EEPROM24C64_TIMEOUT_MS) == STATUS_SUCCESS)
        {
            return true;
        }

        delay_ms(EEPROM24C64_WRITE_POLL_MS);
    }

    return false;
}

bool EEPROM24C64_Init(void)
{
    return EEPROM24C64_IsPresent();
}

bool EEPROM24C64_IsPresent(void)
{
    return EEPROM24C64_WaitReady();
}

bool EEPROM24C64_Read(uint16_t address, uint8_t *data, uint16_t length)
{
    uint8_t address_bytes[2];

    if ((data == NULL) || !EEPROM24C64_IsRangeValid(address, length))
    {
        return false;
    }

    if (!EEPROM24C64_Select())
    {
        return false;
    }

    address_bytes[0] = (uint8_t)(address >> 8U);
    address_bytes[1] = (uint8_t)(address & 0xFFU);
    if (I2C_MasterSendDataBlocking(&i2c1_instance,
                                   address_bytes,
                                   sizeof(address_bytes),
                                   false,
                                   EEPROM24C64_TIMEOUT_MS) != STATUS_SUCCESS)
    {
        return false;
    }

    return (I2C_MasterReceiveDataBlocking(&i2c1_instance,
                                          data,
                                          length,
                                          true,
                                          EEPROM24C64_TIMEOUT_MS) == STATUS_SUCCESS);
}

bool EEPROM24C64_Write(uint16_t address, const uint8_t *data, uint16_t length)
{
    uint8_t page_buffer[2U + EEPROM24C64_PAGE_SIZE];
    uint16_t remaining = length;
    uint16_t offset = 0U;

    if ((data == NULL) || !EEPROM24C64_IsRangeValid(address, length))
    {
        return false;
    }

    while (remaining > 0U)
    {
        uint16_t page_space = (uint16_t)(EEPROM24C64_PAGE_SIZE -
                              ((address + offset) % EEPROM24C64_PAGE_SIZE));
        uint16_t chunk = (remaining < page_space) ? remaining : page_space;
        uint16_t target = (uint16_t)(address + offset);
        uint16_t index;

        if (!EEPROM24C64_Select())
        {
            return false;
        }

        page_buffer[0] = (uint8_t)(target >> 8U);
        page_buffer[1] = (uint8_t)(target & 0xFFU);
        for (index = 0U; index < chunk; index++)
        {
            page_buffer[2U + index] = data[offset + index];
        }

        if (I2C_MasterSendDataBlocking(&i2c1_instance,
                                       page_buffer,
                                       (uint32_t)(2U + chunk),
                                       true,
                                       EEPROM24C64_TIMEOUT_MS) != STATUS_SUCCESS)
        {
            return false;
        }

        if (!EEPROM24C64_WaitReady())
        {
            return false;
        }

        offset = (uint16_t)(offset + chunk);
        remaining = (uint16_t)(remaining - chunk);
    }

    return true;
}
