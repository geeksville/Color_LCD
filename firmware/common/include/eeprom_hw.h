
#pragma once
#include "stdint.h"
#include "stdbool.h"

#define ADDRESS_KEY 0
#define KEY 7

void eeprom_hw_init(void);

// @casainho, this routine doesn't need to be public anymore - because it is private to the 850c
// HAL provided by eeprom-hw.c
// uint32_t eeprom_write(uint32_t ui32_address, uint8_t ui8_data);

bool flash_write_words(const void *value, uint16_t length_words);

// Read raw EEPROM data, return false if it is blank or malformatted
bool flash_read_words(void *dest, uint16_t length_words);


