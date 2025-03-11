#ifndef __MARTIN_H__
#define __MARTIN_H__


#include <stdio.h>

void pinConfig(uint8_t pin, uint8_t peripheral, uint8_t config);

int readPin(uint8_t pin, uint8_t mode);

void writePin(uint8_t pin, uint8_t mode, uint8_t value);

void begin_adc(uint8_t channel, uint8_t mode);

int stop_adc();

int disable_adc();

void begin_systick(uint32_t reload);



#endif