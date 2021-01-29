/*
 * LP5018.h
 *
 *  Created on: Jan 6, 2021
 *      Author: Luke
 */

#ifndef LP5018_H_
#define LP5018_H_

#include "main.h"
#include "stm32f0xx_hal.h"

#define REG_DEVICE_CONFIG0  0
#define REG_DEVICE_CONFIG1  1
#define REG_LED_CONFIG0  2
#define REG_BANK_BRIGHTNESS  3
#define REG_BANK_A_COLOR  4
#define REG_BANK_B_COLOR  5
#define REG_BANK_C_COLOR  6
#define REG_LED0_BRIGHTNESS  7
#define REG_LED1_BRIGHTNESS  8
#define REG_LED2_BRIGHTNESS  9
#define REG_LED3_BRIGHTNESS  10
#define REG_LED4_BRIGHTNESS  11
#define REG_LED5_BRIGHTNESS  12
#define REG_LED6_BRIGHTNESS  13 //only LP5024
#define REG_LED7_BRIGHTNESS  14 //only LP5024
#define REG_OUT0_COLOR  15
#define REG_OUT1_COLOR  16
#define REG_OUT2_COLOR  17
#define REG_OUT3_COLOR  18
#define REG_OUT4_COLOR  19
#define REG_OUT5_COLOR  20
#define REG_OUT6_COLOR  21
#define REG_OUT7_COLOR  22
#define REG_OUT8_COLOR  23
#define REG_OUT9_COLOR  24
#define REG_OUT10_COLOR  25
#define REG_OUT11_COLOR  26
#define REG_OUT12_COLOR  27
#define REG_OUT13_COLOR  28
#define REG_OUT14_COLOR  29
#define REG_OUT15_COLOR  30
#define REG_OUT16_COLOR  31
#define REG_OUT17_COLOR  32
#define REG_OUT18_COLOR  33 //only LP5024
#define REG_OUT19_COLOR  34 //only LP5024
#define REG_OUT20_COLOR  35 //only LP5024
#define REG_OUT21_COLOR  36 //only LP5024
#define REG_OUT22_COLOR  37 //only LP5024
#define REG_OUT23_COLOR  38 //only LP5024
#define REG_RESET  39
#define REG_TOTAL  40

uint8_t write_Reg(I2C_HandleTypeDef *hi2c1, uint8_t regAddr, uint8_t writeData, uint8_t length);
uint8_t write_Continuous_Regs(I2C_HandleTypeDef *hi2c1, uint8_t regAddr, uint8_t *writeData, uint8_t length);
uint8_t read_Reg(I2C_HandleTypeDef *hi2c1, uint8_t *regAddr, uint8_t *rxdata, uint8_t length);
void configLEDs(I2C_HandleTypeDef *hi2c1, uint8_t ledsOn);
void setLEDsColor(I2C_HandleTypeDef *hi2c1, uint8_t brightness, uint8_t red, uint8_t green, uint8_t blue);

#endif /* LP5018_H_ */

