/*
 * Code generated from Atmel Start.
 *
 * This file will be overwritten when reconfiguring your Atmel Start project.
 * Please copy examples or other code you want to keep to a separate file
 * to avoid losing it when reconfiguring.
 */
#ifndef ATMEL_START_PINS_H_INCLUDED
#define ATMEL_START_PINS_H_INCLUDED

#include <hal_gpio.h>

// SAMD10 has 8 pin functions

#define GPIO_PIN_FUNCTION_A 0
#define GPIO_PIN_FUNCTION_B 1
#define GPIO_PIN_FUNCTION_C 2
#define GPIO_PIN_FUNCTION_D 3
#define GPIO_PIN_FUNCTION_E 4
#define GPIO_PIN_FUNCTION_F 5
#define GPIO_PIN_FUNCTION_G 6
#define GPIO_PIN_FUNCTION_H 7

#define LED_ROW0 GPIO(GPIO_PORTA, 2)
#define LED_ROW1 GPIO(GPIO_PORTA, 3)
#define LED_ROW2 GPIO(GPIO_PORTA, 4)
#define LED_ROW3 GPIO(GPIO_PORTA, 5)
#define LED_COL0 GPIO(GPIO_PORTA, 6)
#define LED_COL1 GPIO(GPIO_PORTA, 7)
#define LED_COL2 GPIO(GPIO_PORTA, 8)
#define LED_COL3 GPIO(GPIO_PORTA, 9)
#define PA22 GPIO(GPIO_PORTA, 22)
#define PA23 GPIO(GPIO_PORTA, 23)

#endif // ATMEL_START_PINS_H_INCLUDED
