/*
 * Code generated from Atmel Start.
 *
 * This file will be overwritten when reconfiguring your Atmel Start project.
 * Please copy examples or other code you want to keep to a separate file
 * to avoid losing it when reconfiguring.
 */

#include "driver_init.h"
#include <peripheral_clk_config.h>
#include <utils.h>
#include <hal_init.h>
#include <hpl_gclk_base.h>
#include <hpl_pm_base.h>

#include <hpl_adc_base.h>

struct adc_sync_descriptor ADC_0;

struct i2c_m_sync_desc I2C_0;

struct calendar_descriptor CALENDAR_0;

void ADC_0_PORT_init(void)
{
}

void ADC_0_CLOCK_init(void)
{
	_pm_enable_bus_clock(PM_BUS_APBC, ADC);
	_gclk_enable_channel(ADC_GCLK_ID, CONF_GCLK_ADC_SRC);
}

void ADC_0_init(void)
{
	ADC_0_CLOCK_init();
	ADC_0_PORT_init();
	adc_sync_init(&ADC_0, ADC, (void *)NULL);
	// adc_sync_set_reference(&ADC_0, 0);
	// adc_sync_set_channel_differential_mode(&ADC_0, 0, ADC_DIFFERENTIAL_MODE_DIFFERENTIAL);
	// adc_sync_set_inputs(&ADC_0, 0x1a, 0x18, 0);
}

void I2C_0_PORT_init(void)
{

	gpio_set_pin_pull_mode(PA22,
						   // <y> Pull configuration
						   // <id> pad_pull_config
						   // <GPIO_PULL_OFF"> Off
						   // <GPIO_PULL_UP"> Pull-up
						   // <GPIO_PULL_DOWN"> Pull-down
						   GPIO_PULL_OFF);

	gpio_set_pin_function(PA22, PINMUX_PA22D_SERCOM2_PAD0);

	gpio_set_pin_pull_mode(PA23,
						   // <y> Pull configuration
						   // <id> pad_pull_config
						   // <GPIO_PULL_OFF"> Off
						   // <GPIO_PULL_UP"> Pull-up
						   // <GPIO_PULL_DOWN"> Pull-down
						   GPIO_PULL_OFF);

	gpio_set_pin_function(PA23, PINMUX_PA23D_SERCOM2_PAD1);
}

void I2C_0_CLOCK_init(void)
{
	_pm_enable_bus_clock(PM_BUS_APBC, SERCOM2);
	_gclk_enable_channel(SERCOM2_GCLK_ID_CORE, CONF_GCLK_SERCOM2_CORE_SRC);
	_gclk_enable_channel(SERCOM2_GCLK_ID_SLOW, CONF_GCLK_SERCOM2_SLOW_SRC);
}

void I2C_0_init(void)
{
	I2C_0_CLOCK_init();
	i2c_m_sync_init(&I2C_0, SERCOM2);
	I2C_0_PORT_init();
}

void CALENDAR_0_CLOCK_init(void)
{
	_pm_enable_bus_clock(PM_BUS_APBA, RTC);
	_gclk_enable_channel(RTC_GCLK_ID, CONF_GCLK_RTC_SRC);
}

void CALENDAR_0_init(void)
{
	CALENDAR_0_CLOCK_init();
	calendar_init(&CALENDAR_0, RTC);
}

void system_init(void)
{
	init_mcu();

	// GPIO on PA02

	/*gpio_set_pin_level(LED_ROW0,
					   // <y> Initial level
					   // <id> pad_initial_level
					   // <false"> Low
					   // <true"> High
					   false);

	// Set pin direction to output
	gpio_set_pin_direction(LED_ROW0, GPIO_DIRECTION_OUT);

	gpio_set_pin_function(LED_ROW0, GPIO_PIN_FUNCTION_OFF);

	// GPIO on PA03

	gpio_set_pin_level(LED_ROW1,
					   // <y> Initial level
					   // <id> pad_initial_level
					   // <false"> Low
					   // <true"> High
					   false);

	// Set pin direction to output
	gpio_set_pin_direction(LED_ROW1, GPIO_DIRECTION_OUT);

	gpio_set_pin_function(LED_ROW1, GPIO_PIN_FUNCTION_OFF);

	// GPIO on PA04

	gpio_set_pin_level(LED_ROW2,
					   // <y> Initial level
					   // <id> pad_initial_level
					   // <false"> Low
					   // <true"> High
					   false);

	// Set pin direction to output
	gpio_set_pin_direction(LED_ROW2, GPIO_DIRECTION_OUT);

	gpio_set_pin_function(LED_ROW2, GPIO_PIN_FUNCTION_OFF);

	// GPIO on PA05

	gpio_set_pin_level(LED_ROW3,
					   // <y> Initial level
					   // <id> pad_initial_level
					   // <false"> Low
					   // <true"> High
					   false);

	// Set pin direction to output
	gpio_set_pin_direction(LED_ROW3, GPIO_DIRECTION_OUT);

	gpio_set_pin_function(LED_ROW3, GPIO_PIN_FUNCTION_OFF);

	// GPIO on PA06

	gpio_set_pin_level(LED_COL0,
					   // <y> Initial level
					   // <id> pad_initial_level
					   // <false"> Low
					   // <true"> High
					   false);

	// Set pin direction to output
	gpio_set_pin_direction(LED_COL0, GPIO_DIRECTION_OUT);

	gpio_set_pin_function(LED_COL0, GPIO_PIN_FUNCTION_OFF);

	// GPIO on PA07

	gpio_set_pin_level(LED_COL1,
					   // <y> Initial level
					   // <id> pad_initial_level
					   // <false"> Low
					   // <true"> High
					   false);

	// Set pin direction to output
	gpio_set_pin_direction(LED_COL1, GPIO_DIRECTION_OUT);

	gpio_set_pin_function(LED_COL1, GPIO_PIN_FUNCTION_OFF);

	// GPIO on PA08

	gpio_set_pin_level(LED_COL2,
					   // <y> Initial level
					   // <id> pad_initial_level
					   // <false"> Low
					   // <true"> High
					   false);

	// Set pin direction to output
	gpio_set_pin_direction(LED_COL2, GPIO_DIRECTION_OUT);

	gpio_set_pin_function(LED_COL2, GPIO_PIN_FUNCTION_OFF);

	// GPIO on PA09

	gpio_set_pin_level(LED_COL3,
					   // <y> Initial level
					   // <id> pad_initial_level
					   // <false"> Low
					   // <true"> High
					   false);

	// Set pin direction to output
	gpio_set_pin_direction(LED_COL3, GPIO_DIRECTION_OUT);

	gpio_set_pin_function(LED_COL3, GPIO_PIN_FUNCTION_OFF);
	*/
	// gpio_set_port_level(GPIO_PORTA, 0x3FC, false);
	// gpio_set_port_direction(GPIO_PORTA, 0x3FC, GPIO_DIRECTION_OUT);
	for (int i = 2; i < 10; i++)
	{
		// gpio_set_pin_pull_mode(i, GPIO_PULL_OFF);
		gpio_set_pin_level(i, false);
		gpio_set_pin_direction(i, GPIO_DIRECTION_OUT);
		gpio_set_pin_function(i, GPIO_PIN_FUNCTION_OFF);
	}

	ADC_0_init();

	I2C_0_init();

	CALENDAR_0_init();
}
