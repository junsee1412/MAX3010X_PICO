#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "max3010x/max3010x.hpp"


#define MAX3010X_ADDRESS	0x57

#define PIN_WIRE_SDA 4
#define PIN_WIRE_SCL 5
#define I2C_PORT i2c0
MAX3010X heartSensor(I2C_PORT, PIN_WIRE_SDA, PIN_WIRE_SCL, I2C_SPEED_FAST);

int main(void) {

	stdio_init_all();

	busy_wait_ms(500);
	while (heartSensor.begin() != true)
	{
		printf("MAX30102 not connect r fail load calib coeff \r\n");
		busy_wait_ms(500);
	}

	heartSensor.setup();

	while (1) 
	{
		uint32_t redValue = heartSensor.getRed();
		uint32_t irValue = heartSensor.getIR();
		uint32_t greenValue = heartSensor.getGreen();
		printf("Red %lu, IR %lu, Green %lu \r\n", (unsigned long)redValue, (unsigned long)irValue, (unsigned long)greenValue);

		busy_wait_ms(500);
	}
	return 0;
}