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

	uint8_t powerLevel = 0x1F; //Options: 0=Off to 255=50mA
	uint8_t sampleAverage = 0x08; //Options: 1, 2, 4, 8, 16, 32
	uint8_t ledMode = 0x03; //Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
	int sampleRate = 100; //Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
	int pulseWidth = 411; //Options: 69, 118, 215, 411
	int adcRange = 4096; //Options: 2048, 4096, 8192, 16384
	heartSensor.setup(powerLevel, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange);

	while (1) 
	{
		// uint32_t redValue = heartSensor.getRed();
		// uint32_t irValue = heartSensor.getIR();
		// uint32_t greenValue = heartSensor.getGreen();
		// printf("%lu %lu %lu\n", (unsigned long)redValue, (unsigned long)irValue, (unsigned long)greenValue);
		printf("%lu,%lu\n", (unsigned long)heartSensor.getIR(), (unsigned long)heartSensor.getRed());
		// printf("%lu\n", (unsigned long)heartSensor.getIR());

		// busy_wait_ms(500);
	}
	return 0;
}