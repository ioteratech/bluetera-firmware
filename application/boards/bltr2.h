/*
	MIT License

	Copyright (c) 2019 Tensor Technologies LTD

	Permission is hereby granted, free of charge, to any person obtaining a copy
	of this software and associated documentation files (the "Software"), to deal
	in the Software without restriction, including without limitation the rights
	to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
	copies of the Software, and to permit persons to whom the Software is
	furnished to do so, subject to the following conditions:

	The above copyright notice and this permission notice shall be included in all
	copies or substantial portions of the Software.

	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
	IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
	FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
	AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
	LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
	OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
	SOFTWARE.
*/

#ifndef BLTR2_H
#define BLTR2_H

// ICM hardware config
#define ICM_SPI_INSTANCE			0
#define ICM_SPI_SCK_PIN 			27
#define ICM_SPI_MOSI_PIN			6
#define ICM_SPI_MISO_PIN			8
#define ICM_SPI_CS_PIN				26
#define ICM_INT1_PIN				41
#define ICM_INT2_PIN				11

// LIS2MDL hardware config
#define MAG_I2C_INSTACE				1
#define MAG_I2C_SCL_PIN				42
#define MAG_I2C_SDA_PIN				46
#define MAG_I2C_ADDR				0x1E

// Battery measuremnt
#define BATT_MEASURE_EN_PIN	 		45
#define BATT_MEASURE_CHANNEL		NRF_SAADC_INPUT_AIN0

// GPIO config
#define LED_RED_PIN					21
#define LED_GREEN_PIN				15
#define LED_BLUE_PIN				22
#define DEBUG_GPIO_TIMING			3

// SD card SPI
#define SDC_SPI_SCK_PIN     		18
#define SDC_SPI_MOSI_PIN    		13
#define SDC_SPI_MISO_PIN    		12
#define SDC_SPI_CS_PIN      		16
#define SDC_CD_PIN      			14

#endif