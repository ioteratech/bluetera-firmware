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

#ifndef BLTR1_H
#define BLTR1_H

// ICM hardware config
#define ICM_SPI_INSTANCE			0
#define ICM_SPI_SCK_PIN 			10
#define ICM_SPI_MOSI_PIN			9
#define ICM_SPI_MISO_PIN			3
#define ICM_SPI_CS_PIN				2
#define ICM_INTERRUPT_PIN			1

// GPIO config
#define LED_BLUE_PIN				26
#define DEBUG_GPIO_TIMING			23

// SD card SPI
#define SDC_SPI_SCK_PIN     		18
#define SDC_SPI_MOSI_PIN    		13
#define SDC_SPI_MISO_PIN    		12
#define SDC_SPI_CS_PIN      		16
#define SDC_CD_PIN      			14

#endif