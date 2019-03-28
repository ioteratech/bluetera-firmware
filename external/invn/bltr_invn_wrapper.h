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

// This wrapper is built around the ICM20649 Embedded Motion Driver Ver 3.8.1.
// Can be downloaded from Invensense/TDK website.

#ifndef BLTR_INVN_WRAPPER_H_
#define BLTR_INVN_WRAPPER_H_

#include "bltr_imu.h"

typedef struct
{
	bltr_imu_data_handler_t imu_data_handler;
	bltr_imu_data_handler_t imu_irq_data_handler;
	int(*read_reg)(void* spi, uint8_t reg, uint8_t* data, uint32_t len);
	int (*write_reg)(void* spi, uint8_t reg, const uint8_t* data, uint32_t len);
	void(*delay_ms)(int ms);
	void(*delay_us)(int us);
	uint64_t(*get_timestamp_us)(void);
	void(*enter_critical_section)(void); 
	void(*leave_critical_section)(void);	
	void* spi;
} bltr_invn_init_t;

void bltr_invn_init(const bltr_invn_init_t* init);
void bltr_invn_start(uint32_t period);
void bltr_invn_stop();
void bltr_invn_set_mode(bltr_imu_mode_t mode);
void bltr_invn_update();
void bltr_invn_set_freq_divider(uint8_t div);
void bltr_invn_set_fsr(uint16_t acc, uint16_t gyro);
void bltr_invn_get_fsr(uint16_t* acc, uint16_t* gyro);
void bltr_invn_irq_handler();

#endif // BLTR_INVN_WRAPPER_H_