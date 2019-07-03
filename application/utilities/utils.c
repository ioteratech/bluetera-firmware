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

#include "utils.h"

#include <nrfx_timer.h>
#include <app_timer.h>

static nrfx_timer_t _timer_instance = NRFX_TIMER_INSTANCE(1);

static void timer_event_handler(nrf_timer_event_t event_type, void* p_context)
{

}

void bltr_utils_init()
{
	nrfx_timer_config_t timer_cfg = NRFX_TIMER_DEFAULT_CONFIG;

	timer_cfg.frequency = NRF_TIMER_FREQ_1MHz;
	timer_cfg.mode = TIMER_MODE_MODE_Timer;
	timer_cfg.bit_width = NRF_TIMER_BIT_WIDTH_32;

	nrfx_timer_init(&_timer_instance, &timer_cfg, timer_event_handler);

	nrfx_timer_enable(&_timer_instance);
}

uint64_t bltr_utils_get_timestamp()
{
	static uint32_t last = 0;
	uint32_t now = nrfx_timer_capture(&_timer_instance, NRF_TIMER_CC_CHANNEL0);
	uint32_t dt = app_timer_cnt_diff_compute(now, last);
	last = now;
	
	static uint64_t elapsed = 0;
	elapsed += dt;

	// TODO handle overflow
	// TODO better accuracy (casting float to int truncates it)

	return elapsed;
}