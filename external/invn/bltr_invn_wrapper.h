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
void bltr_invn_config(const bltr_imu_config_t* config);	// TODO(Tomer): implement
void bltr_invn_start(uint32_t period);					// TODO(Tomer): remove argument from footprint
void bltr_invn_stop();

void bltr_invn_set_mode(bltr_imu_mode_t mode);			// TODO(Tomer): deprecate
void bltr_invn_update();
void bltr_invn_set_freq_divider(uint8_t div);			// TODO(Tomer): deprecate
void bltr_invn_set_fsr(uint16_t acc, uint16_t gyro);	// TODO(Tomer): deprecate
void bltr_invn_get_fsr(uint16_t* acc, uint16_t* gyro);	// TODO(Tomer): deprecate
void bltr_invn_irq_handler();

#endif // BLTR_INVN_WRAPPER_H_