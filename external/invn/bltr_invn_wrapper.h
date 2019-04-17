#ifndef BLTR_INVN_WRAPPER_H_
#define BLTR_INVN_WRAPPER_H_

#include "bltr_imu.h"
#include "bluetera_err.h"

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

ret_code_t bltr_invn_init(const bltr_invn_init_t* init);
ret_code_t bltr_invn_config(const bltr_imu_config_t* config);
ret_code_t bltr_invn_start();
ret_code_t bltr_invn_stop();

void bltr_invn_poll();
void bltr_invn_irq_handler();

#endif // BLTR_INVN_WRAPPER_H_