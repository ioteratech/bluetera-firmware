#ifndef BLUETERA_ERROR_H_
#define BLUETERA_ERROR_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <app_error.h>

#define BLTR_ERROR_BASE_NUM			(0xF000)

#define BLTR_MSG_ERR_BASE			(BLTR_ERROR_BASE_NUM + 0x100)
#define BLTR_IMU_ERR_BASE			(BLTR_ERROR_BASE_NUM + 0x200)

#define BLTR_SUCCESS				(NRF_SUCCESS)
#define BLTR_MSG_ERROR_OP_FAILED	(BLTR_MSG_ERR_BASE+0)
#define BLTR_MSG_ERROR_RESOURCES	(BLTR_MSG_ERR_BASE+1)

#ifdef __cplusplus
}
#endif
#endif // BLUETERA_ERROR_H_
