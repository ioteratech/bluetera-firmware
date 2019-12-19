#ifndef BLUETERA_ERROR_H_
#define BLUETERA_ERROR_H_

#ifdef __cplusplus
extern "C" {
#endif

#ifndef EXCLUDE_BLUETERA_HEADERS
	#include <app_error.h>
#endif

#define BLTR_ERROR_BASE_NUM				(0xF000)

#define BLTR_MSG_ERR_BASE				(BLTR_ERROR_BASE_NUM + 0x100)
#define BLTR_IMU_ERR_BASE				(BLTR_ERROR_BASE_NUM + 0x200)
#define BLTR_SD_CARD_ERR_BASE			(BLTR_ERROR_BASE_NUM + 0x300)

// Success code - same as NRF_SUCCESS from Nordic's app_error.h
#define BLTR_SUCCESS					(0x0)

// message module
#define BLTR_MSG_ERROR_OP_FAILED		(BLTR_MSG_ERR_BASE + 0)
#define BLTR_MSG_ERROR_RESOURCES		(BLTR_MSG_ERR_BASE + 1)
#define BLTR_MSG_ERROR_NO_TRANSPORT		(BLTR_MSG_ERR_BASE + 2)
#define BLTR_MSG_ERROR_UNSUPPORTED		(BLTR_MSG_ERR_BASE + 3)

// IMU module
#define BLTR_IMU_ERROR_INVALID_STATE	(BLTR_IMU_ERR_BASE+0)
#define BLTR_IMU_ERROR_INVALID_COMMAND	(BLTR_IMU_ERR_BASE+1)
#define BLTR_IMU_ERROR_INVALID_CONFIG	(BLTR_IMU_ERR_BASE+2)
#define BLTR_IMU_ERROR_CRITICAL			(BLTR_IMU_ERR_BASE+3)
#define BLTR_IMU_NOT_FOUND				(BLTR_IMU_ERR_BASE+4)

// SD-card module
#define BLTR_SD_CARD_ERROR_INVALID_STATE	(BLTR_SD_CARD_ERR_BASE+0)
#define BLTR_SD_CARD_ERROR_INIT_FAILED		(BLTR_SD_CARD_ERR_BASE+1)
#define BLTR_SD_CARD_ERROR_WRITE_FAILED		(BLTR_SD_CARD_ERR_BASE+2)
#define BLTR_SD_CARD_ERROR_CARD_FULL		(BLTR_SD_CARD_ERR_BASE+3)
#define BLTR_SD_CARD_ERROR_UNSUPPORTED		(BLTR_SD_CARD_ERR_BASE+4)


// utility macros
#define BLTR_RETURN_CODE_IF_ERROR(ERR_CODE)        	\
    do                                              \
    {                                               \
        const uint32_t LOCAL_ERR_CODE = (ERR_CODE);	\
        if (LOCAL_ERR_CODE != BLTR_SUCCESS)         \
        {                                           \
            return (ERR_CODE);              		\
        }                                           \
    } while (0)

#ifdef __cplusplus
}
#endif
#endif // BLUETERA_ERROR_H_
