#ifndef _H_BOARDS
#define _H_BOARDS

#include <nrf_gpio.h>

// stubs, nrf_bootloader require them even if we are not using buttons for dfu
#define BUTTON_PULL NRF_GPIO_PIN_PULLUP
#define NRF_GPIO_PIN_SENSE_LOW 0

#endif