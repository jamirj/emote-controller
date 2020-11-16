/*
 * board.h
 *
 *  Created on: Nov 11, 2020
 *      Author: ericc
 */

#ifndef BOARD_H_
#define BOARD_H_

#include "driverlib.h"
#include "device.h"

#define HSA_PIN 1
#define LSA_PIN 0
#define HSB_PIN 3
#define LSB_PIN 2
#define HSC_PIN 5
#define LSC_PIN 4

#define LSA_GPIO GPIO_0_GPIO0
#define HSA_GPIO GPIO_1_GPIO1
#define LSB_GPIO GPIO_2_GPIO2
#define HSB_GPIO GPIO_3_GPIO3
#define LSC_GPIO GPIO_4_GPIO4
#define HSC_GPIO GPIO_5_GPIO5

#define LSA_PWM GPIO_0_EPWM1A
#define HSA_PWM GPIO_1_EPWM1B
#define LSB_PWM GPIO_2_EPWM2A
#define HSB_PWM GPIO_3_EPWM2B
#define LSC_PWM GPIO_4_EPWM3A
#define HSC_PWM GPIO_5_EPWM3B

#define HALLA_PIN 32
#define HALLB_PIN 67
#define HALLC_PIN 111

void Board_init();
void GPIO_init();
void PinMux_init();

#endif /* BOARD_H_ */
