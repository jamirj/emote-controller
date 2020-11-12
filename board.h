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

#define HALLA_PIN 32
#define HALLB_PIN 67
#define HALLC_PIN 111

void Board_init();
void GPIO_init();
void PinMux_init();

#endif /* BOARD_H_ */
