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

void Board_init();
void GPIO_init();
void PinMux_init();

#endif /* BOARD_H_ */
