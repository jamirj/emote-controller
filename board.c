/*
 * board.c
 *
 *  Created on: Nov 11, 2020
 *      Author: ericc
 */

#include "board.h"

void Board_init()
{
    EALLOW;

    PinMux_init();
    GPIO_init();

    EDIS;
}

void PinMux_init()
{
    GPIO_setPinConfig(GPIO_0_GPIO0);
    //GPIO_setPinConfig(GPIO_1_GPIO1);
    //GPIO_setPinConfig(GPIO_2_GPIO2);
    //GPIO_setPinConfig(GPIO_3_GPIO3);
    //GPIO_setPinConfig(GPIO_4_GPIO4);
    //GPIO_setPinConfig(GPIO_5_GPIO5);
}

void GPIO_init(){
    GPIO_setDirectionMode(0,GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(0, GPIO_PIN_TYPE_STD);
    GPIO_setMasterCore(0, GPIO_CORE_CPU1);
    GPIO_setQualificationMode(0, GPIO_QUAL_ASYNC);
}
