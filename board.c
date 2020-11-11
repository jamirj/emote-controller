/*
 * board.c
 *
 *  Created on: Nov 11, 2020
 *      Author: ericc
 */

#include "board.h"
#include <stdint.h>

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
    GPIO_setPinConfig(GPIO_1_GPIO1);
    GPIO_setPinConfig(GPIO_2_GPIO2);
    GPIO_setPinConfig(GPIO_3_GPIO3);
    GPIO_setPinConfig(GPIO_4_GPIO4);
    GPIO_setPinConfig(GPIO_5_GPIO5);
}

void GPIO_init(){

    // Set the pin settings. Note that this can almost certainly be made more efficient if ported away from driverlib.
    uint8_t i = 0;
    for(i=0; i<6; i++){
        GPIO_setDirectionMode(i,GPIO_DIR_MODE_OUT);
        GPIO_setPadConfig(i, GPIO_PIN_TYPE_STD);
        GPIO_setMasterCore(i, GPIO_CORE_CPU1);
        GPIO_setQualificationMode(i, GPIO_QUAL_ASYNC);
    }

}
