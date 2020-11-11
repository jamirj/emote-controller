//#############################################################################
//
// FILE:   empty_driverlib_main.c
//
// TITLE:  Empty Project
//
// Empty Project Example
//
// This example is an empty project setup for Driverlib development.
//
//#############################################################################
// $TI Release: F2837xD Support Library v3.11.00.00 $
// $Release Date: Sun Oct  4 15:55:24 IST 2020 $
// $Copyright:
// Copyright (C) 2013-2020 Texas Instruments Incorporated - http://www.ti.com/
//
// Redistribution and use in source and binary forms, with or without 
// modification, are permitted provided that the following conditions 
// are met:
// 
//   Redistributions of source code must retain the above copyright 
//   notice, this list of conditions and the following disclaimer.
// 
//   Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the 
//   documentation and/or other materials provided with the   
//   distribution.
// 
//   Neither the name of Texas Instruments Incorporated nor the names of
//   its contributors may be used to endorse or promote products derived
//   from this software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// $
//#############################################################################

//
// Included Files
//
#include "driverlib.h"
#include "device.h"
#include "board.h"
#include <stdint.h>
#include <stdlib.h>

enum switch_commands{
    INCREMENT,
    DECREMENT,
    RESET,
    SET1,
    SET2,
    SET3,
    SET4,
    SET5,
    SET6
};

enum switch_states{
    STATE0,
    STATE1,
    STATE2,
    STATE3,
    STATE4,
    STATE5,
    STATE6
};

#define HSA_PIN 1
#define LSA_PIN 0
#define HSB_PIN 3
#define LSB_PIN 2
#define HSC_PIN 5
#define LSC_PIN 4

#define GUARD_PD_US 500


void switch_state_machine(enum switch_commands command)
{
    static uint8_t state = STATE0;
    switch(state){
        case STATE0:
            switch(command){
                case INCREMENT:
                    state = 1;
                    break;
                case DECREMENT:
                    state = 6;
                    break;
                case SET1:
                    state = 1;
                    break;
                case SET2:
                    state = 2;
                    break;
                case SET3:
                    state = 3;
                    break;
                case SET4:
                    state = 4;
                    break;
                case SET5:
                    state = 5;
                    break;
                case SET6:
                    state = 6;
                    break;
                default:
                    state = 0;
                    break;
            }
            break;
        case STATE1:
            switch(command){
                case INCREMENT:
                    state = STATE2;
                    break;
                case DECREMENT:
                    state = STATE6;
                    break;
                case RESET:
                    state = STATE0;
                    break;
            }
            break;
        case STATE2:
            switch(command){
                case INCREMENT:
                    state = STATE3;
                    break;
                case DECREMENT:
                    state = STATE1;
                    break;
                case RESET:
                    state = STATE0;
                    break;
            }
            break;
            case STATE3:
                switch(command){
                    case INCREMENT:
                        state = STATE4;
                        break;
                    case DECREMENT:
                        state = STATE2;
                        break;
                    case RESET:
                        state = STATE0;
                        break;
                }
                break;
            case STATE4:
                switch(command){
                    case INCREMENT:
                        state = STATE5;
                        break;
                    case DECREMENT:
                        state = STATE3;
                        break;
                    case RESET:
                        state = STATE0;
                        break;
                }
                break;
            case STATE5:
                switch(command){
                    case INCREMENT:
                        state = STATE6;
                        break;
                    case DECREMENT:
                        state = STATE4;
                        break;
                    case RESET:
                        state = STATE0;
                        break;
                }
                break;
            case STATE6:
                switch(command){
                    case INCREMENT:
                        state = STATE1;
                        break;
                    case DECREMENT:
                        state = STATE5;
                        break;
                    case RESET:
                        state = STATE0;
                        break;
                }
                break;
    }
    switch(state){
        case STATE0:
            GPIO_writePin(HSA_PIN,0);
            GPIO_writePin(LSA_PIN,0);
            GPIO_writePin(HSB_PIN,0);
            GPIO_writePin(LSB_PIN,0);
            GPIO_writePin(HSC_PIN,0);
            GPIO_writePin(LSC_PIN,0);
            break;
        case STATE1:
            GPIO_writePin(HSA_PIN,1);
            GPIO_writePin(LSA_PIN,0);
            GPIO_writePin(HSB_PIN,0);
            GPIO_writePin(LSB_PIN,1);
            GPIO_writePin(HSC_PIN,0);
            GPIO_writePin(LSC_PIN,0);
            break;
        case STATE2:
            GPIO_writePin(HSA_PIN,1);
            GPIO_writePin(LSA_PIN,0);
            GPIO_writePin(HSB_PIN,0);
            GPIO_writePin(LSB_PIN,0);
            GPIO_writePin(HSC_PIN,0);
            GPIO_writePin(LSC_PIN,1);
            break;
        case STATE3:
            GPIO_writePin(HSA_PIN,0);
            GPIO_writePin(LSA_PIN,0);
            GPIO_writePin(HSB_PIN,1);
            GPIO_writePin(LSB_PIN,0);
            GPIO_writePin(HSC_PIN,0);
            GPIO_writePin(LSC_PIN,1);
            break;
        case STATE4:
            GPIO_writePin(HSA_PIN,0);
            GPIO_writePin(LSA_PIN,1);
            GPIO_writePin(HSB_PIN,1);
            GPIO_writePin(LSB_PIN,0);
            GPIO_writePin(HSC_PIN,0);
            GPIO_writePin(LSC_PIN,0);
            break;
        case STATE5:
            GPIO_writePin(HSA_PIN,0);
            GPIO_writePin(LSA_PIN,1);
            GPIO_writePin(HSB_PIN,0);
            GPIO_writePin(LSB_PIN,0);
            GPIO_writePin(HSC_PIN,1);
            GPIO_writePin(LSC_PIN,0);
            break;
        case STATE6:
            GPIO_writePin(HSA_PIN,0);
            GPIO_writePin(LSA_PIN,0);
            GPIO_writePin(HSB_PIN,0);
            GPIO_writePin(LSB_PIN,1);
            GPIO_writePin(HSC_PIN,1);
            GPIO_writePin(LSC_PIN,0);
            break;
    }
    DEVICE_DELAY_US(GUARD_PD_US);
}

//
// Main
//
void main(void)
{
    Device_init();
    Interrupt_initModule();
    Interrupt_initVectorTable();

    Interrupt_enableMaster();

    Board_init();

    for(;;)
    {
        switch_state_machine(INCREMENT);
        DEVICE_DELAY_US(50000);
    }


}

//
// End of File
//
