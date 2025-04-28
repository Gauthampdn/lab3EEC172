//*****************************************************************************
//
// Copyright (C) 2014 Texas Instruments Incorporated - http://www.ti.com/ 
// 
// 
//  Redistribution and use in source and binary forms, with or without 
//  modification, are permitted provided that the following conditions 
//  are met:
//
//    Redistributions of source code must retain the above copyright 
//    notice, this list of conditions and the following disclaimer.
//
//    Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the 
//    documentation and/or other materials provided with the   
//    distribution.
//
//    Neither the name of Texas Instruments Incorporated nor the names of
//    its contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
//  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
//  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
//  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
//  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
//  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
//  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
//  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
//  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
//  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
//  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
//*****************************************************************************

//*****************************************************************************
//
// Application Name     - Timer Demo
// Application Overview - This application is to showcases the usage of Timer 
//                        DriverLib APIs. The objective of this application is 
//                        to showcase the usage of 16 bit timers to generate 
//                        interrupts which in turn toggle the state of the GPIO 
//                        (driving LEDs).
//                        Two timers with different timeout value(one is twice 
//                        the other) are set to toggle two different GPIOs which 
//                        in turn drives two different LEDs, which will give a 
//                        blinking effect.
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup timer_demo
//! @{
//
//*****************************************************************************

// Standard include
#include <stdio.h>

// Driverlib includes
#include "hw_types.h"
#include "interrupt.h"
#include "hw_ints.h"
#include "hw_apps_rcm.h"
#include "hw_common_reg.h"
#include "prcm.h"
#include "rom.h"
#include "rom_map.h"
#include "hw_memmap.h"
#include "timer.h"
#include "utils.h"

// Common interface includes
#include "timer_if.h"
#include "gpio_if.h"
#include "gpio.h"
#include "uart_if.h"

#include "pin_mux_config.h"


//*****************************************************************************
//                      MACRO DEFINITIONS
//*****************************************************************************
#define APPLICATION_VERSION        "1.4.0"
#define FOREVER                    1

//*****************************************************************************
//                      Global Variables for Vector Table
//*****************************************************************************
#if defined(ccs)
extern void (* const g_pfnVectors[])(void);
#endif
#if defined(ewarm)
extern uVectorEntry __vector_table;
#endif

//*****************************************************************************
//
// Globals used by the timer interrupt handler.
//
//*****************************************************************************
static volatile unsigned long g_ulBase;
static volatile unsigned long g_ulRefBase;
static volatile unsigned long g_ulRefTimerInts = 0;
static volatile unsigned long g_ulIntClearVector;
unsigned long g_ulTimerInts;

// Add global variable to store last rising edge time
static volatile unsigned long last_rising_edge_time = 0;

//*****************************************************************************
//
//! The interrupt handler for the first timer interrupt.
//!
//! \param  None
//!
//! \return none
//
//*****************************************************************************
void TimerBaseIntHandler(void)
{
    Timer_IF_InterruptClear(g_ulBase);
    g_ulTimerInts ++;
    GPIO_IF_LedToggle(MCU_GREEN_LED_GPIO);
}

//*****************************************************************************
//
//! The interrupt handler for the second timer interrupt.
//!
//! \param  None
//!
//! \return none
//
//*****************************************************************************
void TimerRefIntHandler(void)
{
    Timer_IF_InterruptClear(g_ulRefBase);
    g_ulRefTimerInts ++;
    GPIO_IF_LedToggle(MCU_RED_LED_GPIO);
}

//*****************************************************************************
//
//! Board Initialization & Configuration
//!
//! \param  None
//!
//! \return None
//
//*****************************************************************************
static void
BoardInit(void)
{
/* In case of TI-RTOS vector table is initialize by OS itself */
#ifndef USE_TIRTOS
  //
  // Set vector table base
  //
#if defined(ccs)
    MAP_IntVTableBaseSet((unsigned long)&g_pfnVectors[0]);
#endif
#if defined(ewarm)
    MAP_IntVTableBaseSet((unsigned long)&__vector_table);
#endif
#endif
    //
    // Enable Processor
    //
    MAP_IntMasterEnable();
    MAP_IntEnable(FAULT_SYSTICK);

    PRCMCC3200MCUInit();
}

static void
DisplayBanner(char * AppName)
{

    Report("\n\n\n\r");
    Report("\t\t *************************************************\n\r");
    Report("\t\t        CC3200 %s Application       \n\r", AppName);
    Report("\t\t *************************************************\n\r");
    Report("\n\n\n\r");
}

// Add GPIO interrupt handler
void GPIOIntHandler(void)
{
    GPIOIntClear(GPIOA0_BASE, 0x8);  // Clear interrupt for pin 58

    // Only process rising edge
    if (GPIOPinRead(GPIOA0_BASE, 0x8)) {
        unsigned long now = TimerValueGet(TIMERA0_BASE, TIMER_A);
        unsigned long elapsed_ticks;
        
        // For up-counting timer - we need to get elapsed ticks correctly
        if (now >= last_rising_edge_time) {
            elapsed_ticks = now - last_rising_edge_time;
        } else {
            // Timer wrapped around
            elapsed_ticks = (0xFFFFFFFF - last_rising_edge_time) + now;
        }
        
        unsigned long elapsed_us = elapsed_ticks / 80; // 80 MHz clock

        if (last_rising_edge_time != 0) { // skip first edge
            char msg[32];
            char bit = (elapsed_us > 1500) ? '1' : '0';
            UARTCharPut(UARTA0_BASE, bit);
            float elapsed_ms = elapsed_us / 1000.0f;
            snprintf(msg, sizeof(msg), " (%.3f ms)\r\n", elapsed_ms);
            Report(msg);
        }
        last_rising_edge_time = now;
    }
}

//*****************************************************************************
//
//!    main function demonstrates the use of the timers to generate
//! periodic interrupts.
//!
//! \param  None
//!
//! \return none
//
//*****************************************************************************
int
main(void)
{
    //
    // Initialize board configurations
    BoardInit();
    
    //
    // Pinmuxing for UART
    //
    PinMuxConfig();
    
    //
    // Initialize UART
    //
    InitTerm();
    ClearTerm();
    DisplayBanner("GPIO to UART Demo");

    //
    // Set up TIMERA0 as a free-running timer (periodic, max value)
    //
    MAP_PRCMPeripheralClkEnable(PRCM_TIMERA0, PRCM_RUN_MODE_CLK);
    MAP_PRCMPeripheralReset(PRCM_TIMERA0);
    MAP_TimerConfigure(TIMERA0_BASE, TIMER_CFG_A_PERIODIC_UP);
    MAP_TimerLoadSet(TIMERA0_BASE, TIMER_A, 0xFFFFFFFF);
    MAP_TimerEnable(TIMERA0_BASE, TIMER_A);

    //
    // Configure GPIO interrupt for pin 58
    //
    GPIOIntRegister(GPIOA0_BASE, GPIOIntHandler);  // Register interrupt handler
    GPIOIntTypeSet(GPIOA0_BASE, 0x8, GPIO_BOTH_EDGES);  // Trigger on both edges
    GPIOIntEnable(GPIOA0_BASE, 0x8);                    // Enable interrupt
    
    //
    // Loop forever while GPIO interrupts run.
    //
    while(FOREVER)
    {
        // Main loop does nothing - all work is done in interrupts
    }
}

//*****************************************************************************
//
// Close the Doxygen group.
//! @}
//
//*****************************************************************************


