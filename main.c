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
#include <stdbool.h>

// Driverlib includes
#include "hw_types.h"
#include "hw_memmap.h"
#include "interrupt.h"
#include "hw_ints.h"
#include "hw_apps_rcm.h"
#include "hw_common_reg.h"
#include "prcm.h"
#include "rom.h"
#include "rom_map.h"
#include "spi.h"
#include "uart.h"
#include "hw_memmap.h"
#include "timer.h"
#include "utils.h"
#include "Adafruit_GFX.h"
#include "glcdfont.h"
#include "oled_test.h"
#include "Adafruit_SSD1351.h"

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

#define SPI_IF_BIT_RATE  1000000
#define TR_BUFF_SIZE     100

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
static volatile unsigned long last_valid_edge_time = 0;  // Track last edge that we accepted for processing
static volatile unsigned char ready_to_receive = 1;      // Flag to track if we're ready to receive new bit patterns
static volatile unsigned long last_digit_time = 0;       // Time when last digit was output
static volatile int received_digit = -1;
static volatile unsigned short cur_pattern = 0;
static volatile int got_a_code = 0;  // Changed from -1 to 0 to properly track

// Add global buffer to store bits for pattern matching
#define PATTERN_SIZE 16
#define READY_DELAY_MS 50    // 100ms delay before accepting new patterns

// How long to wait (ms) before committing a tap sequence
#define MULTI_TAP_TIMEOUT_MS 1000

#define MAX_TEXT_LEN 32

// For the display
#define OLED_WIDTH      128
#define OLED_HEIGHT     128
#define CHAR_WIDTH       8    // pixels per character horizontally
#define CHAR_HEIGHT      8    // pixels per character vertically
#define CHARS_PER_LINE  (OLED_WIDTH / CHAR_WIDTH)   // =16
#define MAX_LINES       (OLED_HEIGHT / CHAR_HEIGHT) // =16

static unsigned short bit_buffer = 0;
static unsigned char bit_count = 0;
static unsigned char debug_mode = 0; // Set to 1 for raw bit output, 0 for digit decoding

// State for multi-tap decoding
static volatile int  last_multi_key      = -1;        // last numeric key (0–9)
static volatile int  multi_tap_count     = 0;         // how many taps so far
static volatile unsigned long last_tap_ts = 0;         // timestamp of last tap (in timer ticks)

static char  text_buffer[MAX_TEXT_LEN+1] = "";  // committed text + '\0'
static int   text_len        = 0;               // number of chars in buffer

// Add buffer for received UART data
static char received_text[MAX_TEXT_LEN+1] = "";
static int received_text_len = 0;

// Add circular buffer for UART receive
#define UART_BUFFER_SIZE 64
static char uart_buffer[UART_BUFFER_SIZE];
static int uart_head = 0;
static int uart_tail = 0;
static volatile int uart_count = 0;

// IR remote button patterns - each 16-bit code represents a digit
static const unsigned short PATTERN_0 = 0xC142; // 1100000101000010
static const unsigned short PATTERN_1 = 0xC202; // 1100001000000010
static const unsigned short PATTERN_2 = 0xC102; // 1100000100000010
static const unsigned short PATTERN_3 = 0xC302; // 1100001100000010
static const unsigned short PATTERN_4 = 0xC082; // 1100000010000010
static const unsigned short PATTERN_5 = 0xC282; // 1100001010000010
static const unsigned short PATTERN_6 = 0xC182; // 1100000110000010
static const unsigned short PATTERN_7 = 0xC382; // 1100001110000010
static const unsigned short PATTERN_8 = 0xC042; // 1100000001000010
static const unsigned short PATTERN_9 = 0xC242; // 1100001001000010
static const unsigned short PATTERN_L = 0xC3D2; // 1100001111010010
static const unsigned short PATTERN_M = 0xC3A2; // 1100001110100010

// The multi-tap character map (index by key)
static const char *multi_tap_map[10] = {
    " ",    // 0 -> space
    "1",    // 1 -> just "1"
    "abc2", // 2
    "def3", // 3
    "ghi4", // 4
    "jkl5", // 5
    "mno6", // 6
    "pqrs7",// 7
    "tuv8", // 8
    "wxyz9" // 9
};

// Function declarations
void DrawTextBuffer(char current);

// Function to add a character to the UART buffer
void UARTBufferAdd(char c)
{
    if (uart_count < UART_BUFFER_SIZE) {
        uart_buffer[uart_head] = c;
        uart_head = (uart_head + 1) % UART_BUFFER_SIZE;
        uart_count++;
    }
}

// Function to get a character from the UART buffer
char UARTBufferGet(void)
{
    char c = '\0';
    if (uart_count > 0) {
        c = uart_buffer[uart_tail];
        uart_tail = (uart_tail + 1) % UART_BUFFER_SIZE;
        uart_count--;
    }
    return c;
}

// Check if UART buffer has characters
int UARTBufferAvailable(void)
{
    return uart_count;
}

// UART Interrupt Handler
void UARTIntHandler(void)
{
    unsigned long ulStatus;
    
    // Get the interrupt status
    ulStatus = UARTIntStatus(UARTA0_BASE, true);
    
    // Clear the interrupt
    UARTIntClear(UARTA0_BASE, ulStatus);
    
    // Process received characters
    while (UARTCharsAvail(UARTA0_BASE)) {
        char c = UARTCharGetNonBlocking(UARTA0_BASE);
        
        // Print to terminal what was received
        Report("received: %c\r\n", c);
        
        // Add to our buffer
        UARTBufferAdd(c);
    }
}

// Function to process characters from UART buffer
void ProcessUARTData(void)
{
    while (UARTBufferAvailable()) {
        char c = UARTBufferGet();
        
        // Handle backspace
        if (c == 8 || c == 127) {
            if (received_text_len > 0) {
                received_text_len--;
                received_text[received_text_len] = '\0';
            }
        } 
        // Handle newline/carriage return
        else if (c == '\r' || c == '\n') {
            // Only add newline if it's not consecutive
            if (received_text_len > 0 && received_text[received_text_len-1] != '\n') {
                // Treat as end of message
                DrawTextBuffer(0);
            }
        }
        // Handle debug mode toggle
        else if (c == 'd' || c == 'D') {
            debug_mode = !debug_mode;
            Report("\r\nDebug mode ");
            Report(debug_mode ? "enabled\r\n" : "disabled\r\n");
        }
        // Regular characters
        else if (received_text_len < MAX_TEXT_LEN) {
            received_text[received_text_len++] = c;
            received_text[received_text_len] = '\0';
            DrawTextBuffer(0);
        }
    }
}

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

static void delay(unsigned long ulCount){
    int i;

  do{
    ulCount--;
        for (i=0; i< 65535; i++) ;
    }while(ulCount);
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

// Draw entire committed buffer plus an optional "current" char,
// wrapping every CHARS_PER_LINE characters.
void DrawTextBuffer(char current)
{
    fillScreen(BLACK);
    
    // Draw a white line in the middle of the screen
    drawFastHLine(0, OLED_HEIGHT/2, OLED_WIDTH, WHITE);
    
    // Draw committed chars (top half)
    unsigned int i;
    for(i = 0; i < text_len; i++) {
        int line = i / CHARS_PER_LINE;
        int col  = i % CHARS_PER_LINE;
        if (line >= MAX_LINES/2) break; // Only use top half
        int x = col * CHAR_WIDTH;
        int y = line * CHAR_HEIGHT;
        char s[2] = { text_buffer[i], '\0' };
        drawString(x, y, s, WHITE, BLACK, 1);
    }
    
    // Draw in‐progress char (if any)
    if (current) {
        int idx  = text_len;
        int line = idx / CHARS_PER_LINE;
        int col  = idx % CHARS_PER_LINE;
        if (line < MAX_LINES/2) {
            int x = col * CHAR_WIDTH;
            int y = line * CHAR_HEIGHT;
            char s[2] = { current, '\0' };
            drawString(x, y, s, WHITE, BLACK, 1);
        }
    }
    
    // Draw received UART text (bottom half)
    for(i = 0; i < received_text_len; i++) {
        int line = (i / CHARS_PER_LINE) + (MAX_LINES/2 + 1); // +1 to skip the line with divider
        int col  = i % CHARS_PER_LINE;
        if (line >= MAX_LINES) break;
        int x = col * CHAR_WIDTH;
        int y = line * CHAR_HEIGHT;
        char s[2] = { received_text[i], '\0' };
        drawString(x, y, s, WHITE, BLACK, 1);
    }
}

void HandleMultiTap(int key) {
    unsigned long now = TimerValueGet(TIMERA0_BASE, TIMER_A);
    unsigned long delta = (now >= last_tap_ts)
        ? (now - last_tap_ts)
        : (0xFFFFFFFF - last_tap_ts) + now;
    unsigned long elapsed_ms = (delta / 80) / 1000;  // 80 ticks = 1µs

    Report("Multi-tap: key=%d, elapsed=%ld ms\r\n", key, elapsed_ms);
    
    bool same_key   = (key == last_multi_key && elapsed_ms < MULTI_TAP_TIMEOUT_MS);
    const char *choices = multi_tap_map[key];
    int  choice_len = strlen(choices);

    // If it's a new key OR we've timed out, commit previous char
    if ((!same_key) && last_multi_key >= 0) {
        // previous letter was:
        char prev = multi_tap_map[last_multi_key][multi_tap_count];
        Report("Committing character: %c\r\n", prev);
        if (text_len < MAX_TEXT_LEN) {
            text_buffer[text_len++] = prev;
            text_buffer[text_len]  = '\0';
        }
        // reset tap count for this new sequence
        multi_tap_count = 0;
    }
    else if (same_key) {
        // advance within the same key
        multi_tap_count = (multi_tap_count + 1) % choice_len;
        Report("Multi-tap count: %d, choice: %c\r\n", multi_tap_count, choices[multi_tap_count]);
    }
    // else: brand new key and first tap → multi_tap_count stays at 0

    // pick the current "in-progress" char
    char current_char = choices[multi_tap_count];

    // redraw entire line
    DrawTextBuffer(current_char);

    // update state
    last_multi_key = key;
    last_tap_ts    = now;
}

// Function to commit any pending multi-tap character
void CommitPendingChar() {
    if (last_multi_key >= 0) {
        // Get the character that was being typed
        char current = multi_tap_map[last_multi_key][multi_tap_count];
        Report("Committing pending character: %c\r\n", current);
        
        if (text_len < MAX_TEXT_LEN) {
            text_buffer[text_len++] = current;
            text_buffer[text_len] = '\0';
        }
        
        // Reset multi-tap state
        last_multi_key = -1;
        multi_tap_count = 0;
    }
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
        float elapsed_ms = elapsed_us / 1000.0f;

        // Check if ready flag should be set (100ms since last digit was output)
        if (!ready_to_receive) {
            unsigned long digit_elapsed_ticks;
            if (now >= last_digit_time) {
                digit_elapsed_ticks = now - last_digit_time;
            } else {
                // Timer wrapped around
                digit_elapsed_ticks = (0xFFFFFFFF - last_digit_time) + now;
            }
            unsigned long digit_elapsed_ms = (digit_elapsed_ticks / 80) / 1000; // Convert to ms
            
            if (digit_elapsed_ms >= READY_DELAY_MS) {
                // 100ms has elapsed since last digit - now ready to receive
                ready_to_receive = 1;
                if (debug_mode) {
                    Report("Now ready to receive\r\n");
                }
            } else {
                // Reset the timer
                last_digit_time = now;
                if (debug_mode) {
                    Report("Timer reset (%dms)\r\n", digit_elapsed_ms);
                }
            }
        }
        
        // Update last edge time unconditionally
        last_rising_edge_time = now;
        
        // Only process bits if we're ready to receive
        if (ready_to_receive) {
            if (last_rising_edge_time != 0) { // skip first edge
                // Debug mode: Output raw bits
                if (debug_mode) {
                    unsigned char current_bit = (elapsed_us > 1500) ? 1 : 0;
                    char bit = current_bit ? '1' : '0';
                    
                    // Add a newline before output if more than 3ms has passed
                    if (elapsed_ms > 3.0f) {
                        Report("\r\n");
                    }
                    
                    // Just output the bit
                    UARTCharPut(UARTA0_BASE, bit);
                } 
                // Normal mode: Collect and decode patterns
                else {
                    unsigned char current_bit = (elapsed_us > 1500) ? 1 : 0;
                    
                    // Add a newline before output if more than 3ms has passed
                    if (elapsed_ms > 3.0f) {
                        // Reset bit buffer on long pauses
                        bit_buffer = 0;
                        bit_count = 0;
                    }
                    
                    // Add the current bit to our buffer
                    bit_buffer = (bit_buffer << 1) | current_bit;
                    bit_count++;
                    
                    // If we have collected 16 bits, check for pattern match
                    if (bit_count >= PATTERN_SIZE) {
                        got_a_code = 1;  // Set to 1 to indicate we got a code
                        cur_pattern = bit_buffer;
                        // Check which digit pattern matches
                        if (bit_buffer == PATTERN_0) {
                            Report("0\r\n");

                        } else if (bit_buffer == PATTERN_1) {
                            Report("1\r\n");

                        } else if (bit_buffer == PATTERN_2) {
                            Report("2\r\n");

                        } else if (bit_buffer == PATTERN_3) {
                            Report("3\r\n");

                        } else if (bit_buffer == PATTERN_4) {
                            Report("4\r\n");

                        } else if (bit_buffer == PATTERN_5) {
                            Report("5\r\n");

                        } else if (bit_buffer == PATTERN_6) {
                            Report("6\r\n");

                        } else if (bit_buffer == PATTERN_7) {
                            Report("7\r\n");

                        } else if (bit_buffer == PATTERN_8) {
                            Report("8\r\n");

                        } else if (bit_buffer == PATTERN_9) {
                            Report("9\r\n");

                        } else if (bit_buffer == PATTERN_L) {
                            Report("LAST\r\n");

                        } else if (bit_buffer == PATTERN_M) {
                            Report("MUTE detected - sending text\r\n");

                        }
                        
                        // Set not ready and start timer after digit is printed
                        ready_to_receive = 0;
                        last_digit_time = now;
                        
                        // Reset for next pattern
                        bit_buffer = 0;
                        bit_count = 0;
                    }
                }
            }
        } else if (debug_mode) {
            // Enhanced debug output showing time elapsed in timer
            unsigned long digit_elapsed_ticks;
            if (now >= last_digit_time) {
                digit_elapsed_ticks = now - last_digit_time;
            } else {
                // Timer wrapped around
                digit_elapsed_ticks = (0xFFFFFFFF - last_digit_time) + now;
            }
            unsigned long digit_elapsed_ms = (digit_elapsed_ticks / 80) / 1000; // Convert to ms
            
            Report("Signal ignored - %dms in 100ms timer\r\n", digit_elapsed_ms);
        }
    }
}

void MasterMain()
{
    //
    // Reset SPI
    //
    MAP_SPIReset(GSPI_BASE);

    //
    // Configure SPI interface
    //
    MAP_SPIConfigSetExpClk(GSPI_BASE,MAP_PRCMPeripheralClockGet(PRCM_GSPI),
                     SPI_IF_BIT_RATE,SPI_MODE_MASTER,SPI_SUB_MODE_0,
                     (SPI_SW_CTRL_CS |
                     SPI_4PIN_MODE |
                     SPI_TURBO_OFF |
                     SPI_CS_ACTIVEHIGH |
                     SPI_WL_8));

    //
    // Enable SPI for communication
    //
    MAP_SPIEnable(GSPI_BASE);

    // Add a small delay before initialization
    delay(10);
    Adafruit_Init();

    // Clear the display
    fillScreen(BLACK);
    delay(10);
}

// Function to send text over UART
void SendTextOverUART(void)
{
    int i;
    Report("Sending message: %s\r\n", text_buffer);
    
    // Send the text
    for(i = 0; i < text_len; i++) {
        UARTCharPut(UARTA0_BASE, text_buffer[i]);
        
        // Also add to our received buffer to display locally what we're sending
        if (received_text_len < MAX_TEXT_LEN) {
            received_text[received_text_len++] = text_buffer[i];
            received_text[received_text_len] = '\0';
        }
    }
    
    // Send newline
    UARTCharPut(UARTA0_BASE, '\r');
    UARTCharPut(UARTA0_BASE, '\n');
    
    // Also add newline to display buffer
    if (received_text_len < MAX_TEXT_LEN) {
        received_text[received_text_len++] = ' ';
        received_text[received_text_len] = '\0';
    }
    
    // Clear the buffer after sending
    text_len = 0;
    text_buffer[0] = '\0';
    
    // Reset multi-tap state
    last_multi_key = -1;
    multi_tap_count = 0;
    
    // Update display
    DrawTextBuffer(0);
    
    Report("Text sent via UART and buffer cleared\r\n");
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
    // Initialize UART with proper configuration
    //
    PRCMPeripheralReset(PRCM_UARTA0);
    UARTConfigSetExpClk(
        UARTA0_BASE,
        PRCMPeripheralClockGet(PRCM_UARTA0),
        115200, // Standard baud rate
        (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE)
    );
    
    // Register and enable UART interrupt
    UARTIntRegister(UARTA0_BASE, UARTIntHandler);
    UARTIntEnable(UARTA0_BASE, UART_INT_RX | UART_INT_RT);
    UARTFIFOLevelSet(UARTA0_BASE, UART_FIFO_TX1_8, UART_FIFO_RX1_8);
    UARTEnable(UARTA0_BASE);
    
    // Initialize terminal for Report functions
    InitTerm();
    ClearTerm();
    DisplayBanner("GPIO to UART Demo");
    Report("Press 'd' to toggle debug mode (raw bit output)\r\n");
    
    //
    // Enable the SPI module clock
    //
    MAP_PRCMPeripheralClkEnable(PRCM_GSPI,PRCM_RUN_MODE_CLK);

    //
    // Reset the peripheral
    //
    MAP_PRCMPeripheralReset(PRCM_GSPI);

    MasterMain();

    fillScreen(BLACK);
    
    // Draw the initial dividing line
    drawFastHLine(0, OLED_HEIGHT/2, OLED_WIDTH, WHITE);

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
        // Process any available UART data
        ProcessUARTData();
        
        // Check for multi-tap timeout automatically
        unsigned long now = TimerValueGet(TIMERA0_BASE, TIMER_A);
        if (last_multi_key >= 0) {
            unsigned long delta = (now >= last_tap_ts)
                ? (now - last_tap_ts)
                : (0xFFFFFFFF - last_tap_ts) + now;
            unsigned long elapsed_ms = (delta / 80) / 1000;
            
            if (elapsed_ms >= MULTI_TAP_TIMEOUT_MS) {
                // Auto-commit the character after timeout
                CommitPendingChar();
                DrawTextBuffer(0);
            }
        }
        
        // if a code has been received, then print on OLED display
        if (got_a_code) {
            Report("Processing code: 0x%04X\r\n", cur_pattern);
            got_a_code = 0;  // Reset flag
            
            // map pattern → numeric digit 0–9
            int digit = -1;
            if (cur_pattern == PATTERN_0) digit = 0;
            else if (cur_pattern == PATTERN_1) digit = 1;
            else if (cur_pattern == PATTERN_2) digit = 2;
            else if (cur_pattern == PATTERN_3) digit = 3;
            else if (cur_pattern == PATTERN_4) digit = 4;
            else if (cur_pattern == PATTERN_5) digit = 5;
            else if (cur_pattern == PATTERN_6) digit = 6;
            else if (cur_pattern == PATTERN_7) digit = 7;
            else if (cur_pattern == PATTERN_8) digit = 8;
            else if (cur_pattern == PATTERN_9) digit = 9;

            if (digit >= 0) {
                HandleMultiTap(digit);
            }
            // LAST button -> delete current char
            else if (cur_pattern == PATTERN_L) {
                Report("LAST button pressed - deleting character\r\n");
                
                if (last_multi_key >= 0) {
                    // If there's a character being typed, just clear the multi-tap state
                    Report("Clearing in-progress character\r\n");
                    last_multi_key = -1;
                    multi_tap_count = 0;
                } 
                else if (text_len > 0) {
                    // Otherwise delete the last committed character
                    Report("Deleting last character\r\n");
                    text_len--;
                    text_buffer[text_len] = '\0';
                }
                
                DrawTextBuffer(0);
            }
            // MUTE button -> "send" through UART
            else if (cur_pattern == PATTERN_M) {
                Report("MUTE button pressed - sending text\r\n");
                
                // First commit any pending character
                CommitPendingChar();
                
                SendTextOverUART();
            }
        }
    }
}

//*****************************************************************************
//
// Close the Doxygen group.
//! @}
//
//*****************************************************************************


