//
// timersSAMD.cpp
// Library C++ code
// ----------------------------------
// Developed with embedXcode+
// http://embedXcode.weebly.com
//
// Project 		Lara Data Logger 32 bit SAMD
//
// Created by 	Sage Thayer, 8/25/16 12:53 PM
// 				Sage Thayer
//
// Copyright 	(c) Sage Thayer, 2016
// Licence		<#license#>
//
// See 			timersSAMD.h and ReadMe.txt for references
//



// Library header
#include "timersSAMD.h"

void initTimers() {
    // ----------- WDT --------------------//
    
    //divisor
    GCLK->GENDIV.bit.ID = 0x06;     //general clock 6
    //GCLK->GENDIV.bit.DIV = 32;   //division: will result to ~8s interrupt if EWOFFSET = 0xA
    GCLK->GENDIV.bit.DIV = 128;   //division: will result to ~30s interrupt if EWOFFSET = 0xA
    //GCLK->GENDIV.bit.DIV = 256;   //division: will result to ~1min interrupt if EWOFFSET = 0xA
    
    //set generic clock one to wdt
    GCLK->CLKCTRL.bit.GEN = 0x6;    //general clock 6
    GCLK->CLKCTRL.bit.ID = 0x03;    //WDT clock
    GCLK->CLKCTRL.bit.CLKEN = 1;    // enable the clock
    while (GCLK->STATUS.bit.SYNCBUSY);
    
    //32kHz reference oscillator
    GCLK->GENCTRL.bit.ID = 0x06;    //general clock 6
    GCLK->GENCTRL.bit.SRC = 0x03;      //OSCULP32K
    GCLK->GENCTRL.bit.GENEN = 1;    //enable generator
    while (GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY);
    /*
    
    //---------------- External interrupts clock ------------//
    delay(1000);
    EIC->WAKEUP.bit.WAKEUPEN9 = 1;  //enable interrupt wake up on pin 3
    
    //set up clock for external interrupt module
    GCLK->CLKCTRL.bit.GEN = 0x3;    //general clock 3
    GCLK->CLKCTRL.bit.ID = GCM_EIC;    //EIC clock
    GCLK->CLKCTRL.bit.CLKEN = 1;    // enable the clock
    while (GCLK->STATUS.bit.SYNCBUSY);
    
    //32kHz reference oscillator
    GCLK->GENCTRL.bit.ID = 0x03;    //general clock 3
    GCLK->GENCTRL.bit.SRC = 0x03;      //OSCULP32K
    GCLK->GENCTRL.bit.GENEN = 1;    //enable generator
    while (GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY);
    */
    
    //------------------ Timer 4 ----------------//
    /*
    //set up clock for timer
    GCLK->CLKCTRL.bit.GEN = 0x6;    //general clock 6
    GCLK->CLKCTRL.bit.ID = 0x1D;    //TC3 clock ID
    GCLK->CLKCTRL.bit.CLKEN = 1;    // enable the clock
    
    //32kHz reference oscillator
    GCLK->GENCTRL.bit.ID = 0x06;    //general clock 6
    GCLK->GENCTRL.bit.SRC = 0x03;      //OCULP32K
    GCLK->GENCTRL.bit.GENEN = 1;    //enable generator
    
    //set up a timer
    TC4->COUNT8.CTRLA.bit.ENABLE = 0;   //disable the timer
    TC4->COUNT8.CTRLA.bit.WAVEGEN = 0;  //normal frequency mode
    //TC4->COUNT8.PER.reg = 0xFF;         //top is 255
    //TC3->COUNT8.CTRLBSET.bit.ONESHOT = 1;
    
    NVIC_EnableIRQ(TC4_IRQn);           //enable interrupt
    TC4->COUNT8.INTENSET.bit.OVF = 1;   //overflow interrupt
    TC4->COUNT8.CTRLA.bit.ENABLE = 1;   //enable the timer
    SerialUSB.println("did we make it?");
*/
}


// Code
void wdtInit() {
    /*GCLK->CLKCTRL.bit.CLKEN = 0;    // disable the clock
     while (GCLK->STATUS.bit.SYNCBUSY);
     */
    
    //Config internal interrupt for wdt
    NVIC_EnableIRQ(WDT_IRQn);     //bit 2 turns the WDT interrupt to pending
    
    //config WDT
    WDT->CONFIG.bit.PER = 0xB;  // set prescaler to 16384 cycles
    WDT->EWCTRL.bit.EWOFFSET = 0xA; // early warning interrupt at 8192 cycles
    WDT->INTENSET.bit.EW = 1;       // enable the early warning interrupt
    
    WDT->CTRL.bit.ENABLE = 1;   //enable the timer
}

void wdtClear() {
    WDT->CLEAR.bit.CLEAR = 0xA5;    // clear the timer
}