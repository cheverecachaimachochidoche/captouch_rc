//******************************************************************************
// structure.c
// 
// RO_PINOSC_TA0_WDTp_One_Button example
// Touch the middle element to turn on/off the center button LED
// RO method capactiance measurement using PinOsc IO, TimerA0, and WDT+
//
//******************************************************************************
#include "msp430g2553.h"
#include "structure.h"

// Middle Element (P2.5)
const struct Element middle_element = {

              .inputPxselRegister = (uint8_t *)&P2SEL,  
              .inputPxsel2Register = (uint8_t *)&P2SEL2,  
              .inputBits = BIT5,
              // When using an abstracted function to measure the element
              // the 100*(maxResponse - threshold) < 0xFFFF
              // ie maxResponse - threshold < 655
              .maxResponse = 450+655,		
              .threshold = 450
};

//*** CAP TOUCH HANDLER *******************************************************/
// This defines the grouping of sensors, the method to measure change in
// capacitance, and the function of the group

// One Button Sensor
const struct Sensor one_button =
               { 
                  .halDefinition = RO_PINOSC_TA0_WDTp,	// Sensing Method
                  .numElements = 1,						// # of Elements
                  .baseOffset = 0,						// First element index = 0
                  // Pointer to elements
                  .arrayPtr[0] = &middle_element,  		// point to middle element
                  // Timer Information
                  .measGateSource= GATE_WDT_ACLK,     //  0->SMCLK, 1-> ACLK
                  .accumulationCycles= WDTp_GATE_64   //64 - Default                                                
               };
