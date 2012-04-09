//Main file where is the capacitive touch main code and the CC100L main code

#include "include.h"
#include "CTS_Layer.h"


extern char paTable[];
extern char paTableLen;
char txBuffer[12];
char rxBuffer[12];
unsigned int i = 0;

//+++++++++++++++++++++++++++++++++++capacitive touch code++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#define DELAY 5000 		// Timer delay timeout count - 5000*0.1msec = 500 msec

#ifdef ELEMENT_CHARACTERIZATION_MODE
// Delta Counts returned from the API function for the sensor during characterization
unsigned int dCnt;
#endif
  //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void sleep(unsigned int time)
{
    TA0CCR0 = time;
    TA0CTL = TASSEL_1+MC_1+TACLR;
    TA0CCTL0 &= ~CCIFG;
    TA0CCTL0 |= CCIE; 
    __bis_SR_register(LPM3_bits+GIE);
    __no_operation();
}

void main (void)
{
  WDTCTL = WDTPW + WDTHOLD;                 // Stop WDT

  // 5ms delay to compensate for time to startup between MSP430 and CC1100/2500
  __delay_cycles(5000); 
  uartInit();
  TI_CC_SPISetup();                         // Initialize SPI port
  TI_CC_PowerupResetCCxxxx();               // Reset CCxxxx
  writeRFSettings();                        // Write RF settings to config reg
  TI_CC_SPIWriteBurstReg(TI_CCxxx0_PATABLE, paTable, paTableLen);//Write PATABLE

  // Configure ports -- switch inputs, LEDs, GDO0 to RX packet info from CCxxxx
  TI_CC_SW_PxREN = TI_CC_SW1;               // Enable Pull up resistor
  TI_CC_SW_PxOUT = TI_CC_SW1;               // Enable pull up resistor
  TI_CC_SW_PxIES |= TI_CC_SW1;               // Int on falling edge
  TI_CC_SW_PxIFG &= ~(TI_CC_SW1);           // Clr flags
  TI_CC_SW_PxIE |= TI_CC_SW1;                // Activate interrupt enables
   TI_CC_SW_PxDIR = 0x1;
  TI_CC_LED_PxOUT &= ~(TI_CC_LED1); // Outputs = 0
  TI_CC_LED_PxDIR |= TI_CC_LED1;// LED Direction to Outputs
  TI_CC_GDO0_PxIES |= TI_CC_GDO0_PIN;       // Int on falling edge (end of pkt)
  TI_CC_GDO0_PxIFG &= ~TI_CC_GDO0_PIN;      // Clear flag
  TI_CC_GDO0_PxIE |= TI_CC_GDO0_PIN;        // Enable int on end of packet
  TI_CC_SPIStrobe(TI_CCxxx0_SRX);           // Initialize CCxxxx in RX mode.
                                            // When a pkt is received, it will
                                            // signal on GDO0 and wake CPU  

//+++++++++++++++++++++++++++++++++++capacitive touch code++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

   WDTCTL = WDTPW + WDTHOLD;             // Stop watchdog timer
  BCSCTL1 = CALBC1_1MHZ;                // Set DCO to 1, 8, 12 or 16MHz
  DCOCTL = CALDCO_1MHZ;
  BCSCTL2 |= DIVS_2;                    // divide SMCLK by 4 for 250khz      
  BCSCTL3 |= LFXT1S_2;                  // LFXT1 = VLO 
  P2SEL &= ~(BIT6 + BIT7);				// Configure XIN (P2.6) and XOUT (P2.7) to GPIO
  P2OUT = 0x00;							// Drive all Port 2 pins low
  P2DIR = 0xFF;							// Configure all Port 2 pins outputs
  TI_CAPT_Init_Baseline(&one_button);
  TI_CAPT_Update_Baseline(&one_button,5);  
  
  //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  
  
  
  __bis_SR_register(LPM0_bits + GIE);       // Enter LPM0, interrupts enabled


while (1)
  {
  
	#ifdef ELEMENT_CHARACTERIZATION_MODE
	TI_CAPT_Custom(&one_button,&dCnt);
	__no_operation();
		#endif
	  	
	#ifndef ELEMENT_CHARACTERIZATION_MODE	 

        sleep(DELAY);		
        #endif
}
}



// The ISR assumes the interrupt came from a pressed button
#pragma vector=PORT1_VECTOR
__interrupt void Port1_ISR (void)
{
   // If Switch was pressed
  if(TI_CC_SW_PxIFG & TI_CC_SW1) {
    if(TI_CAPT_Button(&one_button)) {
    
    // Build packet
    txBuffer[0] = 11;                        // Packet length
    txBuffer[1] = 0x01;                     // Packet address
    txBuffer[2] = TI_CC_LED1;
    txBuffer[3] = 0x32;
    txBuffer[4] = 0x33;
    txBuffer[5] = 0x34;
    txBuffer[6] = 0x35;
    txBuffer[7] = 0x36;
    txBuffer[8] = 0x37;
    txBuffer[9] = 0x38;
    txBuffer[10] = 0x39;
    txBuffer[11] = 0x40;
    
       RFSendPacket(txBuffer, 12);              // Send value over RF
    __delay_cycles(5000);                   // Switch debounce
    } }
  TI_CC_SW_PxIFG &= ~(TI_CC_SW1);           // Clr flag that caused int
 }





// The ISR assumes the interrupt came from GDO0. GDO0 fires indicating that
// CCxxxx received a packet
#pragma vector=PORT2_VECTOR
__interrupt void Port2_ISR(void)
{
    // if GDO fired
  if(TI_CC_GDO0_PxIFG & TI_CC_GDO0_PIN)
  {
    char len=11;                            // Len of pkt to be RXed (only addr
                                            // plus data; size byte not incl b/c
                                            // stripped away within RX function)
    if (RFReceivePacket(rxBuffer,&len))  
    {   
        // Fetch packet from CCxxxx
        TI_CC_LED_PxOUT ^= rxBuffer[1];         // Toggle LEDs according to pkt data
           
        if (rxBuffer[1] == 0xFF) 
        {
          puts("RX ACK\r\n");
        }
        else 
        {
          __delay_cycles(500000);
          // Send ACK
          // Build packet
          txBuffer[0] = 3;                        // Packet length
          txBuffer[1] = 0x01;                     // Packet address
          txBuffer[2] = 0xFF;
          txBuffer[3] = 0x00;

          RFSendPacket(txBuffer, 4);              // Send value over RF
        }}}
  TI_CC_GDO0_PxIFG &= ~TI_CC_GDO0_PIN;      // After pkt RX, this flag is set.
}


//+++++++++++++++++++++++++++++++++++capacitive touch code++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#pragma vector=TIMER0_A0_VECTOR
__interrupt void ISR_Timer0_A0(void)
{
  TA0CTL &= ~(MC_1);
  TA0CCTL0 &= ~(CCIE);
  __bic_SR_register_on_exit(LPM3_bits+GIE);
}
  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++