/*
 * File:   timer.c
 * Author: joudeh
 *
 * Created on July 19, 2022, 8:15 PM
 */


#include <xc.h>
#include <time.h>
unsigned short count_t1=0;
unsigned short count_t2=0;

#define STARTVALUE  40536
unsigned char ReceivedChar ;
void reloadTimer1(void)
{  
    TMR1H = (unsigned char) ((STARTVALUE >>  8) & 0x00FF);
    TMR1L =  (unsigned char)(STARTVALUE & 0x00FF );   
}
void Timer1_isr(unsigned int f)
{
    PIR1bits.TMR1IF = 0;// Must be cleared by software
    count_t1++;
    count_t2++;
    if(count_t1 >= f*20) { 
        PORTBbits.RB1 = !PORTBbits.RB1; //Toggle RD1 every 10*f(1,2,3) seconds
        count_t1=0;
    }
       if(count_t1 >= 5) { 
    PORTBbits.RB2 = !PORTBbits.RB2; //Toggle RD0 every 1 second
        count_t2=0;
    }
    reloadTimer1();
  
}
char RX_isr(void)
{
    CLRWDT();
   // PIR1bits.RCIF; not needed , does nothing
    PORTBbits.RB3 = !PORTBbits.RB3; //Toggle RD3 when a character is received
    ReceivedChar  = RCREG; 
    while (!TXSTAbits.TRMT) {CLRWDT();}//echo the character back
    return ReceivedChar;
}


void initTimers01(void) {
    count_t1=0;
    //setupPorts();
    
    INTCON = 0; // disable interrupts first, then enable the ones u want
    PORTD = 0;
    T1CON = 0x30 ;// pre-scalar 8 off , RD16 = 0, TMRON = 0
    
    //T1CON =0;
    //T1CONbits.T1CKPS =0x3
    reloadTimer1(); //TR1Value = 3036, 0.5 second
    RCONbits.IPEN = 0; // Disable Interrupt priority , All are high
    PIE1 = 0;
    
    PIE1bits.TMR1IE = 1; // Enable Timer1 Interrupt
    PIE1bits.RCIE = 1;   // Enable Serial Receive Interrupts
    PIR1 =0; //clear interrupt flags
    //PIE1bits.TXIE =1;
    PIE2 =0; // all interrupts in PIE are disabled
      
    INTCONbits.GIEH = 1;  // enable global interrupt bits
    INTCONbits.GIEL = 1;  // enable global interrupt bits
    T1CONbits.TMR1ON = 1;//start timer
}

