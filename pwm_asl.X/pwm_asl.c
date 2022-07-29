/*
 * File:   pwm_asl.c
 * Author: raed
 * PWM + ADC + SERIAL + LCD
 * Created on March 30, 2019, 1:05 PM
 * LCD is set to work on the simulator, must be fixed to work with real
 */


#define _XTAL_FREQ   4000000UL     // needed for the delays, set to 4 MH= your crystal frequency
// CONFIG1H
#pragma config OSC = XT         // Oscillator Selection bits (XT oscillator)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor disabled)
#pragma config IESO = OFF       // Internal/External Oscillator Switchover bit (Oscillator Switchover mode disabled)

// CONFIG2L
#pragma config PWRT = OFF       // Power-up Timer Enable bit (PWRT disabled)
#pragma config BOREN = SBORDIS  // Brown-out Reset Enable bits (Brown-out Reset enabled in hardware only (SBOREN is disabled))
#pragma config BORV = 3         // Brown Out Reset Voltage bits (Minimum setting)

// CONFIG2H
#pragma config WDT = ON         // Watchdog Timer Enable bit (WDT enabled)
#pragma config WDTPS = 32768    // Watchdog Timer Postscale Select bits (1:32768)

// CONFIG3H
#pragma config CCP2MX = PORTC   // CCP2 MUX bit (CCP2 input/output is multiplexed with RC1)
#pragma config PBADEN = ON      // PORTB A/D Enable bit (PORTB<4:0> pins are configured as analog input channels on Reset)
#pragma config LPT1OSC = OFF    // Low-Power Timer1 Oscillator Enable bit (Timer1 configured for higher power operation)
#pragma config MCLRE = ON       // MCLR Pin Enable bit (MCLR pin enabled; RE3 input pin disabled)

// CONFIG4L
#pragma config STVREN = ON      // Stack Full/Underflow Reset Enable bit (Stack full/underflow will cause Reset)
#pragma config LVP = ON         // Single-Supply ICSP Enable bit (Single-Supply ICSP enabled)
#pragma config XINST = OFF      // Extended Instruction Set Enable bit (Instruction set extension and Indexed Addressing mode disabled (Legacy mode))

// CONFIG5L
#pragma config CP0 = OFF        // Code Protection bit (Block 0 (000800-003FFFh) not code-protected)
#pragma config CP1 = OFF        // Code Protection bit (Block 1 (004000-007FFFh) not code-protected)
#pragma config CP2 = OFF        // Code Protection bit (Block 2 (008000-00BFFFh) not code-protected)
#pragma config CP3 = OFF        // Code Protection bit (Block 3 (00C000-00FFFFh) not code-protected)

// CONFIG5H
#pragma config CPB = OFF        // Boot Block Code Protection bit (Boot block (000000-0007FFh) not code-protected)
#pragma config CPD = OFF        // Data EEPROM Code Protection bit (Data EEPROM not code-protected)

// CONFIG6L
#pragma config WRT0 = OFF       // Write Protection bit (Block 0 (000800-003FFFh) not write-protected)
#pragma config WRT1 = OFF       // Write Protection bit (Block 1 (004000-007FFFh) not write-protected)
#pragma config WRT2 = OFF       // Write Protection bit (Block 2 (008000-00BFFFh) not write-protected)
#pragma config WRT3 = OFF       // Write Protection bit (Block 3 (00C000-00FFFFh) not write-protected)

// CONFIG6H
#pragma config WRTC = OFF       // Configuration Register Write Protection bit (Configuration registers (300000-3000FFh) not write-protected)
#pragma config WRTB = OFF       // Boot Block Write Protection bit (Boot Block (000000-0007FFh) not write-protected)
#pragma config WRTD = OFF       // Data EEPROM Write Protection bit (Data EEPROM not write-protected)

// CONFIG7L
#pragma config EBTR0 = OFF      // Table Read Protection bit (Block 0 (000800-003FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR1 = OFF      // Table Read Protection bit (Block 1 (004000-007FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR2 = OFF      // Table Read Protection bit (Block 2 (008000-00BFFFh) not protected from table reads executed in other blocks)
#pragma config EBTR3 = OFF      // Table Read Protection bit (Block 3 (00C000-00FFFFh) not protected from table reads executed in other blocks)

// CONFIG7H
#pragma config EBTRB = OFF      // Boot Block Table Read Protection bit (Boot Block (000000-0007FFh) not protected from table reads executed in other blocks)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.
#include <xc.h>
#include <stdio.h>
#include <math.h>

#include "my_ser.h"
#include "my_adc.h"
#include "my_pwm.h"
#include "lcd_x8.h"
#include "timer.h"
#include "display7s.h"
//function prototypes
#define STARTVALUE  40536
  unsigned  int p2;
  unsigned char test0=0;
  unsigned int f=0;
 unsigned char strr[]={'c','c','c'};
 unsigned int size=0;
unsigned int RPS_count = 0;
         float tes=33.45;

void setupPorts(void) {
    ADCON0 = 0;
    ADCON1 = 0b00001100; //3 analog channels, change this according to your application
    TRISB = 0xF1; // all pushbuttons are inputs
    TRISC = 0x80; // RX input , others output
    TRISA = 0xCF; // All inputs
    TRISD = 0x00; // All outputs
    TRISE = 0x00; // All outputs
}
void __interrupt(high_priority) highIsr (void)
//void interrupt high_priority highIsr(void)
{
    if(PIR1bits.TMR1IF) Timer1_isr(p2%3);
    else if(PIR1bits.RCIF){
        strr[size++]=RX_isr();
        if(size==3){
            size=0;
        }
        examine();
    }
    
      // send_string_no_lib(strr);

   // size+=1;
    
    //else if(PIR1bits.TXIF &&  PIE1bits.TXIE ) TX_isr();
   
}
void examine(void){
    if(strr[0]=='o'){
        if(strr[1]=='n'){
            size=0;
            f= 1;
            strr[0]='\n';
            strr[1]='\n';
            strr[2]='\n';
        }else if(strr[1]=='f'){
            if(strr[2]=='f'){
                size=0;
                f= 0;
                 strr[0]='\n';
            strr[1]='\n';
            strr[2]='\n';
            }
        }
    }
          
}

// used also for measuring speed
//void interrupt high_priority highIsr(void)//old syntax


void main(void) {

    size=0;
    //ADCON1 = 0b00001100; //3 analog channels, change this according to your application
    char Buffer[32]; // for sprintf
    float AN[3];     // To store the voltages of AN0, AN1, AN2
    int raw_val;
    float threshold;
    int flag=0;
    unsigned char channel;
    float voltage;
    setupPorts();
    setupSerial();
    lcd_init();
    init_adc_no_lib();
    init_pwm1();
    //PORTCbits.RC4 =1;
    PORTCbits.RC5 = 1;
    lcd_putc('\f'); //clears the display
    unsigned char RecvedChar = 0;
    unsigned char SendToSerial = 0;
     initTimers01();   // These will be used to measure the speed
     TRISCbits.RC0 = 1; //Timer1 clock
    while (!f);
    while(1){
            //send_string_no_lib((unsigned char *) "\r\rReading AN0, AN1, AN2\r\r");
    if (!f){
         lcd_putc('\f');//clears the display
         PORTD=0;
         while(!f);
    }

        //CLRWDT(); // no need for this inside the delay below
        PORTCbits.RC5 = !PORTCbits.RC5;
        //delay_ms(20); //read ADC AN0,AN1, AN2 every 2 seconds
        for (channel = 0; channel < 3; channel++) {
            // read the adc voltage
            voltage = read_adc_voltage((unsigned char) channel);
            AN[channel] = voltage; // store in array AN0--AN2
            if (is_byte_available()) { // Read serial, if receive S
                RecvedChar = read_byte_no_lib(); // Then start sending to serial
                if (RecvedChar == 'S') SendToSerial = 1;
                else if (RecvedChar == 'N') SendToSerial = 0;// Stop sending to serialif N is recived
                else {
                    /*No Change */
                }
            }
            if (SendToSerial) {
                // If Sending to Serial ( after receiving S, send values)
                sprintf(Buffer, "V%d:%6.2f volt\r", channel, voltage);
                //send_string_no_lib(Buffer);
            }
        }

        raw_val = read_adc_raw_no_lib(0); // read raw value for POT1 
        set_pwm1_raw(raw_val);  // set the Pwm to that value 0--1023
        if(!PORTBbits.RB0){
            flag=~flag;// toggle the display of the temperature between C and F
        }
        if(!flag){
            lcd_gotoxy(1, 1);
            sprintf(Buffer, "temp=%3.2f C", AN[2]*100);//Celsius
        }else{
         lcd_gotoxy(1, 1);
            sprintf(Buffer, "temp=%3.2f F", (AN[2]*100)+33.8);//Fahrenheit
        }
        lcd_puts(Buffer);
        lcd_gotoxy(1, 2);
        p2=read_adc_raw_no_lib(1);
        sprintf(Buffer, "level of speed=%1.1f", floor(AN[1]));//level of frequency
        lcd_puts(Buffer);
        threshold = read_adc_temp(0);
        lcd_gotoxy(1, 3);
         sprintf(Buffer, "thresh=%1.1f", threshold);//threshold
        lcd_puts(Buffer);
        if (AN[2]*100>threshold) {
            send_string_no_lib((unsigned char *) "Warning!");
        }
         lcd_gotoxy(1, 4);
         sprintf(Buffer, "thresh=%1.1f", AN[0]);//test
             lcd_puts(Buffer);
//            
//        lcd_gotoxy(2, 4);
//         sprintf(Buffer, "%c", strr[1]);//test
//        lcd_puts(Buffer);
//         lcd_gotoxy(3, 4);
//         sprintf(Buffer, "%2.3f", floor(tes/10));//test
//        lcd_puts(Buffer);
//     lcd_gotoxy(6, 4);
//         sprintf(Buffer, "%d", f);//test
//        lcd_puts(Buffer);
//        PORTA=0x20;
//        PORTD=display7seg(test0);
//        delay_ms(200);
        
                
 PORTD=display7seg((int)(AN[0]*10)%10);
            PORTA=0x20;
//           PORTAbits.RA5=1;

                 delay_ms(300);
//                            PORTAbits.RA5=0;
//           PORTAbits.RA4=1;

            PORTD=display7seg(floor(AN[0]));
           PORTA=0x10;

            delay_ms(300);
//           PORTAbits.RA4=0;

//           PORTA=0x08;
//                 PORTD=display7seg(floor(tes/10));
//
//           PORTA=0x04;
//        
//                 PORTD=display7seg(floor(tes/10));

    }
}
