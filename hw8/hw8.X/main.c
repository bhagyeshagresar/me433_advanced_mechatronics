#include<xc.h>           // processor SFR definitions
#include<sys/attribs.h>  // __ISR macro
#define DELAYTIME 12000000
#define NU32_DESIRED_BAUD 230400    // Baudrate for RS232
#include <stdio.h>
#include <math.h>
#include "i2c_master.h"
#include "ssd1306.h"
#include "font.h"



// DEVCFG0
#pragma config DEBUG = OFF // disable debugging
#pragma config JTAGEN = OFF // disable jtag
#pragma config ICESEL = ICS_PGx1 // use PGED1 and PGEC1
#pragma config PWP = OFF // disable flash write protect
#pragma config BWP = OFF // disable boot write protect
#pragma config CP = OFF // disable code protect

// DEVCFG1
#pragma config FNOSC = FRCPLL // use fast frc oscillator with pll
#pragma config FSOSCEN = OFF // disable secondary oscillator
#pragma config IESO = OFF // disable switching clocks
#pragma config POSCMOD = OFF // primary osc disabled
#pragma config OSCIOFNC = OFF // disable clock output
#pragma config FPBDIV = DIV_1 // divide sysclk freq by 1 for peripheral bus clock
#pragma config FCKSM = CSDCMD // disable clock switch and FSCM
#pragma config WDTPS = PS1 // use largest wdt value
#pragma config WINDIS = OFF // use non-window mode wdt
#pragma config FWDTEN = OFF // wdt disabled
#pragma config FWDTWINSZ = WINSZ_25 // wdt window at 25%

// DEVCFG2 - get the sysclk clock to 48MHz from the 8MHz fast rc internal oscillator
#pragma config FPLLIDIV = DIV_2 // divide input clock to be in range 4-5MHz //4
#pragma config FPLLMUL = MUL_24 // multiply clock after FPLLIDIV //80
#pragma config FPLLODIV = DIV_2 // divide clock after FPLLMUL to get 48MHz

// DEVCFG3
#pragma config USERID = 0000 // some 16bit userid, doesn't matter what
#pragma config PMDL1WAY = OFF // allow multiple reconfigurations
#pragma config IOL1WAY = OFF // allow multiple reconfigurations


void readUART1(char * string, int maxLength);
void writeUART1(const char * string);
void initSPI();
unsigned char spi_io(unsigned char o);
unsigned short make16(unsigned char a_or_b, unsigned char v);
void delay();
void write_fn(unsigned char some_register, unsigned char data);
unsigned char read_fn(unsigned char some_register2);
void draw_char_fn(char character, unsigned char x, unsigned char y);
void draw_string(unsigned char x, unsigned char y, char *message_array);
void draw_fps(unsigned char x, unsigned char y, int fps);

int main() {

    __builtin_disable_interrupts(); // disable interrupts while initializing things

    // set the CP0 CONFIG register to indicate that kseg0 is cacheable (0x3)
    __builtin_mtc0(_CP0_CONFIG, _CP0_CONFIG_SELECT, 0xa4210583);

    // 0 data RAM access wait states
    BMXCONbits.BMXWSDRM = 0x0;

    // enable multi vector interrupts
    INTCONbits.MVEC = 0x1;

    // disable JTAG to get pins back
    DDPCONbits.JTAGEN = 0;

    // do your TRIS and LAT commands here
    i2c_master_setup();
    ssd1306_setup();


    __builtin_enable_interrupts();
    TRISBbits.TRISB4 = 1; //INITIALISE B4 as input
    TRISAbits.TRISA4 = 0; //Initialise A4 as output
    LATAbits.LATA4 = 0; //initially off


    U1MODEbits.BRGH = 0; // set baud to NU32_DESIRED_BAUD
    U1BRG = ((48000000 / NU32_DESIRED_BAUD) / 16) - 1;

    // 8 bit, no parity bit, and 1 stop bit (8N1 setup)
    U1MODEbits.PDSEL = 0;
    U1MODEbits.STSEL = 0;

    // configure TX & RX pins as output & input pins
    U1STAbits.UTXEN = 1;
    U1STAbits.URXEN = 1;
    // configure hardware flow control using RTS and CTS
    U1MODEbits.UEN = 2;

    U1RXRbits.U1RXR = 0b0000; // Set A2 to U1RX
    RPB3Rbits.RPB3R = 0b0001; // Set B3 to U1TX


    // enable the uart
    U1MODEbits.ON = 1;


    char m_message[100];
    char fps_message[100];
//    char m[100]; I2C HW functions
//    write_fn(0b00000000, 0b01111111); //set trisbits
//    write_fn(0b00001010, 0b10000000); //set loat

//    int ptr = &m_message;
//    int var = 0;


    sprintf(m_message, "Hello world");
    float fps = 0.0;
//    sprintf(fps_message, "fps: ");
    while(1){
        
        _CP0_SET_COUNT(0);
//        while (_CP0_GET_COUNT() < DELAYTIME) {
//             LATAbits.LATA4 = 1;
//         }
//         _CP0_SET_COUNT(0);
//
//        while (_CP0_GET_COUNT() < DELAYTIME) {
//             LATAbits.LATA4 = 0;
//         }
//        _CP0_SET_COUNT(0);
//
//        while (_CP0_GET_COUNT() < DELAYTIME) {
//             LATAbits.LATA4 = 1;
//         }
//         _CP0_SET_COUNT(0);
//         
//         while (_CP0_GET_COUNT() < DELAYTIME) {
//             LATAbits.LATA4 = 0;
//         }
         
         

      
//         ssd1306_drawPixel(5, 5, 1);
//         ssd1306_update();

//         draw_char_fn('a', 5, 5);
//        j = j + 1;
         draw_string(5, 5, &m_message);
         ssd1306_update();
         fps = (24000000.0/_CP0_GET_COUNT());
         sprintf(fps_message, "fps = %f", fps);
//         sprintf(fps_message, "fps = %d", j);

         draw_string(30, 25, &fps_message);
//
//         
         
         
       
         
        
       
    }


}

void writeUART1(const char * string) {
    while (*string != '\0') {
        while (U1STAbits.UTXBF) {
            ; // wait until tx buffer isn't full
        }
        U1TXREG = *string;
        ++string;
    }
}

void readUART1(char * message, int maxLength) {
    char data = 0;
    int complete = 0, num_bytes = 0;
    // loop until you get a '\r' or '\n'
    while (!complete) {
        if (U1STAbits.URXDA) { // if data is available
            data = U1RXREG; // read the data
            if ((data == '\n') || (data == '\r')) {
                complete = 1;
            } else {
                message[num_bytes] = data;
                ++num_bytes;
                // roll over if the array is too small
                if (num_bytes >= maxLength) {
                    num_bytes = 0;
                }
            }
        }
    }
    // end the string
    message[num_bytes] = '\0';
}

void delay(){
    int i;
    for(i = 0; i < DELAYTIME; i++){
        ;
    }
}

void initSPI() {
    // Pin B14 has to be SCK1
    // Turn of analog pins
    ANSELA = 0;
    // Make an output pin for CS
    TRISAbits.TRISA0 = 0;
    LATAbits.LATA0 = 1;
    //...
    //...
    // Set SDO1
    RPA1Rbits.RPA1R = 0b0011;

    //...
    // Set SDI1
    SDI1Rbits.SDI1R = 0b0001;
    //...

    // setup SPI1
    SPI1CON = 0; // turn off the spi module and reset it
    SPI1BUF; // clear the rx buffer by reading from it
    SPI1BRG = 1000; // 1000 for 24kHz, 1 for 12MHz; // baud rate to 10 MHz [SPI1BRG = (48000000/(2*desired))-1]
    SPI1STATbits.SPIROV = 0; // clear the overflow bit
    SPI1CONbits.CKE = 1; // data changes when clock goes from hi to lo (since CKP is 0)
    SPI1CONbits.MSTEN = 1; // master operation
    SPI1CONbits.ON = 1; // turn on spi 
}

unsigned char spi_io(unsigned char o) {
    SPI1BUF = o;
    while (!SPI1STATbits.SPIRBF) { // wait to receive the byte
        ;
    }
    return SPI1BUF;
}

unsigned short make16(unsigned char a_or_b, unsigned char v) {
    unsigned short s;
    s = 0;
    s = s | (a_or_b << 15);
    s = s | (0b111 << 12);
    s = s | (v << 4);

    return s;
}


void write_fn(unsigned char some_register, unsigned char data){
    
    //write step
    i2c_master_start();
    i2c_master_send(0b01000000);
    i2c_master_send(some_register);
    i2c_master_send(data);
    i2c_master_stop();
    
}


unsigned char read_fn(unsigned char some_register2){
    
    i2c_master_start(); //add start bit
    i2c_master_send(0b01000000); //add write address
    i2c_master_send(some_register2);
    i2c_master_restart();
    i2c_master_send(0b01000001); //add read address
    unsigned char result = i2c_master_recv();
    i2c_master_ack(1);
    i2c_master_stop();
    
    return result;
    
}


void draw_char_fn(char character, unsigned char x, unsigned char y){
    
    for (int i = 0; i < 5; i++){
        char col = ASCII[character - 32][i];
        for (int j = 0; j < 8; j++){
            int on_or_off = (col >> j) & 0b1;
            ssd1306_drawPixel(x+i, y+j, on_or_off);
//            ssd1306_update();
        
        }
       
        
    }
  
}

void draw_string(unsigned char x, unsigned char y, char *message_array){
    int i = 0;
    while(message_array[i] != 0){
        draw_char_fn(message_array[i], x+(i*5), y);
        i = i + 1;
    }
    


}



