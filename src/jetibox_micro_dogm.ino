/* 

// TODO
// POINTER Variant - to improve memory usage. 
// - simpleTextExtraction using pointers for direct variable manipulation instead of writing globals
// - STILL Terst change to #define instaed of variables.
//
// Open Tests/Changes
// - #define replace variables
// - printJBText subroutine now using pointers to chars
// - printJBText subroutine now used for init screen as well as standard screen display to save memory
// - replaced init screen display by initializing lines prior
// - removed struct
// - subroutinge now returning 0 or 1. Does it make sense?

Jetibox Micro by André
This program allows to rebuild a Jetibox with an Ardunio and external 16x2 matrix display
Intention is to built a smaller version than Jetibox Mini, allowing to remove teh analog voltage meter
of the Graupner remote control with a display ( I used EA DOGM162L-A - reflective, very small). Display
is connected via Hardware SPI, while I chose to use Hardware UART from Atmega 328po. 

The program reads the data stream from Jeti sensors/TU-modules and filters for JetiBox text to be 
displayed. It allows to display real live data but also access the Jetibox menu to configure
e.g. sensors.

The system is based on the Jeti Telemtry protocol. In general the Jeti devices can send one of the following
messages:
- alarm protocol message, followed by simple text
- EX protocol message
-- data message, followed by simple text
-- text message, followed by simple text
- simple text message
We are interested in simple text only. To identify the messages, there are clear message separators
defined. To fully identify them, the evaluation of the 9th bit in Jeti UART configuration must be used.
Jeti uses 1 start bit, 9 data bits, odd parity and 2 stop bits (Serial_9O2). This requires to use a
UART library which supports 9bit.

Hardware notes:
You need to connect also 4 buttons which will allow you to navigate in Jeti menus. The debouncing is done in software.
Display is connected in 4-wire mode to make soldering easier. 
  - Arduino Mini Pro 5V 16MHz board used
  - buttons will connect the according pins to ground when pressed. Pullup not required, internal pullup is being used
  - TXD and RXD line need to be connected with 2,2k resistor for Jeti UART setup support
  - Sensor/TU-module to be connected to Arduino Gnd, VCC/Raw (5V) and RXD
  - EA DOGM162L-A display, reflective, no backlight needed

Pin usage:
  Usage.         Atmega Mini Pro 328p board
  -----------------------------------------
  Button right:  Pin A0   \
  Button uo:     Pin A1    \
  Button down:   Pin A2     | 5-pin connector to buttons
  Button left:   Pin A3.   /
  Button Gnd:    Gnd      /

  LCD CS:        Pin 10   \
  LCD CLK:       Pin 13    \
  LCD SI:        Pin 11     \ 6-pin connector to LCD
  LCD RS:        Pin  9     /
  LCD Gnd:       Gnd       /
  LCD VCC:       Vcc      / 

  Sensor Ground: Gnd      \
  Sensor VCC:    Vcc       | 3-pin connector to TU2-Modul (Servo-connector)
  Sensor TXD:    RXD      /

  HW TXD:        RXD via 2,2kOhm resistor

Additional protection resistors can be added between display and Microcontroller. For me system ran stable,
so I decided to not put them in.
*/

#include <Arduino.h>
#include "dogm_7036.h"
#include <SPI.h>              // Needed? Should be part of Arduino.h? Test without todo...
#include <avr/interrupt.h>    // Needed? Should be part of Arduino.h? Test without todo...

// pin definitions buttons and debounce config values
#define PIN_BUTTON_RIGHT A0
#define PIN_BUTTON_UP A1
#define PIN_BUTTON_DOWN A2
#define PIN_BUTTON_LEFT A3
#define DEBOUNCE_TIME 30
// pin definitions LCD - SPI (use HW SPI)
#define LCD_PIN_SI 10
#define LCD_PIN_RS 8
#define STARTUPSCREEN_SHOW 2000
// Important protocol bytes to check for
#define SIMPLETEXT_START 0xFE
#define SIMPLETEXT_END 0xFF

// DO NOT CHANGE BELOW
// global variables to be defined
volatile uint16_t JetiBoxButtons = 0x00F0;  // Byte to send to sensor/transmitter to navigate through Jetibox menu. Need to be 2 bytes to capture data bit #9 as well
int lastpressed_buttons = 0;  // required for software debouncing
// initiate the display lines with start screen values
char line1[17] = "  JetiBox-Micro ";
char line2[17] = "  by Andre Kuhn ";

// declaration of required functions
void printScreen(char *line1, char *line2);
void simpleTextExtraction();

dogm_7036 lcd;

void setup() {
  // configure pins
  pinMode(PIN_BUTTON_UP, INPUT_PULLUP);     // up
  pinMode(PIN_BUTTON_DOWN, INPUT_PULLUP);   // down
  pinMode(PIN_BUTTON_LEFT, INPUT_PULLUP);   // left
  pinMode(PIN_BUTTON_RIGHT, INPUT_PULLUP);  // right

  // configure interrupts 
  cli();                      // turn interrupts off, while changing
  //  PCICR |= 0b00000001;    // Enable Port B Pin Change Interrupts. Bit0=B - PCINT 0-7, Bit1=C - PCINT 8-14, Bit2=D - PCINT 16-23
  PCICR |= 0b00000010;        // Enable Port C Pin Change Interrupts. Bit0=B - PCINT 0-7, Bit1=C - PCINT 8-14, Bit2=D - PCINT 16-23
  //  PCMSK0 |= 0b00001111;   // Enable pin interrupts: PCINT0  -> Port B
  PCMSK1 |= 0b00001111;       // Enable Pin interrupts: PCINT8-11 -> Port C
  //  PCMSK2 |= 0b10000000;   // PCINT11 -> Port D
  sei();                      // turn interrupts on after changing their configuration

  // initialize LCD
  lcd.initialize(LCD_PIN_SI, 0, 0, LCD_PIN_RS, 4, 1, DOGM162);
  lcd.displ_onoff(true);
  // initialize UART
  Serial.begin(9600, SERIAL_9O2);
  // Show Startup screen
  printScreen(line1,line2);
  delay(STARTUPSCREEN_SHOW);
  // wait for serial to connect
  while (!Serial) {}
}

void loop() {
  while (Serial.available() < 1) {}
  if (Serial.read() == SIMPLETEXT_START) {      // check if simple text was sent with EX protocol.
    simpleTextExtraction();                     // Extract ASCII text for Jetibox display - 2x 16 characters
  }

  // TODO - CHECK IF simpleTextExtraction can provide a value back to check for success. subfunction to return boolean? or is void good enough. I assume no.

  // Use 20ms pause to send Jetibox buttons and display text. JetiBox Buttons need to support of
  // simultanuously pressing buttons and long holding them (e.g. push left & right long to clear values)
  if (strlen(line1) == 16) {                    // Check Text-Length to be sure to know the package ending was detected correctly
    if ((JetiBoxButtons & 0x00F0) != 0x00F0) {  // If buttons pressed, send them
      delay(1);                                 // might not be needed
      Serial.write(JetiBoxButtons);             // Send button status if some are pressed (!= 0x00F0).
    }
    else {
      Serial.write(0x00F0);                     // Send buttons up if not pressed
    }
    printScreen(line1,line2);                   // print lines to display, if full message was received
  }
}

void printScreen(char *line1, char *line2) {
  lcd.clear_display();
  lcd.position(1, 1);
  lcd.string(line1);
  lcd.position(1, 2);
  lcd.string(line2);
  return 0;
}

// ISR(PCINT1_vect) {}
// ISR(PCINT2_vect){}
ISR(PCINT1_vect) {                              // Button up interrupt. They can be pressed simultaneously. Contact swinging is filtered by waiting some ms for the next button press
  if ((millis() - lastpressed_buttons) > DEBOUNCE_TIME) {
    uint16_t tmp = PINC << 4;                   // Read pins in one shot. By connecting the buttons properly, bit shifting is the solution and IRS becomes really short.
    JetiBoxButtons = 0x00F0 & tmp;              // make sure you dont pick up anything else than the 4 buttons.
  }
  lastpressed_buttons = millis();               // debounce check timestamp setting
  return 0;
}

// TODO: Shall I use to move pointers and not modify global variables in sub-function?
void simpleTextExtraction () {
  for (int i = 0; i < 32 ; i++) {               // read text message (32 bytes ASCII) and print each one in display 
    while (Serial.available() < 1) {}
    if (i<16) {
      line1[i] = Serial.read() & 0xFF;          // make sure to extract ASCII characters from 9bit protocol and omit 9th bit (& 0xFF)
    }
    else {
      line2[i-16] = Serial.read() & 0xFF;       // make sure to extract ASCII characters from 9bit protocol and omit 9th bit (& 0xFF)    
    }
  }
  while (Serial.available() < 1) {} 
  if (Serial.read() != SIMPLETEXT_END) {        // if nok and end byte (0xFF) is missing, remove strings (string length = 0)
    line1[0] = NULL;
    line2[0] = NULL;
    return 1;
  }
  else {
    line1[16] = NULL;                           // if ok and end byte (0xFF) was sent, set string length to 16. Not sure if required, as this byte 
    line2[16] = NULL;                           // was set as NULL during initialization
    return 0;
  }
}