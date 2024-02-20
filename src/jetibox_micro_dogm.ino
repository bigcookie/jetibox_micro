/* 
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
  Button right:  Pin A0
  Button uo:     Pin A1
  Button down:   Pin A2 
  Button left:   Pin A3
  LCD CS:        Pin 10
  LCD CLK:       Pin 13
  LCD SI:        Pin 11
  LCD RS:        Pin  9
  Sensor Ground: Gnd
  Sensor VCC:    Vcc
  Sensor TXD:    RXD
  HW TXD:        RXD via 2,2kOhm resistor

Additional protection resistors can be added between display and Microcontroller. For me system ran stable,
so I decided to not mount them.
*/

#include <Arduino.h>
#include "dogm_7036.h"
#include <SPI.h>
#include <avr/interrupt.h>

// global variables to be defined
volatile uint16_t JetiBoxButtons = 0x00F0;  // Byte to send to sensor/transmitter to navigate through Jetibox menu.

// pin definitions LCD - SPI
const int pin_lcd_si = 10, pin_lcd_rs = 8;

// button pin definitions and config values
const int pin_button_right = A0, pin_button_up = A1, pin_button_down = A2, pin_button_left = A3;
const int debounceTime = 30;
int lastpressed_buttons;  // required for software debouncing

// Important protocol bytes
const int SIMPLETEXT_START = 0xFE, SIMPLETEXT_END = 0xFF;  // protocol byte definition: separator 0x7E, simpleText start 0xFE, simpleText end 0xFF

struct JBTEXT {
  char line1[17];
  char line2[17];
};

// declaration of required functions and structs.
void loop();
void setup();
void printInitScreen();
void simpleTextExtractionChar();

JBTEXT JetiBoxTextChar;
dogm_7036 lcd;

void setup() {
  // setup for pins and buttons±
  lastpressed_buttons = 0;
  pinMode(pin_button_up, INPUT_PULLUP);     // up
  pinMode(pin_button_down, INPUT_PULLUP);   // down
  pinMode(pin_button_left, INPUT_PULLUP);   // left
  pinMode(pin_button_right, INPUT_PULLUP);  // right

  // interrupt configuration
  cli();                 // turn interrupts off, while changing
//  PCICR |= 0b00000001;   // Enable Port B Pin Change Interrupts. Bit0=B - PCINT 0-7, Bit1=C - PCINT 8-14, Bit2=D - PCINT 16-23
  PCICR |= 0b00000010;   // Enable Port C Pin Change Interrupts. Bit0=B - PCINT 0-7, Bit1=C - PCINT 8-14, Bit2=D - PCINT 16-23
//  PCMSK0 |= 0b00001111;  // Enable pin interrupts: PCINT0  -> Port B
  PCMSK1 |= 0b00001111;     // Enable Pin interrupts: PCINT8-11 -> Port C
  //  PCMSK2 |= 0b10000000;     // PCINT11 -> Port D
  sei();  // turn interrupts on after changing their configuariont

  // LCD and serial configuration
  lcd.initialize(pin_lcd_si, 0, 0, pin_lcd_rs, 4, 1, DOGM162);
  lcd.displ_onoff(true);
  Serial.begin(9600, SERIAL_9O2);
  while (!Serial) {
    ;  // wait for serial to connect
  }

  // Show Startup screen
  printInitScreen();
  delay(2000);
}

void loop() {
  while (Serial.available() < 1) {}

  if (Serial.read() == SIMPLETEXT_START) {  // check if simple text was sent with EX protocol.
    simpleTextExtractionChar();
  }

// TODO Prüfung der Länge strlen()
  // Use 20ms pause to send Jetibox buttons and display text.
  if ((strlen(JetiBoxTextChar.line1) == 16) && (strlen(JetiBoxTextChar.line2) == 16)) {             // Check Text-Length to be sure to know the package ended.
    if ((JetiBoxButtons & 0x00F0) != 0x00F0) {  // If buttons pressed, send them
      delay(1);
      Serial.write(JetiBoxButtons);
    }
    else {
      Serial.write(0x00F0);                     // Send buttons up if not pressed
    }
    lcd.position(1, 1);
    lcd.string(JetiBoxTextChar.line1);
    lcd.position(1, 2);
    lcd.string(JetiBoxTextChar.line2);
  }
}

// DOGM display
void printInitScreen() {
  lcd.clear_display();
  lcd.position(1, 1);
  lcd.string("JetiBox-Micro   ");
  lcd.position(1, 2);
  lcd.string("by Andre Kuhn   ");
  return;
}

// ISR(PCINT1_vect) {}
// ISR(PCINT2_vect){}
ISR(PCINT1_vect) {                    // Button up interrupt. They can be pressed simultaneously. Contact swinging is filtered by waiting some ms for the next button press
  if ((millis() - lastpressed_buttons) > debounceTime) {
    uint16_t tmp = PINC << 4;         // Read pins in one shot. By connecting the buttons properly, bit shifting is the solution and IRS becomes really short.
    JetiBoxButtons = 0x00F0 & tmp;    // make sure you dont pick up anything else than the 4 buttons.
  }
  lastpressed_buttons = millis();     // debounce check timestamp setting
}

void simpleTextExtractionChar () {
  for (int i = 0; i < 32 ; i++) {         // read text message (32 bytes ASCII) and print each one in display 
    while (Serial.available() < 1) {}
    if (i<16) {
      JetiBoxTextChar.line1[i] = Serial.read() & 0xFF; // make sure to extract ASCII characters from 9bit protocol and omit 9th bit (& 0xFF)
    }
    else {
      JetiBoxTextChar.line2[i-16] = Serial.read() & 0xFF; // make sure to extract ASCII characters from 9bit protocol and omit 9th bit (& 0xFF)    
    }
  }
  while (Serial.available() < 1) {} 
  if (Serial.read() != SIMPLETEXT_END) {  // all ok if message end byte (0xFF) is sent, return string
    JetiBoxTextChar.line1[0]='\0';
    JetiBoxTextChar.line2[0]='\0';
  }
  else {
    JetiBoxTextChar.line1[16]='\0';
    JetiBoxTextChar.line2[16]='\0';
  }
}