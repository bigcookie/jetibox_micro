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
defined (Separateor Byte 0xFE, end byte 0xFF). To fully identify them, the evaluation of the 9th bit 
in Jeti UART configuration must be used. Jeti uses 1 start bit, 9 data bits, odd parity and 2 stop bits 
(Serial_9O2). This requires to use a UART library which supports 9bit.

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
  Push-Buttons:
  Button right:  Pin A0   \
  Button up:     Pin A1    \
  Button down:   Pin A2     | 5-pin connector to buttons
  Button left:   Pin A3.   /  
  Button Gnd:    Gnd      /

  DOGM162L-A display:
  LCD CS:        Pin 10   \
  LCD CLK:       Pin 13    \
  LCD SI:        Pin 11     \ 6-pin connector to LCD
  LCD RS:        Pin  8     / 
  LCD Gnd:       Gnd       /
  LCD VCC:       Vcc      / 

  Jeti TU2-Module/Sensor:
  Sensor Ground: Gnd      \
  Sensor VCC:    Vcc       | 3-pin connector to TU2-Modul (Servo-connector)
  Sensor TXD:    RXD      /

                 Gnd      \ optional 2-pin connector to allow to put a 
                 Res      / reset button on the remote control housing

  TXD:           RXD      connected via 2,2kOhm resistor

  Alternative 2x16 display:
  LCD RS:        Pin  7
  LCD EN:        Pin  8
  LCD D4:        Pin 10
  LCD D5:        Pin 11
  LCD D6:        Pin 12
  LCD D7:        Pin 13
  LCD RW:        Gnd
  LCD VSS:       Gnd
  LCD Vdd:       Vcc
  LCD  A:        Vcc via 220 Ohm resistor
  LCD  K:        Gnd
  LCD V0:        0-5V for contrast (I use about 1V with a voltage divider)

Additional protection resistors can be added between display and Microcontroller. For me system ran stable,
so I decided to not put them in.
*/

// Display selection
//#define LCD_2x16                        // Uncomment if you use a standard 2x16 display, otherwise DOGM is being used.

// pin buttons and debounce config values
#define PIN_BUTTON_RIGHT A0
#define PIN_BUTTON_UP A1
#define PIN_BUTTON_DOWN A2
#define PIN_BUTTON_LEFT A3
#define DEBOUNCE_TIME 20                // in ms
// pin definitions 2x16 LCD - 4-Bite mode
#define LCD_PIN_RS 7
#define LCD_PIN_EN 8
#define LCD_PIN_D4 10
#define LCD_PIN_D5 11
#define LCD_PIN_D6 12
#define LCD_PIN_D7 13
// pin definitions DOGM LCD - SPI (use with HW SPI, which is initialized in DOGM library)
#define DOG_LCD_PIN_SI 10
#define DOG_LCD_PIN_RS 8

// ------------------------------
// DO NOT CHANGE BELOW
// ------------------------------

// Startup screen configuration
#define STARTUPSCREEN_SHOW 2000
// Important protocol bytes to check for (see Jeti protocol definition)
#define SIMPLETEXT_START 0xFE
#define SIMPLETEXT_END 0xFF
// Init-Screen Text
#define INITSCREEN_LINE1 "JetiBox-Micro by"
#define INITSCREEN_LINE2 "   Andre Kuhn   "

#include <Arduino.h>
#include <LiquidCrystal.h>      // need to be installed in Arduino IDE or added to your project folder
#include "dogm_7036.h"

// global variables to be defined
volatile uint16_t JetiBoxButtons = 0x00F0;  // Byte to send to sensor/transmitter to navigate through Jetibox menu. Need to be 2 bytes to capture data bit #9 as well
int lastpressed_buttons = 0;  // required for software debouncing

// initiate the display lines with start screen values
char line1[17] = INITSCREEN_LINE1;
char line2[17] = INITSCREEN_LINE2;

// declaration of required functions
void printScreen(char *line1, char *line2);
int simpleTextExtraction();

// Initialization of LCD
#ifdef LCD_2x16
  LiquidCrystal lcd(LCD_PIN_RS, LCD_PIN_EN, LCD_PIN_D4, LCD_PIN_D5, LCD_PIN_D6, LCD_PIN_D7);
#else
  dogm_7036 lcd;
#endif

void setup() {
  // configure pins
  pinMode(PIN_BUTTON_UP, INPUT_PULLUP);     // up
  pinMode(PIN_BUTTON_DOWN, INPUT_PULLUP);   // down
  pinMode(PIN_BUTTON_LEFT, INPUT_PULLUP);   // left
  pinMode(PIN_BUTTON_RIGHT, INPUT_PULLUP);  // right

  // configure interrupts 
  cli();                      // turn interrupts off, while changing
  //configure PORT interrupts
  //  PCICR |= 0b00000001;    // Enable Port B Pin Change Interrupts. Bit0=B - PCINT 0-7, Bit1=C - PCINT 8-14, Bit2=D - PCINT 16-23
  PCICR |= 0b00000010;        // Enable Port C Pin Change Interrupts. Bit0=B - PCINT 0-7, Bit1=C - PCINT 8-14, Bit2=D - PCINT 16-23
  //  PCMSK0 |= 0b00001111;   // Enable pin interrupts: PCINT0  -> Port B
  PCMSK1 |= 0b00001111;       // Enable Pin interrupts: PCINT8-11 -> Port C
  //  PCMSK2 |= 0b10000000;   // PCINT11 -> Port D

  // Configure Timer1 interrupts
  TCCR1A = 0;                 // Normal operation
  TCCR1B = (1 << WGM12) | (1 << CS12) | (1 << CS10); // CTC mode, Prescaler 1024
  OCR1A = 11718;              // 750ms delay (16MHz / 1024 prescaler / 2Hz)  
  TIMSK1 |= (1 << OCIE1A);    // Enable Timer1 compare interrupt
  sei();                      // turn interrupts on after changing their configuration

  // initialize LCD
  #ifdef LCD_2x16
    lcd.begin(16,2);
  #else
    lcd.initialize(DOG_LCD_PIN_SI, 0, 0, DOG_LCD_PIN_RS, 4, 1, DOGM162);
    lcd.displ_onoff(true);
  #endif
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
  if (Serial.read() == SIMPLETEXT_START) {        // check if simple text was sent with EX protocol.
    // Extract ASCII text for Jetibox display - 2x 16 characters. If correctl extracted follow-up with buttons an display
    if (simpleTextExtraction() == 0) {
      // Use 2ms pause to send Jetibox buttons status and display text as otherwise Duplex Receiver cannot switch quickly enough to receiving mode after sending Jetibox data
      delay(2);
      Serial.write(JetiBoxButtons & 0x00F0);      // Send button status if some are pressed (!= 0x00F0).
      printScreen(line1,line2);                   // print lines to display, if full message was received
      TCNT1 = 0;                                  // Reset timer after receiving valid text
    }
  }
}

// Function to print to lines on the screen
void printScreen(char *line1, char *line2) {
#ifdef LCD_2x16
//  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print(line1);
  lcd.setCursor(0,1);
  lcd.print(line2);
#else
//  lcd.clear_display();
  lcd.position(1, 1);
  lcd.string(line1);
  lcd.position(1, 2);
  lcd.string(line2);
#endif
  return;
}

// Interrupt Service Routine
// ISR(PCINT1_vect) {}
// ISR(PCINT2_vect){}
ISR(PCINT1_vect) {                              // Button up interrupt. They can be pressed simultaneously. Contact swinging is filtered by waiting some ms for the next button press
  if ((millis() - lastpressed_buttons) > DEBOUNCE_TIME) {
    JetiBoxButtons = 0x00F0 & (PINC << 4);      // make sure you dont pick up anything else than the 4 buttons and move the bits to the correct location for Jeti protocol
    lastpressed_buttons = millis();               // debounce check timestamp setting
  }
  return;
}

// Interrupt Service Routine for timer
ISR(TIMER1_COMPA_vect) {
  const char* l1=INITSCREEN_LINE1;
  const char* l2=INITSCREEN_LINE2;
  printScreen(l1,l2);
}

// Text extraction from simple text protocol
int simpleTextExtraction () {
  for (int i = 0; i < 32 ; i++) {               // read text message (32 bytes ASCII) and print each one in display 
    while (Serial.available() < 1) {}
    volatile uint8_t protocol_byte = Serial.read() & 0xFF;   // make sure to extract ASCII characters from 9bit protocol and omit 9th bit (& 0xFF)
    if (i<16) {
      line1[i] = protocol_byte;          // Store character 1-16 in line1 char array
    }
    else {
      line2[i-16] = protocol_byte;       // Store character 16-32 in line2 char array 
    }
  }
  while (Serial.available() < 1) {} 
  if (Serial.read() != SIMPLETEXT_END) {        // if nok and end byte (0xFF) is missing, remove strings (string length = 0)
    return 1;                                   // return 1 if nok
  }
  else {
    return 0;                                   // return 0 if ok
  }
}
