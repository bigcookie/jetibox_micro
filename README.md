# JetiBox Micro
This is a project to recreate a JetiBox (small Box with display to display sensor data or sensor/device menus from Jeti remote control devices). This project specifically aimed for using a very small display to be able to fit it into a Graupner MC24 instead of the voltmeter/battery meter.
This Jetibox Micro can be used to program sensors or display messages on the receiver (e.g. form TU2-module).

Please note that I am not a skilled developer and the intention is to simply "make it run". The first release (v0.1) is actually not beautiful, but working. Second version is in the works and should be better structured.

## Required hardware
I am not associated with any of the sources I list. Just want to share the specs in an easy way
- Arduino Mini Pro, 5V, 16MHz (e.g. https://www.az-delivery.de/en/products/pro-mini-100-arduino-kompatibel)
- EA DOGM-162LA 2x16 reflective matrix display (e.g. https://www.reichelt.de/lcd-textmodul-48-3-x-12-mm-gelb-gruen-ea-dogm162l-a-p55130.html?&trstct=pos_2&nbc=1)
- 4 buttons (https://www.reichelt.de/kurzhubtaster-6x6mm-hoehe-9-5mm-12v-vertikal-taster-3301b-p27893.html?&nbc=1)
- Some connectors (I used JST-XH connectors)

## Pinning and hardware setup
Take a look at the Arduino sketch for now. All is described in the first comments.

## Hardware and library sources
I am not a skilled developer, so the code is suboptimal. The following are the sources I used for this project.

### HardwareSerial with 9bit support
Thanks to Sherzaad who created the library I used, I could follow-up on this project. You need to copy those to your Arduino folder, where the "core" libraries are located and replace the existing HWSerial files. You will not lose any functionality, but get 9bit support on top.
I got the library from here: 
https://forum.arduino.cc/t/solved-9-bit-serial-problem/496726/5

### DOGM-Arduino library
Officially provided by Display Vision, the manufacture. You can download the libraries and examples from here: 
https://www.lcd-module.com/lcd-tft-code-example-programming/application-note/arduino.html

This is the direct link for the 2x16 displays: 
http://www.lcd-module.de/fileadmin/downloads/development%20service/Arduino/Arduino%20meets%20EA%20DOGM081%20DOGM162%20DOGM163.zip

### Arduino 5V 16MHz (AtMega 328p)
Not associated with AZDelivery, but I used their board from here: 
https://www.az-delivery.de/en/products/pro-mini-100-arduino-kompatibel

### Jeti Telemetry protocol documentation
EX-Bus and Telemetry protocol can be found here: 
https://www.jetimodel.com/support/telemetry-protocol/

Direct link for telemetry protocol documentation: 
https://www.jetimodel.com/support/telemetry-protocol/jeti-telemetry-communication-protocol.html

# Approach on software
Arduino listens on UART port for Jeti Telemetry protocol "Simple Text Messages". They either are sent standalone or come appended to a higher level protocol message. ASCII text is being extracted from the simple text messages and verified by checking the expected and byte of the message. Please note that differences are coming in bit #9 to separate data and message separator for example.
The message is display in the display then.
After message is received, the sensor/remote control module pauses for 20ms and is able to receive data. This is, when you can send button interaction. buttons allowed are "up", "down", "left", "right". They can be pushed simultaneously and can be pressed short and long. All of this is required to edit attributes in the e.g. sensors/remote control modules menu.
While the UART is listening on the telemetry protocol is done via hardware UART, the display is using the hardware SPI interface and the buttons are handled with a separate interrupt service routine.
