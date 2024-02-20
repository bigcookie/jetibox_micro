/*
 * Copyright (c) 2014 by ELECTRONIC ASSEMBLY <technik@lcd-module.de>
 * EA DOGM-Text (ST7036) library for arduino.
 *
 * This file is free software; you can redistribute it and/or modify
 * it under the terms of either the GNU General Public License version 2
 * or the GNU Lesser General Public License version 2.1, both as
 * published by the Free Software Foundation.
 */

#include <Arduino.h>
#include <SPI.h>

#include "dogm_7036.h"


#define INITLEN 8
byte init_DOGM081_3V[INITLEN] = {0x31, 0x14, 0x55, 0x6D, 0x75, 0x30, 0x01, 0x06};
byte init_DOGM081_5V[INITLEN] = {0x31, 0x1C, 0x51, 0x6A, 0x74, 0x30, 0x01, 0x06};

byte init_DOGM162_3V[INITLEN] = {0x39, 0x14, 0x55, 0x6D, 0x78, 0x38, 0x01, 0x06};
byte init_DOGM162_5V[INITLEN] = {0x39, 0x1C, 0x52, 0x69, 0x74, 0x38, 0x01, 0x06};

byte init_DOGM163_3V[INITLEN] = {0x39, 0x15, 0x55, 0x6E, 0x72, 0x38, 0x01, 0x06};
byte init_DOGM163_5V[INITLEN] = {0x39, 0x1D, 0x50, 0x6C, 0x7C, 0x38, 0x01, 0x06};


//----------------------------------------------------public Functions----------------------------------------------------
//Please use these functions in your sketch

/*----------------------------
Func: DOG-INIT
Desc: Initializes SPI Hardware/Software and DOG Displays
Vars: CS-Pin, MOSI-Pin, SCK-Pin (MOSI=SCK Hardware else Software), RS-Pin, Reset-Pin, 5V = true / 3.3V = false, lines
------------------------------*/
void dogm_7036::initialize(byte p_cs, byte p_si, byte p_clk, byte p_rs, byte p_res, boolean sup_5V, byte lines) 
{
	byte *ptr_init; //pointer to the correct init values
	byte i;
	
	cursor = 0x0C; //Display on/off control status at power on reset, needed for cursor on/off and Display on/off
	
	dogm_7036::p_rs = p_rs;
	pinMode(p_rs, OUTPUT);
	spi_initialize(p_cs, p_si, p_clk); //init SPI to Mode 3
	
	//perform a Reset
	digitalWrite(p_res, LOW);
	pinMode(p_res, OUTPUT);
	delayMicroseconds(10);
	digitalWrite(p_res, HIGH);
	delay(5);
	
	//Init DOGM-Text displays, depending on users choice of supply voltages and lines
	ptr_init = init_DOGM162_3V; //default pointer for wrong parameters
	if(lines == 1 && sup_5V == false) 		ptr_init = init_DOGM081_3V;
	else if(lines == 1 && sup_5V == true) 	ptr_init = init_DOGM081_5V;
	else if(lines == 2 && sup_5V == false)	ptr_init = init_DOGM162_3V;
	else if(lines == 2 && sup_5V == true) 	ptr_init = init_DOGM162_5V;
	else if(lines == 3 && sup_5V == false) 	ptr_init = init_DOGM163_3V;
	else if(lines == 3 && sup_5V == true) 	ptr_init = init_DOGM163_5V;
	
	flag_5V = sup_5V;    //need to save which DOG display is connected for function set (contrast) and
	displ_lines = lines; // set position
	
	digitalWrite(p_rs, LOW);
	for (i=0; i < INITLEN; i++)
	command(*ptr_init++);
	
	displ_onoff(true);   //Display on
	cursor_onoff(false); //Cursor off
}

/*----------------------------
Func: String
Desc: Shows a String on the DOG-Display
Vars: String
------------------------------*/
void dogm_7036::string(const char *str)
{
	digitalWrite(p_rs, HIGH); //sending data to DOG
	digitalWrite(p_cs, LOW);
	while(*str) 
	{
		spi_out(*str++);
		delayMicroseconds(30); //data out needs 26 us
	}
	digitalWrite(p_cs, HIGH); //deselect DOG
}

/*----------------------------
Func: ascii
Desc: Shows a Character on the DOG-Display
Vars: Character
------------------------------*/
void dogm_7036::ascii(char character){
  data(character);
}

/*----------------------------
Func: position
Desc: Sets a new cursor position DOG-Display
Vars: column (1..16), line (1..3)
------------------------------*/
void dogm_7036::position(byte column, byte line)
{
	byte cmd = 0;
	if(column == 0) column = 1; //minimum column 1
	if(column > 16) column = 16;//maximum column 16
	
	if(displ_lines == 2 && line == 2) //2-Line display second line adress
		cmd = 0x40;
	else if(displ_lines == 3 && line == 2)//3-Line display second line adress
		cmd = 0x10;
	else if(displ_lines == 3 && line == 3)//3-Line display third line adress
		cmd = 0x20;
	
	command(0x80 + cmd + column - 1); //DOG display starts with column 0 --> decrement
}

/*----------------------------
Func: displ_onoff
Desc: turns the entire DOG-Display on or off
Vars: on (true = display on, false = display off)
------------------------------*/
void dogm_7036::displ_onoff(boolean on) 
{
	if(on == true)
		cursor |= 0x04;
	else
		cursor &= ~0x04;

	command(cursor);
}

/*----------------------------
Func: cursor_onoff
Desc: turns the cursor on or off
Vars: on (true = cursor blinking, false = cursor off)
------------------------------*/
void dogm_7036::cursor_onoff(boolean on) 
{
	if(on == true)
		cursor |= 0x01;
	else
		cursor &= ~0x01;

	command(cursor);
}

/*----------------------------
Func: define_char
Desc: defines own character
Vars: adress (CGRAM) of own char, bit pattern
------------------------------*/
void dogm_7036::define_char(byte mem_adress, byte *dat)
{
	byte i=0;
	command(0x40+8*mem_adress);

	for (i=0; i<8; i++) 
		data(dat[i]);
		
  position(1,1); //set standard position DDRAM Adress
}

/*----------------------------
Func: clear_display
Desc: clears the entire DOG-Display
Vars: ---
------------------------------*/
void dogm_7036::clear_display(void) 
{
	command(0x01); //clear display and return home
}

/*----------------------------
Func: contrast
Desc: sets contrast to the DOG-Display
Vars: byte contrast (0..63)
------------------------------*/
void dogm_7036::contrast(byte contr) 
{
	contr &=0x3F; //contrast has only 6 bits
	
	if(displ_lines == 1) //switch to instruction table 1 depending on display lines
		command(0x31);
	else //2 and 3 line display
		command(0x39);
	
	if(flag_5V)
		command(0x50 | (contr >> 4)); //booster off, 2 high bits of contrast
	else
		command(0x54 | (contr >> 4)); //booster on, 2 high bits of contrast
	
	command(0x70 | (contr&0x0F));   //4 low bits of contrast
	
	if(displ_lines == 1) //switch to instruction table 0 depending on display lines
		command(0x30);
	else
		command(0x38);
}

//----------------------------------------------------private Functions----------------------------------------------------
//normally you don't need those functions in your sketch

/*----------------------------
Func: command
Desc: Sends a command to the DOG-Display
Vars: data
------------------------------*/
void dogm_7036::command(byte dat) 
{
	digitalWrite(p_rs, LOW);
	spi_put_byte(dat);
	if(dat <= 0x03)  //return home or clear display need 1.08 ms
		delay(1);
	else
		delayMicroseconds(30); //all other commands need 26 us
}

/*----------------------------
Func: data
Desc: Sends data to the DOG-Display
Vars: data
------------------------------*/
void dogm_7036::data(byte dat) 
{
	digitalWrite(p_rs, HIGH);
	spi_put_byte(dat);
}

/*----------------------------
Func: spi_initialize
Desc: Initializes SPI Hardware/Software
Vars: CS-Pin, MOSI-Pin, SCK-Pin (MOSI=SCK Hardware else Software)
------------------------------*/
void dogm_7036::spi_initialize(byte cs, byte si, byte clk) 
{
	//Set pin Configuration
	p_cs = cs;

	if(si == clk)
	{
		hardware = true;
		p_si = MOSI;
		p_clk = SCK;
	}
	else
	{
		hardware = false;
		p_si = si;
		p_clk = clk;
	}

	// Set CS to deselct slaves
	digitalWrite(p_cs, HIGH);
	pinMode(p_cs, OUTPUT);
	
	// Set Data pin as output
	pinMode(p_si, OUTPUT);
	
	// Set SPI-Mode 3: CLK idle high, rising edge, MSB first
	digitalWrite(p_clk, HIGH);
	pinMode(p_clk, OUTPUT);
	if(hardware)
	{
		SPI.begin();
		SPI.setBitOrder(MSBFIRST);
		SPI.setDataMode(SPI_MODE3);
		SPI.setClockDivider(SPI_CLOCK_DIV16);
	}
}

/*----------------------------
Func: spi_put_byte
Desc: Sends one Byte using CS
Vars: data
------------------------------*/
void dogm_7036::spi_put_byte(byte dat) 
{
	digitalWrite(p_cs, LOW);
	spi_out(dat);
	digitalWrite(p_cs, HIGH);
	delayMicroseconds(30); //data commands need 26 us
}

/*----------------------------
Func: spi_put
Desc: Sends bytes using CS
Vars: ptr to data and len
------------------------------*/
void dogm_7036::spi_put(byte *dat, int len)
{
	digitalWrite(p_cs, LOW);
	do
	{
		spi_out(*dat++);
		delayMicroseconds(30); //all commands need 26 us (Clear display and return home 1ms see command())
	}while(--len);
	
	digitalWrite(p_cs, HIGH);
}

/*----------------------------
Func: spi_out
Desc: Sends one Byte, no CS
Vars: data
------------------------------*/
void dogm_7036::spi_out(byte dat) 
{
	byte i = 8;
	if(hardware) 
		SPI.transfer(dat);
	else 
	{
		do 
		{
			if(dat & 0x80)
				digitalWrite(p_si, HIGH);
			else
				digitalWrite(p_si, LOW);
			digitalWrite(p_clk, LOW);
			dat <<= 1;
			digitalWrite(p_clk, HIGH);
		}while(--i);
	}
}

