/*
 * Copyright (c) 2014 by ELECTRONIC ASSEMBLY <technik@lcd-module.de>
 * EA DOGM-Text (ST7036) software library for arduino.
 *
 * This file is free software; you can redistribute it and/or modify
 * it under the terms of either the GNU General Public License version 2
 * or the GNU Lesser General Public License version 2.1, both as
 * published by the Free Software Foundation.
 */

#ifndef DOGM7036_H
#define DOGM7036_H

#define DOGM081 1
#define DOGM162 2
#define DOGM163 3


class dogm_7036
{
	public:
		void initialize       (byte p_cs, byte p_si, byte p_clk, byte p_rs, byte p_res, boolean sup_5V, byte lines);
		void string           (const char *str);
		void ascii            (char character);
		void position         (byte column, byte line);
		void displ_onoff	  (boolean on);
		void cursor_onoff	  (boolean on);
		void define_char      (byte mem_adress, byte *dat);
		void clear_display    (void);
		void contrast         (byte contr);
		
	private:
		byte p_cs;
		byte p_si;
		byte p_clk;
		byte p_rs;
		boolean hardware;
		boolean flag_5V;
		byte displ_lines;
		
		byte cursor;
		
		void command(byte dat);
		void data(byte dat);
		
		void spi_out(byte dat);
		void spi_initialize(byte cs, byte si, byte clk);
		void spi_put_byte(byte dat);
		void spi_put(byte *dat, int len);
};

#endif
