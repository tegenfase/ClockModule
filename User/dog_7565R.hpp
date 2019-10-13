/*
 * Copyright (c) 2014 by ELECTRONIC ASSEMBLY <technik@lcd-module.de>
 * EA DOG Graphic (ST7565R) software library for arduino.
 *
 * This file is free software; you can redistribute it and/or modify
 * it under the terms of either the GNU General Public License version 2
 * or the GNU Lesser General Public License version 2.1, both as
 * published by the Free Software Foundation.
 */
#include <stm32f405xx.h>
#include <stm32f4xx_hal.h>

#ifndef DOG_7565R_H
#define DOG_7565R_H


#define DOGM128 1
#define DOGL128 2
#define DOGM132 3

#define VIEW_BOTTOM 0xC0
#define VIEW_TOP 	0xC8


class dog_7565R
{
  public:
    void initialize     (SPI_HandleTypeDef* hspi, uint8_t type);
    void clear			(void);
    void contrast       (uint8_t contr);
	void view			(uint8_t direction);
	void string         (uint8_t column, uint8_t page, const uint8_t *font_adress, const char *str);
	void rectangle		(uint8_t start_column, uint8_t start_page, uint8_t end_column, uint8_t end_page, uint8_t pattern);
	void picture		(uint8_t column, uint8_t page, const uint8_t *pic_adress);

  private:
    uint8_t p_cs;
    uint8_t p_si;
    uint8_t p_clk;
    uint8_t p_a0;
	uint8_t type;
	SPI_HandleTypeDef* hspi;
	bool hardware;
    bool top_view;
	
	void position   (uint8_t column, uint8_t page);
    void command	(uint8_t dat);
    void data		(uint8_t dat);
    
    void spi_initialize	(uint8_t cs, uint8_t si, uint8_t clk);
    void spi_put_byte	(uint8_t dat);
    void spi_put		(uint8_t *dat, int len);
	void spi_out		(uint8_t dat);
};

#endif
