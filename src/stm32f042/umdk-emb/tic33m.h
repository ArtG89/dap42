/*
 * Copyright (c) 2018, Unwired Devices LLC
 *
 * Permission to use, copy, modify, and/or distribute this software
 * for any purpose with or without fee is hereby granted, provided
 * that the above copyright notice and this permission notice
 * appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL
 * WARRANTIES WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE
 * AUTHOR BE LIABLE FOR ANY SPECIAL, DIRECT, INDIRECT, OR
 * CONSEQUENTIAL DAMAGES OR ANY DAMAGES WHATSOEVER RESULTING FROM
 * LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT,
 * NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN
 * CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */

#ifndef TIC33M_H_INCLUDED
#define TIC33M_H_INCLUDED

typedef struct {
    uint32_t load_port;
    uint32_t load_pin;
    uint32_t din_port;
    uint32_t din_pin;
    uint32_t clk_port;
    uint32_t clk_pin;
    uint32_t lclk_port;
    uint32_t lclk_pin;
} tic33m;

typedef enum {
    TIC33M_SYMB_NONE = 0,
    TIC33M_SYMB_MINUS = 10,
    TIC33M_SYMB_DEGREE = 11,
    TIC33M_SYMB_SPACE = 12,
    TIC33M_SYMB_UNDERSCORE = 13,
    TIC33M_SYMB_UPPERSCORE = 14,
    TIC33M_SYMB_I = 15,
    TIC33M_SYMB_U = 16,
    TIC33M_SYMB_P = 17,
    TIC33M_SYMB_C = 18,
    TIC33M_SYMB_A = 19,
    TIC33M_SYMB_L = 20,
} tic33m_first_symbol_t;

void tic33m_init(tic33m *dev);
int  tic33m_display_number(tic33m *dev, int32_t num, uint8_t precision, tic33m_first_symbol_t symb);
int  tic33m_display_time(tic33m *dev, uint32_t seconds);
int  tic33m_display_string(tic33m *dev, uint8_t *digits);
void tic33m_lclk(tic33m *dev);

#endif /* TIC33M_H_INCLUDED */