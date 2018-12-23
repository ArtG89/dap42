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


#include <libopencm3/stm32/gpio.h>
#include <string.h>
#include "tic33m.h"

static tic33m dev;

#define   TIC33M_MINUS   10
#define   TIC33M_DEGREE  11
#define   TIC33M_SPACE   12

static const uint8_t tic33m_digits[]  = {
                                          2+8+16+32+64+128,
                                          2+8,
                                          128+2+4+32+16,
                                          2+8+128+4+16,
                                          64+4+2+8,
                                          128+64+4+8+16,
                                          128+64+4+8+16+32,
                                          128+2+8,
                                          2+4+8+16+32+64+128,
                                          2+4+8+16+64+128,
                                          4,
                                          2+4+64+128,
                                          0
                                        };


static inline void tic33m_delay(void) {
    /* 0.4 us min strobe length */
    /* 20 cycles on 48 MHz CPU */
    __asm("nop; nop; nop; nop; nop;");
    __asm("nop; nop; nop; nop; nop;");
    __asm("nop; nop; nop; nop; nop;");
    __asm("nop; nop; nop; nop; nop;");
}

static void tic33m_putchar(uint8_t digit, bool point) {
    uint8_t code = tic33m_digits[digit];
    if (point) {
        code |= 1;
    }
    
    for (uint8_t i = 8; i; i--) {
        if (code & 128) {
            gpio_set(dev.din_port, dev.din_pin);
        } else {
            gpio_clear(dev.din_port, dev.din_pin);
        }
        gpio_set(dev.clk_port, dev.clk_pin);
        tic33m_delay();
        gpio_clear(dev.clk_port, dev.clk_pin);
        code <<= 1;
    }
}

int tic33m_print(int32_t num, uint8_t precision) {
    /* 9 digits on TIC33M */
    uint8_t digits[9];
    
    memset(digits, TIC33M_SPACE, 8);
    
    if ((num > 999999999) || (num < -99999999) || (precision > 8)) {
        return -1;
    }
    
    if (num == 0) {
        for (int i = 0; i < (precision + 1); i++) {
            digits[8-i] = 0;
        }
    } else {
        int i = 8;
        do {
            digits[i] = num % 10;
            num /= 10;
            i--;
        } while (num != 0);
    }
    
    bool point;
    for (int i = 0; i < 9; i++) {
        if (i == (8 - precision)) {
            point = true;
            if (digits[i] == TIC33M_SPACE) {
                digits[i] = 0;
            }
        } else {
            point = false;
        }
        
        if ((num < 0) && (digits[i] == TIC33M_SPACE) && (digits[i+1] != TIC33M_SPACE)) {
            digits[i] = TIC33M_MINUS;
        }
        
        tic33m_putchar(digits[i], point);
    }
    
    gpio_set(dev.load_port, dev.load_pin);
    tic33m_delay();
    gpio_clear(dev.load_port, dev.load_pin);
    
    return 0;
}

void tic33m_init(tic33m device) {
    dev = device;
    
    gpio_set_output_options(dev.load_port, GPIO_OTYPE_PP, GPIO_OSPEED_HIGH, dev.load_pin);
    gpio_mode_setup(dev.load_port, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, dev.load_pin);
    gpio_clear(dev.load_port, dev.load_pin);
    
    gpio_set_output_options(dev.din_port, GPIO_OTYPE_PP, GPIO_OSPEED_HIGH, dev.din_pin);
    gpio_mode_setup(dev.din_port, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, dev.din_pin);
    gpio_clear(dev.din_port, dev.din_pin);
    
    gpio_set_output_options(dev.clk_port, GPIO_OTYPE_PP, GPIO_OSPEED_HIGH, dev.clk_pin);
    gpio_mode_setup(dev.clk_port, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, dev.clk_pin);
    gpio_clear(dev.clk_port, dev.clk_pin);
}