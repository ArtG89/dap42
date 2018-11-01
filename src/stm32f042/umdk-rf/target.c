/*
 * Copyright (c) 2016, Devan Lai
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

#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/crs.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/flash.h>

#include "tick.h"
#include "target.h"
#include "config.h"
#include "DAP/CMSIS_DAP_config.h"
#include "DFU/DFU.h"

/* Reconfigure processor settings */
void cpu_setup(void) {

}

/* Set STM32 to 48 MHz. */
void clock_setup(void) {
    rcc_clock_setup_in_hsi48_out_48mhz();

    // Trim from USB sync frame
    crs_autotrim_usb_enable();
    rcc_set_usbclk_source(RCC_HSI48);
}

/* 1000 Hz frequency */
static const uint16_t frequency = 1;
static uint16_t button1_counter = 0;
static uint16_t target_release_reset = 0;
static uint16_t target_release_boot = 0;
static uint16_t target_enable_power = 0;
static bool target_power_state = false;
static bool target_boot_state = false;
static const uint16_t blink_period_boot = 1000;
static const uint16_t blink_period_fail = 150;
static uint16_t blink_counter = 0;
static uint16_t target_power_failure = 0;

static void disable_power(void) {
    /* Disable output power */
    gpio_clear(POWER_OUTPUT_EN_PORT, POWER_OUTPUT_EN_PIN);
    /* Disable input power */
    gpio_clear(POWER_INPUT_EN_PORT, POWER_INPUT_EN_PIN);
    /* Disable green LED */
    gpio_set(LED_CON_GPIO_PORT, LED_CON_GPIO_PIN);
    
    target_power_state = false;
    target_boot_state = false;
}

void tim2_isr(void)
{
	if (timer_get_flag(TIM2, TIM_SR_CC1IF)) {
		/* Clear compare interrupt flag. */
		timer_clear_flag(TIM2, TIM_SR_CC1IF);

		/* Calculate and set the next compare value. */
		uint16_t new_time = timer_get_counter(TIM2) + frequency;

		timer_set_oc_value(TIM2, TIM_OC1, new_time);
        
        if (target_power_state && (gpio_get(POWER_FAULT_PORT, POWER_FAULT_PIN) == 0)) {
            target_power_failure++;
            if (target_power_failure == 100) {
                disable_power();
            }
        }
        
        if (target_release_reset) {
            target_release_reset--;
            
            if (target_release_reset == 0) {
                /* Release reset */
                gpio_set(nRESET_GPIO_PORT, nRESET_GPIO_PIN);
            }
        }
        
        /* Short circuit on 5V rail for 100 ms */
        if (target_power_failure == 100) {
            if (blink_counter == 0) {
                blink_counter = blink_period_fail;
                gpio_toggle(LED_CON_GPIO_PORT, LED_CON_GPIO_PIN);
            }
            blink_counter--;
        } else {
            if (target_boot_state) {
                if (blink_counter == 0) {
                    blink_counter = blink_period_boot;
                    gpio_toggle(LED_CON_GPIO_PORT, LED_CON_GPIO_PIN);
                }
                blink_counter--;
            }
        }
        
        if (target_enable_power) {
            target_enable_power--;
            
            if (target_enable_power == 0) {
                if (gpio_get(POWER_INPUT_EN_PORT, POWER_INPUT_EN_PIN) == 0) {    
                    /* Enable output power */
                    gpio_set(POWER_OUTPUT_EN_PORT, POWER_OUTPUT_EN_PIN);
                    
                    /* Release reset in 250 ms */
                    target_release_reset = 250;
                }
                target_power_state = true;
            }
        }
        
        if (target_release_boot) {
            target_release_boot--;
            
            if (target_release_boot == 0) {
                /* Release boot */
                gpio_clear(TARGET_BOOT_PORT, TARGET_BOOT_PIN);
            }
        }
        
		/* Check if button is pressed */
		if (gpio_get(nBOOT0_GPIO_PORT, nBOOT0_GPIO_PIN) == 0) {
            button1_counter++;
            if ((button1_counter == 1000) && target_power_state) {
                /* 1000 ms long press */
                target_boot_state = !target_boot_state;
                /* Reset target */
                gpio_clear(nRESET_GPIO_PORT, nRESET_GPIO_PIN);
                
                if (target_boot_state) {                
                    /* Boot from system memory */
                    gpio_set(TARGET_BOOT_PORT, TARGET_BOOT_PIN);
                    target_release_boot = 500;
                } else {
                    /* Enable green LED */
                    gpio_clear(LED_CON_GPIO_PORT, LED_CON_GPIO_PIN);
                }                
                target_release_reset = 1;
            }
        } else {
            /* button released */
            if ((button1_counter >= 100) && (button1_counter < 1000)) {
                /* 100 ms short press */        
                target_power_state = !target_power_state;
                
                /* Toggle power */
                if (target_power_state) {
                    /* Reset target */
                    gpio_clear(nRESET_GPIO_PORT, nRESET_GPIO_PIN);
                    /* Set boot from flash */
                    gpio_clear(TARGET_BOOT_PORT, TARGET_BOOT_PIN);
                    /* Enable input power */
                    gpio_set(POWER_INPUT_EN_PORT, POWER_INPUT_EN_PIN);
                    /* enable green LED */
                    gpio_clear(LED_CON_GPIO_PORT, LED_CON_GPIO_PIN);
                    
                    /* Enable output power if needed in 500 ms */
                    target_enable_power = 500;

                    target_boot_state = false;
                    target_power_failure = 0;
                } else {
                    disable_power();
                }
            }
            button1_counter = 0;
        }
	}
}

static void tim_setup(void)
{
	/* Enable TIM2 clock. */
	rcc_periph_clock_enable(RCC_TIM2);

	/* Enable TIM2 interrupt. */
	nvic_enable_irq(NVIC_TIM2_IRQ);

	/* Reset TIM2 peripheral to defaults. */
	rcc_periph_reset_pulse(RST_TIM2);

	/* Timer global mode:
	 * - No divider
	 * - Alignment edge
	 * - Direction up
	 * (These are actually default values after reset above, so this call
	 * is strictly unnecessary, but demos the api for alternative settings)
	 */
	timer_set_mode(TIM2, TIM_CR1_CKD_CK_INT,
		TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);

	/*
	 * Sets the prescaler to have the timer run at 2kHz -> IRQs at 1 kHz
	 */
	timer_set_prescaler(TIM2, (rcc_apb1_frequency / 2000));

	/* Disable preload. */
	timer_disable_preload(TIM2);
	timer_continuous_mode(TIM2);

	/* count full range, as we'll update compare value continuously */
	timer_set_period(TIM2, 65535);

	/* Set the initial output compare value for OC1. */
	timer_set_oc_value(TIM2, TIM_OC1, frequency);

	/* Counter enable. */
	timer_enable_counter(TIM2);

	/* Enable Channel 1 compare interrupt to recalculate compare values */
	timer_enable_irq(TIM2, TIM_DIER_CC1IE);
}

static void button_setup(void) {
    /* Set PB8 to an input */
    gpio_mode_setup(nBOOT0_GPIO_PORT, GPIO_MODE_INPUT, GPIO_PUPD_NONE, nBOOT0_GPIO_PIN);
}

void gpio_setup(void) {
    /*
      Button on PB8
      LED0, 1, 2 on PA0, PA1, PA4
      TX, RX (MCU-side) on PA2, PA3
      TGT_RST on PB1
      TGT_SWDIO, TGT_SWCLK on PA5, PA6
      TGT_SWO on PA7
    */

    /* Enable GPIO clocks. */
    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_GPIOB);
    rcc_periph_clock_enable(RCC_GPIOF);
    
    button_setup();
    
    /* reboot STM DFU bootloader if button is pressed */
    if (gpio_get(nBOOT0_GPIO_PORT, nBOOT0_GPIO_PIN) == 0) {
        DFU_reset_and_jump_to_bootloader();
    }

    /* Setup LEDs as open-drain outputs */
    gpio_set_output_options(LED_CON_GPIO_PORT, GPIO_OTYPE_OD, GPIO_OSPEED_LOW, LED_CON_GPIO_PIN);
    gpio_mode_setup(LED_CON_GPIO_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, LED_CON_GPIO_PIN);
                    
    gpio_set_output_options(LED_RUN_GPIO_PORT, GPIO_OTYPE_OD, GPIO_OSPEED_LOW, LED_RUN_GPIO_PIN);
    gpio_mode_setup(LED_RUN_GPIO_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, LED_RUN_GPIO_PIN);
                    
    gpio_set_output_options(LED_ACT_GPIO_PORT, GPIO_OTYPE_OD, GPIO_OSPEED_LOW, LED_ACT_GPIO_PIN);
    gpio_mode_setup(LED_ACT_GPIO_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, LED_ACT_GPIO_PIN);
    
    /* Reset target pin */
    gpio_set_output_options(nRESET_GPIO_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_LOW, nRESET_GPIO_PIN);
    gpio_mode_setup(nRESET_GPIO_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, nRESET_GPIO_PIN);
    
    /* Boot from flash target pin */
    gpio_set_output_options(TARGET_BOOT_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_LOW, TARGET_BOOT_PIN);
    gpio_mode_setup(TARGET_BOOT_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, TARGET_BOOT_PIN);
    
    /* Power sense pin */
    gpio_mode_setup(POWER_FAULT_PORT, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, POWER_FAULT_PIN);
    
    /* Input power enable pin */
    gpio_set_output_options(POWER_INPUT_EN_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_LOW, POWER_INPUT_EN_PIN);
    gpio_mode_setup(POWER_INPUT_EN_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, POWER_INPUT_EN_PIN);
    
    /* Output power enable pin */
    gpio_set_output_options(POWER_OUTPUT_EN_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_LOW, POWER_OUTPUT_EN_PIN);
    gpio_mode_setup(POWER_OUTPUT_EN_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, POWER_OUTPUT_EN_PIN);

    button1_counter = 100;
    
    tim_setup();
}

void target_console_init(void) {
    /* Enable UART clock */
    rcc_periph_clock_enable(CONSOLE_USART_CLOCK);

    /* Setup GPIO pins for UART2 */
    gpio_mode_setup(CONSOLE_USART_GPIO_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, CONSOLE_USART_GPIO_PINS);
    gpio_set_af(CONSOLE_USART_GPIO_PORT, CONSOLE_USART_GPIO_AF, CONSOLE_USART_GPIO_PINS);
}

void led_bit(uint8_t position, bool state) {
    uint32_t gpio = 0xFFFFFFFFU;
    if (position == 0) {
        gpio = LED_ACT_GPIO_PIN;
    }
    else if (position == 1) {
        gpio = LED_RUN_GPIO_PIN;
    }
    /*
    else if (position == 2) {
        gpio = LED_CON_GPIO_PIN;
    }
    */

    if (gpio != 0xFFFFFFFFU) {
        if (state) {
            gpio_clear(LED_CON_GPIO_PORT, gpio);
        } else {
            gpio_set(LED_CON_GPIO_PORT, gpio);
        }
    }
}

void led_num(uint8_t value) {
    /*
    if (value & 0x4) {
        gpio_clear(LED_CON_GPIO_PORT, LED_CON_GPIO_PIN);
    } else {
        gpio_set(LED_CON_GPIO_PORT, LED_CON_GPIO_PIN);
    }
    */
    if (value & 0x2) {
        gpio_clear(LED_RUN_GPIO_PORT, LED_RUN_GPIO_PIN);
    } else {
        gpio_set(LED_RUN_GPIO_PORT, LED_RUN_GPIO_PIN);
    }
    if (value & 0x1) {
        gpio_clear(LED_ACT_GPIO_PORT, LED_ACT_GPIO_PIN);
    } else {
        gpio_set(LED_ACT_GPIO_PORT, LED_ACT_GPIO_PIN);
    }
}
