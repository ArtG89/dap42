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
#include <libopencm3/stm32/adc.h>
#include <libopencm3/stm32/exti.h>

#include "tick.h"
#include "target.h"
#include "config.h"
#include "console.h"
#include "DAP/CMSIS_DAP_config.h"
#include "DFU/DFU.h"
#include "USB/vcdc.h"
#include "tic33m.h"

static tic33m tic33m_dev;

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
static uint16_t button2_counter = 0;
static uint16_t target_release_reset = 0;
static uint16_t target_release_boot = 0;
/* static uint16_t target_enable_power = 0; */
static bool target_power_state = false;
static bool target_boot_state = false;
static const uint16_t blink_period_boot = 1000;
/* static const uint16_t blink_period_fail = 150; */
static uint16_t blink_counter = 0;
static uint16_t target_power_failure = 0;
static volatile uint64_t adc_result = 0;
static volatile uint32_t adc_count = 0;
static uint32_t vdda = 0;
static uint32_t current_report_counter = 0;
static uint8_t  current_power_range = 0;
static bool is_interface_connected = false;

static void disable_power(void) {
    /* Stop ADC */
    adc_power_off(ADC1);
    
    /* Disable output power */
    gpio_set(POWER_OUTPUT_EN_PORT, POWER_OUTPUT_EN_PIN);
    /* Disable green LED */
    gpio_set(LED_CON_GPIO_PORT, LED_CON_GPIO_PIN);

    target_power_state = false;
    target_boot_state = false;
    
    vcdc_println("[INF] power disabled");
}

static void adc_setup_common(void) {
    rcc_periph_clock_enable(RCC_ADC1);
    
    gpio_mode_setup(CURRENT_SENSE_PORT, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, CURRENT_SENSE_PIN);
    
    adc_set_clk_source(ADC1, ADC_CLKSOURCE_ADC);
    adc_calibrate(ADC1);
    adc_set_operation_mode(ADC1, ADC_MODE_SCAN);
    adc_disable_discontinuous_mode(ADC1);
    adc_enable_external_trigger_regular(ADC1, ADC_CFGR1_EXTSEL_VAL(2), ADC_CFGR1_EXTEN_RISING_EDGE);
    adc_set_right_aligned(ADC1);
    adc_disable_temperature_sensor();
    
    /* ~5 us sampling time */
    adc_set_sample_time_on_all_channels(ADC1, ADC_SMPTIME_071DOT5);

    adc_set_resolution(ADC1, ADC_RESOLUTION_12BIT);
    adc_disable_analog_watchdog(ADC1);
}

static void adc_measure_current(void) {
    nvic_enable_irq(NVIC_ADC_COMP_IRQ);
    
    adc_power_off(ADC1);
    
    adc_setup_common();
    
    /* Measurements to be triggered by TIM2 */
    adc_enable_external_trigger_regular(ADC1, ADC_CFGR1_EXTSEL_VAL(2), ADC_CFGR1_EXTEN_RISING_EDGE);
    
    /* No VREF needed */
    adc_disable_vrefint();
    
    /* Data processed by IRQ */
    adc_enable_eoc_interrupt(ADC1);
    
    /* PB0 channel only */
    uint8_t adc_channels = 8;
    adc_set_regular_sequence(ADC1, 1, &adc_channels);
    
    adc_power_on(ADC1);
    
    /* start ADC measurements */
    adc_start_conversion_regular(ADC1);
}

static void adc_measure_vdda(void) {
    adc_power_off(ADC1);
    
    adc_setup_common();
    
    /* ADC will be run once */
    adc_disable_external_trigger_regular(ADC1);
    
    /* Enable VREF */
    adc_enable_vrefint();
    
    /* Disable IRQ */
    adc_disable_eoc_interrupt(ADC1);
    
    /* VREF channel only */
    uint8_t adc_channels = 17;
    adc_set_regular_sequence(ADC1, 1, &adc_channels);
    
    adc_power_on(ADC1);
    
    /* start ADC and wait for it to finish */
    adc_start_conversion_regular(ADC1);
    while (!(adc_eoc(ADC1)));

    /* VREF in ADC counts */
    vdda = adc_read_regular(ADC1);
    
    uint16_t cal_vref = ST_VREFINT_CAL;
    
    /* VDDA in millivolts */
    vdda = (3300 * cal_vref) / vdda;
    
    char vdda_str[5];
    itoa(vdda, vdda_str, 10);
    vcdc_print("[VDD] ");
    vcdc_println(vdda_str);
    
    adc_power_off(ADC1);
}

void adc_comp_isr(void)
{
    uint32_t adc_sample = adc_read_regular(ADC1);
    
    switch (current_power_range) {
        case 3:
            adc_result += adc_sample*206050;
            break;
        case 2:
            adc_result += adc_sample*1005;
            break;
        case 1:
            adc_result += adc_sample*10;
            break;
        default:
            break;
    }
    adc_count++;
    
    if (adc_sample < 20) {
        if (current_power_range == 3) {
            /* make-before-break! */
            gpio_clear(CURRENT_RANGE2_PORT, CURRENT_RANGE2_PIN);
            gpio_set(CURRENT_RANGE3_PORT, CURRENT_RANGE3_PIN);
            current_power_range = 2;
        } else {
            if (current_power_range == 2) {
                /* make-before-break! */
                gpio_clear(CURRENT_RANGE1_PORT, CURRENT_RANGE1_PIN);
                gpio_set(CURRENT_RANGE2_PORT, CURRENT_RANGE2_PIN);
                current_power_range = 1;
            }
        }
    }
}

void exti0_1_isr(void)
{
    exti_reset_request(EXTI0);
    
    if (current_power_range == 1) {
        /* make-before-break! */
        gpio_clear(CURRENT_RANGE2_PORT, CURRENT_RANGE2_PIN);
        gpio_set(CURRENT_RANGE1_PORT, CURRENT_RANGE1_PIN);
        current_power_range = 2;
    } else {
        if (current_power_range == 2) {
            /* make-before-break! */
            gpio_clear(CURRENT_RANGE3_PORT, CURRENT_RANGE3_PIN);
            gpio_set(CURRENT_RANGE2_PORT, CURRENT_RANGE2_PIN);
            current_power_range = 3;
        }
    }
}

/* ticks every 1 ms */
void tim3_isr(void)
{
    /* every 10 ms */
    if ((current_report_counter % 10) == 0) {
        gpio_toggle(TIC33M_LCLK_PORT, TIC33M_LCLK_PIN);
    }
    
    /* calculate average ADC value */
    if (ADC_CR(ADC1) & ADC_CR_ADSTART) {
        current_report_counter++;
        
        /* every 100 ms */
        if (current_report_counter == 100) {
            current_report_counter = 0;
            char cur_str[10] = { 0 };
            /* 100 ms average */
            uint32_t current = adc_result/adc_count;
            adc_result = 0;
            adc_count = 0;
            
            /* convert to mV */
            /* then convert to uA with 50 V/V INA213 scale and 515.125 Ohm shunt */
            /* 1 uA = 0.515 mV on shunt = 25.75 mV on ADC input */
            current = (10 * current * vdda) / (4095*2575);
            itoa(current, cur_str, 10);
            vcdc_print("[CUR] ");
            vcdc_println(cur_str);
            
            tic33m_print(current, 3);
        }
    }
    
    if (timer_get_flag(TIM3, TIM_SR_CC1IF)) {
        /* Clear compare interrupt flag. */
        timer_clear_flag(TIM3, TIM_SR_CC1IF);

        /* Calculate and set the next compare value. */
        uint16_t new_time = timer_get_counter(TIM3) + frequency;

        timer_set_oc_value(TIM3, TIM_OC1, new_time);
    
        if (target_release_reset) {
            target_release_reset--;
        
            if (target_release_reset == 0) {
                /* Release reset */
                gpio_set(nRESET_GPIO_PORT, nRESET_GPIO_PIN);
                vcdc_println("[INF] target reset");
                console_reconfigure(DEFAULT_BAUDRATE, 8, USART_STOPBITS_1, USART_PARITY_NONE);
            }
        }

        /* Blink a LED if target bootloader mode active */
        if (target_boot_state) {
            if (blink_counter == 0) {
                blink_counter = blink_period_boot;
                gpio_toggle(LED_CON_GPIO_PORT, LED_CON_GPIO_PIN);
            }
            blink_counter--;
        }
    
        if (target_release_boot) {
            target_release_boot--;
        
            if (target_release_boot == 0) {
                /* Release boot */
                gpio_clear(TARGET_BOOT_PORT, TARGET_BOOT_PIN);
                vcdc_println("[INF] target bootloader activated");
            }
        }
        
        /* Check if INTERFACE DISCONNECT button is pressed */
        if (gpio_get(TARGET_IFACE_BTN_PORT, TARGET_IFACE_BTN_PIN) == 0) {
            button2_counter++;
        } else {
            /* button released */
            if (button2_counter >= 100) {
                if (is_interface_connected) {
                    gpio_clear(TARGET_IFACE_EN_PORT, TARGET_IFACE_EN_PIN);
                    is_interface_connected = false;
                } else {
                    gpio_set(TARGET_IFACE_EN_PORT, TARGET_IFACE_EN_PIN);
                    is_interface_connected = true;
                }
            }
            button2_counter = 0;
        }
    
        /* Check if POWER button is pressed */
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
                    gpio_clear(POWER_OUTPUT_EN_PORT, POWER_OUTPUT_EN_PIN);
                    /* enable green LED */
                    gpio_clear(LED_CON_GPIO_PORT, LED_CON_GPIO_PIN);

                    target_boot_state = false;
                    target_power_failure = 0;
                    
                    target_release_reset = 250;
                    /* start current monitoring */
                    adc_measure_vdda();
                    adc_measure_current();
                    
                    vcdc_println("[INF] power enabled");
                } else {
                    disable_power();
                }
            }
            button1_counter = 0;
        }
    }
}

/* starts ADC conversion every 10 us */
static void tim2_setup(void)
{
    rcc_periph_clock_enable(RCC_TIM2);

    /* Generates ~10 us clock */
    rcc_periph_reset_pulse(RST_TIM2);
    timer_set_mode(TIM2, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
    timer_set_period(TIM2, 1);
    timer_set_prescaler(TIM2, 225);
    timer_set_clock_division(TIM2, 0x0);
    /* Generate TRGO on every update. */
    timer_set_master_mode(TIM2, TIM_CR2_MMS_UPDATE);
    timer_enable_counter(TIM2);
}

/* starts button and data processing every 1 ms */
static void tim3_setup(void)
{
    /* Enable TIM3 clock. */
    rcc_periph_clock_enable(RCC_TIM3);

    /* Enable TIM3 interrupt. */
    nvic_enable_irq(NVIC_TIM3_IRQ);

    /* Reset TIM3 peripheral to defaults. */
    rcc_periph_reset_pulse(RST_TIM3);

    /* Timer global mode:
     * - No divider
     * - Alignment edge
     * - Direction up
     * (These are actually default values after reset above, so this call
     * is strictly unnecessary, but demos the api for alternative settings)
     */
    timer_set_mode(TIM3, TIM_CR1_CKD_CK_INT,
        TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);

    /*
     * Sets the prescaler to have the timer run at 2kHz -> IRQs at 1 kHz
     */
    timer_set_prescaler(TIM3, (rcc_apb1_frequency / 2000));

    /* Disable preload. */
    timer_disable_preload(TIM3);
    timer_continuous_mode(TIM3);

    /* count full range, as we'll update compare value continuously */
    timer_set_period(TIM3, 65535);

    /* Set the initial output compare value for OC1. */
    timer_set_oc_value(TIM3, TIM_OC1, frequency);

    /* Counter enable. */
    timer_enable_counter(TIM3);

    /* Enable Channel 1 compare interrupt to recalculate compare values */
    timer_enable_irq(TIM3, TIM_DIER_CC1IE);
}

static void button_setup(void) {
    /* Set BOOT0 pin to an input */
    gpio_mode_setup(nBOOT0_GPIO_PORT, GPIO_MODE_INPUT, GPIO_PUPD_NONE, nBOOT0_GPIO_PIN);
    
    gpio_mode_setup(TARGET_IFACE_BTN_PORT, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, TARGET_IFACE_BTN_PIN);
}

#define FLASH_OBP_RDP 0x1FFFF800
#define FLASH_OBP_RDP_KEY 0x55AA
#define FLASH_OBP_USR 0x1FFFF802
#define FLASH_OBP_USR_KEY 0x807F

void gpio_setup(void) {
    /* Enable GPIO clocks. */
    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_GPIOB);
    rcc_periph_clock_enable(RCC_GPIOF);

    button_setup();

    /* disable BOOT0 pin if enabled */
    if (FLASH_OBR & FLASH_OBR_BOOT_SEL) {
        flash_unlock();    
        flash_erase_option_bytes();
    
        /* reset BOOT_SEL bit to disable BOOT0 pin */
        flash_program_option_bytes(FLASH_OBP_USR, FLASH_OBP_USR_KEY);
        /* restore default RDP value */
        flash_program_option_bytes(FLASH_OBP_RDP, FLASH_OBP_RDP_KEY);
    
        flash_lock();
    }

    /* jump to STM DFU bootloader if button is pressed */
    if (gpio_get(nBOOT0_GPIO_PORT, nBOOT0_GPIO_PIN) == 0) {
        DFU_reset_and_jump_to_bootloader();
    }

    /* Setup LEDs as open-drain outputs */
    gpio_set_output_options(LED_CON_GPIO_PORT, GPIO_OTYPE_OD, GPIO_OSPEED_HIGH, LED_CON_GPIO_PIN);
    gpio_mode_setup(LED_CON_GPIO_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, LED_CON_GPIO_PIN);
                
    gpio_set_output_options(LED_RUN_GPIO_PORT, GPIO_OTYPE_OD, GPIO_OSPEED_HIGH, LED_RUN_GPIO_PIN);
    gpio_mode_setup(LED_RUN_GPIO_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, LED_RUN_GPIO_PIN);
                
    gpio_set_output_options(LED_ACT_GPIO_PORT, GPIO_OTYPE_OD, GPIO_OSPEED_HIGH, LED_ACT_GPIO_PIN);
    gpio_mode_setup(LED_ACT_GPIO_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, LED_ACT_GPIO_PIN);

    /* Reset target pin */
    gpio_set_output_options(nRESET_GPIO_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_HIGH, nRESET_GPIO_PIN);
    gpio_mode_setup(nRESET_GPIO_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, nRESET_GPIO_PIN);

    /* Boot from flash target pin */
    gpio_set_output_options(TARGET_BOOT_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_HIGH, TARGET_BOOT_PIN);
    gpio_mode_setup(TARGET_BOOT_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, TARGET_BOOT_PIN);

    /* Target interface enable pin */
    gpio_set_output_options(TARGET_IFACE_EN_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_HIGH, TARGET_IFACE_EN_PIN);
    gpio_mode_setup(TARGET_IFACE_EN_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, TARGET_IFACE_EN_PIN);

    /* Output power enable pin */
    gpio_set_output_options(POWER_OUTPUT_EN_PORT, GPIO_OTYPE_OD, GPIO_OSPEED_HIGH, POWER_OUTPUT_EN_PIN);
    gpio_mode_setup(POWER_OUTPUT_EN_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, POWER_OUTPUT_EN_PIN);
    
    /* Disable output power */
    gpio_set(POWER_OUTPUT_EN_PORT, POWER_OUTPUT_EN_PIN);
    
    /* Overload IRQ */
    gpio_mode_setup(CURRENT_OVERLOAD_PORT, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, CURRENT_OVERLOAD_PIN);
    nvic_enable_irq(CURRENT_OVERLOAD_NVIC);
    exti_select_source(EXTI0, CURRENT_OVERLOAD_PORT);
    exti_set_trigger(CURRENT_OVERLOAD_IRQ, EXTI_TRIGGER_RISING);
    exti_enable_request(CURRENT_OVERLOAD_IRQ);
    
    /* Current ranges select */
    gpio_set_output_options(CURRENT_RANGE1_PORT, GPIO_OTYPE_OD, GPIO_OSPEED_HIGH, CURRENT_RANGE1_PIN);
    gpio_mode_setup(CURRENT_RANGE1_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, CURRENT_RANGE1_PIN);
    gpio_set_output_options(CURRENT_RANGE2_PORT, GPIO_OTYPE_OD, GPIO_OSPEED_HIGH, CURRENT_RANGE2_PIN);
    gpio_mode_setup(CURRENT_RANGE2_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, CURRENT_RANGE2_PIN);
    gpio_set_output_options(CURRENT_RANGE3_PORT, GPIO_OTYPE_OD, GPIO_OSPEED_HIGH, CURRENT_RANGE3_PIN);
    gpio_mode_setup(CURRENT_RANGE3_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, CURRENT_RANGE3_PIN);
    
    /* 2.5A current range by default */
    gpio_set(CURRENT_RANGE1_PORT, CURRENT_RANGE1_PIN);
    gpio_set(CURRENT_RANGE2_PORT, CURRENT_RANGE2_PIN);
    gpio_clear(CURRENT_RANGE3_PORT, CURRENT_RANGE3_PIN);
    current_power_range = 3;

    /* Power on in 100 ms by TIM3 */
    /* button1_counter = 100; */

    /* Setup timers */
    tim2_setup();
    tim3_setup();
    
    gpio_set_output_options(TIC33M_LCLK_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_HIGH, TIC33M_LCLK_PIN);
    gpio_mode_setup(TIC33M_LCLK_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, TIC33M_LCLK_PIN);
    gpio_clear(TIC33M_LCLK_PORT, TIC33M_LCLK_PIN);
    
    tic33m_dev.load_port    = TIC33M_LOAD_PORT;
    tic33m_dev.load_pin     = TIC33M_LOAD_PIN;
    tic33m_dev.din_port     = TIC33M_DIN_PORT;
    tic33m_dev.din_pin      = TIC33M_DIN_PIN;
    tic33m_dev.clk_port     = TIC33M_CLK_PORT;
    tic33m_dev.clk_pin      = TIC33M_CLK_PIN;
    
    tic33m_init(tic33m_dev);
    
    tic33m_print(0, 3);
    
    gpio_set(LED_CON_GPIO_PORT, LED_CON_GPIO_PIN);
}

void target_console_init(void) {
    /* Enable UART clock */
    rcc_periph_clock_enable(CONSOLE_USART_CLOCK);

    /* Setup GPIO pins for UART2 */
    gpio_mode_setup(CONSOLE_USART_GPIO_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, CONSOLE_USART_GPIO_PINS);
    gpio_set_af(CONSOLE_USART_GPIO_PORT, CONSOLE_USART_GPIO_AF, CONSOLE_USART_GPIO_PINS);
}

void led_bit(uint8_t position, bool state) {
    if (position == 0) {
#if !defined(LED_ACT_DISABLED) || !LED_ACT_DISABLED
        if (state) {
            gpio_clear(LED_ACT_GPIO_PORT, LED_ACT_GPIO_PIN);
        } else {
            gpio_set(LED_ACT_GPIO_PORT, LED_ACT_GPIO_PIN);
        }
#endif
    }
#if !defined(LED_RUN_DISABLED) || !LED_RUN_DISABLED
    else if (position == 1) {
        if (state) {
            gpio_clear(LED_RUN_GPIO_PORT, LED_RUN_GPIO_PIN);
        } else {
            gpio_set(LED_RUN_GPIO_PORT, LED_RUN_GPIO_PIN);
        }
    }
#endif
#if !defined(LED_CON_DISABLED) || !LED_CON_DISABLED
    else if (position == 2) {
        if (state) {
            gpio_clear(LED_CON_GPIO_PORT, LED_CON_GPIO_PIN);
        } else {
            gpio_set(LED_CON_GPIO_PORT, LED_CON_GPIO_PIN);
        }
    }
#endif
}

void led_num(uint8_t value) {
#if !defined(LED_CON_DISABLED) || !LED_CON_DISABLED
    if (value & 0x4) {
        gpio_clear(LED_CON_GPIO_PORT, LED_CON_GPIO_PIN);
    } else {
        gpio_set(LED_CON_GPIO_PORT, LED_CON_GPIO_PIN);
    }
#endif

#if !defined(LED_RUN_DISABLED) || !LED_RUN_DISABLED
    if (value & 0x2) {
        gpio_clear(LED_RUN_GPIO_PORT, LED_RUN_GPIO_PIN);
    } else {
        gpio_set(LED_RUN_GPIO_PORT, LED_RUN_GPIO_PIN);
    }
#endif

#if !defined(LED_ACT_DISABLED) || !LED_ACT_DISABLED
    if (value & 0x1) {
        gpio_clear(LED_ACT_GPIO_PORT, LED_ACT_GPIO_PIN);
    } else {
        gpio_set(LED_ACT_GPIO_PORT, LED_ACT_GPIO_PIN);
    }
#endif
}
