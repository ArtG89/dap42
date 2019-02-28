/*
 * Copyright (c) 2016, Devan Lai
 * Copyright (c) 2019, Unwired Devices LLC <info@unwds.com>
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
#include <libopencm3/stm32/dma.h>

#include <string.h>

#include "tick.h"
#include "target.h"
#include "config.h"
#include "console.h"
#include "DAP/CMSIS_DAP_config.h"
#include "DFU/DFU.h"
#include "USB/vcdc.h"
#include "tic33m.h"

#define ENABLE_DEBUG    (0)

#define FLASH_CONFIG_PAGE   31 /* last of 32 pages */
#define FLASH_CONFIG_ADDR   (FLASH_BASE + 1024*FLASH_CONFIG_PAGE)
#define FLASH_CONFIG_MAGIC  0xDEADF00D

#define TIM2_PRESCALER_3US      75
#define TIM2_PRESCALER_10US     225

#define TIM2_PRESCALER          TIM2_PRESCALER_3US

#define DMA_DATA_SIZE   200
static volatile uint16_t dma_data[DMA_DATA_SIZE];

static tic33m tic33m_dev;

/*
 * Divide positive or negative dividend by positive divisor and round
 * to closest integer. Result is undefined for negative divisors and
 * for negative dividends if the divisor variable type is unsigned.
 */
#define DIV_ROUND_CLOSEST(x, divisor)(			\
{							\
	typeof(x) __x = x;				\
	typeof(divisor) __d = divisor;			\
	(((typeof(x))-1) > 0 ||				\
	 ((typeof(divisor))-1) > 0 || (__x) > 0) ?	\
		(((__x) + ((__d) / 2)) / (__d)) :	\
		(((__x) - ((__d) / 2)) / (__d));	\
}							\
)

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

/* 1000 ms blink period */
static const uint32_t blink_period_boot = 1000;

static uint32_t button1_counter = 0;
static uint32_t button2_counter = 0;
static uint32_t button3_counter = 0;
static uint16_t target_release_reset = 0;
static uint16_t target_release_boot = 0;
static uint16_t target_start_measurements = 0;
static bool target_power_state = false;
static bool target_boot_state = false;
static uint32_t blink_counter = 0;
static uint16_t target_power_failure = 0;

static volatile uint32_t adc_voltage_raw = 0;
static volatile uint32_t seconds_passed = 0;
static bool display_counter = false;
static uint32_t display_mode = 0;
static uint32_t vdda = 0;
static uint32_t current_report_counter = 0;
static bool is_interface_connected = false;
static uint64_t energy_accumultated_uwh = 0;
static uint64_t energy_accumultated_uah = 0;
static uint32_t energy_ahr = 0;
static uint32_t energy_whr = 0;
static uint32_t current_max_ua = 0;
static bool update_display = false;
static volatile uint8_t led_act = 0;

static bool banner_displayed = false;
static uint32_t do_calibrate = 0;
static volatile uint32_t cal_voltage = 0;
static volatile int current_power_range = 0;
static volatile uint8_t cmd_int = 0;

#define USB_COMMAND_SIZE    20

static volatile struct {
    uint32_t current[3];
    uint32_t raw_current[3];
    uint32_t count[3];
    uint32_t voltage;
    uint32_t vcount;
} adc_data;

typedef enum {
    SHOW_SECONDS        = 1 << 0,
    SHOW_VOLTAGE        = 1 << 1,
    SHOW_CURRENT        = 1 << 2,
    SHOW_AMPEREHOURS    = 1 << 3,
    SHOW_WATTHOURS      = 1 << 4,
} show_values_t;

typedef enum {
    CMD_INT_CALIBRATE   = 1 << 0,
    CMD_INT_CONSOLEOUT  = 1 << 1,
    CMD_INT_LCDOUT      = 1 << 2,
} internal_commands_t;

static volatile struct {
    uint32_t magic;
    uint32_t voltage_coeff;
    uint32_t period;
    uint32_t show;
    uint32_t baudrate;
} emb_settings;

static void save_settings(void) {
    flash_unlock();
    flash_erase_page(FLASH_CONFIG_ADDR);
    flash_program_word(FLASH_CONFIG_ADDR, FLASH_CONFIG_MAGIC);
    
    uint32_t *settings = (void *)&emb_settings;
    for (unsigned i = 1; i < sizeof(emb_settings)/4; i++) {
        flash_program_word(FLASH_CONFIG_ADDR + i*4, settings[i]);
    }
    flash_lock();
    vcdc_println("[INF] Settings saved");
}

static void disable_power(void) {
    /* Stop TIM2 */
    timer_disable_counter(TIM2);
    
    /* Stop ADC */
    adc_power_off(ADC1);
    
    /* Disable output power */
    gpio_clear(POWER_OUTPUT_EN_PORT, POWER_OUTPUT_EN_PIN);
    /* Disable green LED */
    /* gpio_set(LED_CON_GPIO_PORT, LED_CON_GPIO_PIN); */

    target_power_state = false;
    target_boot_state = false;
    
    gpio_clear(CURRENT_RANGE0_PORT, CURRENT_RANGE0_PIN);
    gpio_clear(CURRENT_RANGE1_PORT, CURRENT_RANGE1_PIN);
    gpio_clear(CURRENT_RANGE2_PORT, CURRENT_RANGE2_PIN);
    
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
    adc_disable_dma(ADC1);
    
    adc_set_resolution(ADC1, ADC_RESOLUTION_12BIT);
    
    /* Enable analog watchdog on PB0 */
    adc_enable_analog_watchdog_on_selected_channel(ADC1, 8);
    ADC_TR1(ADC1) = (ADC_TR1(ADC1) & ~ADC_TR1_HT) | ADC_TR1_HT_VAL(CURRENT_HIGHER_THRESHOLD);
    ADC_TR1(ADC1) = (ADC_TR1(ADC1) & ~ADC_TR1_LT) | ADC_TR1_LT_VAL(CURRENT_LOWER_THRESHOLD);
    adc_enable_watchdog_interrupt(ADC1);
    
    /* Analog watchdog interrupt */
    nvic_set_priority(NVIC_ADC_COMP_IRQ, 0);
    nvic_enable_irq(NVIC_ADC_COMP_IRQ);
    
    /* DMA interrupt with low priority */
    nvic_set_priority(NVIC_DMA1_CHANNEL1_IRQ, 128);
    nvic_enable_irq(NVIC_DMA1_CHANNEL1_IRQ);
    
    rcc_periph_clock_enable(RCC_DMA1);
    
    /* overwrite data in case of overrun */
    ADC_CFGR1(ADC1) |= ADC_CFGR1_OVRMOD;
    
    /* enable DMA circular mode */
    ADC_CFGR1(ADC1) |= ADC_CFGR1_DMACFG;
}

static void adc_measure_current(void) {
    adc_power_off(ADC1);
    
    adc_disable_dma(ADC1);
    adc_calibrate(ADC1);
    
    /* 1 us sampling time */
    adc_set_sample_time_on_all_channels(ADC1, ADC_SMPTIME_007DOT5);
    
    /* Measurements to be triggered by TIM2 */
    adc_enable_external_trigger_regular(ADC1, ADC_CFGR1_EXTSEL_VAL(2), ADC_CFGR1_EXTEN_RISING_EDGE);
    
    /* PB0 channel */
    uint8_t adc_channels = 8;
    adc_set_regular_sequence(ADC1, 1, &adc_channels);

    dma_channel_reset(DMA1, DMA_CHANNEL1);
    
    /* Medium priority. */
    dma_set_priority(DMA1, DMA_CHANNEL1, DMA_CCR_PL_MEDIUM);

    /* ADC is 16 bit */
    dma_set_memory_size(DMA1, DMA_CHANNEL1, DMA_CCR_MSIZE_16BIT);
    dma_set_peripheral_size(DMA1, DMA_CHANNEL1, DMA_CCR_PSIZE_16BIT);

    dma_enable_memory_increment_mode(DMA1, DMA_CHANNEL1);
    dma_disable_peripheral_increment_mode(DMA1, DMA_CHANNEL1);
    
    dma_enable_transfer_complete_interrupt(DMA1, DMA_CHANNEL1);
    dma_enable_half_transfer_interrupt(DMA1, DMA_CHANNEL1);
    
    dma_set_read_from_peripheral(DMA1, DMA_CHANNEL1);
    dma_set_peripheral_address(DMA1, DMA_CHANNEL1, (uint32_t)(&ADC1_DR));
    dma_set_memory_address(DMA1, DMA_CHANNEL1, (uint32_t)(dma_data));
    dma_set_number_of_data(DMA1, DMA_CHANNEL1, DMA_DATA_SIZE);
    dma_enable_circular_mode(DMA1, DMA_CHANNEL1);
    
    dma_enable_channel(DMA1, DMA_CHANNEL1);

    adc_enable_dma(ADC1);
    adc_power_on(ADC1);
    
    /* start ADC measurements */
    adc_start_conversion_regular(ADC1);
    
    /* start TIM2 */
    timer_enable_counter(TIM2);
}

static void adc_measure_vdda(void) {
    adc_power_off(ADC1);
    
    adc_disable_dma(ADC1);
    
    adc_calibrate(ADC1);
    
    /* ~5 us sampling time */
    adc_set_sample_time_on_all_channels(ADC1, ADC_SMPTIME_071DOT5);
    
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
    
    /* Disable VREF */
    adc_disable_vrefint();
    
    char vdda_str[5];
    itoa(vdda, vdda_str, 10);
    vcdc_print("[VDD] ");
    vcdc_println(vdda_str);
}

static uint32_t adc_measure_voltage(void) {
    adc_power_off(ADC1);
    
    adc_disable_dma(ADC1);
    
    adc_calibrate(ADC1);
    
    /* ADC will be run once */
    adc_disable_external_trigger_regular(ADC1);

    /* Disable IRQ */
    adc_disable_eoc_interrupt(ADC1);
    
    /* VREF channel only */
    uint8_t adc_channels = 9;
    adc_set_regular_sequence(ADC1, 1, &adc_channels);
    
    adc_power_on(ADC1);
    
    /* discard old data if present */
    uint32_t adc_read = adc_read_regular(ADC1);
    
    adc_read = 0;
    
    /* start ADC and wait for it to finish */
    for (int i = 0; i < 5; i++) {
        adc_start_conversion_regular(ADC1);
        while (!(adc_eoc(ADC1)));
        adc_read += adc_read_regular(ADC1);
    }

    uint32_t voltage = (emb_settings.voltage_coeff * vdda * adc_read) / (4095*10*5);
    
    return voltage;
}

static void calibrate_voltage(uint32_t cal_value) {
    adc_measure_vdda();
    
    /* PB1 channel only */
    uint8_t adc_channels = 9;
    adc_set_regular_sequence(ADC1, 1, &adc_channels);
    
    adc_power_on(ADC1);
    
    uint32_t voltage_mv = 0;
    
    for (int i = 0; i < 10; i++) {
        /* start ADC and wait for it to finish */
        adc_start_conversion_regular(ADC1);
        while (!(adc_eoc(ADC1)));

        voltage_mv += adc_read_regular(ADC1);
        
        /* 100 us delay between measurements */
        volatile uint32_t k = 4800;
        do {
            k--;
        } while (k);
    }
    voltage_mv = (vdda * voltage_mv) / 4095;
    
    /* coeff is x10 for better accuracy */
    emb_settings.voltage_coeff = (100 * cal_value)/voltage_mv;
    
    char coeff_str[5];
    itoa(emb_settings.voltage_coeff, coeff_str, 10);
    vcdc_print("[CAL] ");
    vcdc_println(coeff_str);
    
    adc_power_off(ADC1);
    
    save_settings();
}

void dma1_channel1_isr(void) {
    /* using local variables for data processing */
    uint32_t data = 0;
    uint32_t range = current_power_range;
    
    /* half transfer event */
    if ((DMA1_ISR & DMA_ISR_HTIF1) != 0) {
        DMA1_IFCR |= DMA_IFCR_CHTIF1;
        for (int i = 0; i < DMA_DATA_SIZE/2; i++) {
            data += dma_data[i];
        }
        data = DIV_ROUND_CLOSEST(data, DMA_DATA_SIZE/2);
        adc_data.raw_current[range] += data;
        adc_data.count[range] += 1;
    }
    
    /* transfer completed event */
    if ((DMA1_ISR & DMA_ISR_TCIF1) != 0) {
        DMA1_IFCR |= DMA_IFCR_CTCIF1;
        for (int i = DMA_DATA_SIZE/2; i < DMA_DATA_SIZE; i++) {
            data += dma_data[i];
            
        }
        data = DIV_ROUND_CLOSEST(data, DMA_DATA_SIZE/2);
        adc_data.raw_current[range] += data;
        adc_data.count[range] += 1;
    }
}

/* analog watchdog interrupt */
void adc_comp_isr(void)
{
    /* stop ADC timer */
    TIM_CR1(TIM2) &= ~TIM_CR1_CEN;
    /* reset ADC watchdog flag */
    ADC_ISR(ADC1) |= ADC_ISR_AWD1;
    /* disable DMA channel */
    DMA_CCR(DMA1, DMA_CHANNEL1) &= ~DMA_CCR_EN;
        
    uint16_t adc_sample = ADC_DR(ADC1);
    
    /* using local variables for data processing */
    int range = current_power_range;
    int delta = 0;

    while ((adc_sample < CURRENT_LOWER_THRESHOLD) ||
           (adc_sample > CURRENT_HIGHER_THRESHOLD))
    {
        if (adc_sample < CURRENT_LOWER_THRESHOLD) {
            if ((range + delta) != 0) {
                if ((range + delta) == 2) {
                    /* make-before-break! */
                    GPIO_BSRR(CURRENT_RANGE1_PORT) = CURRENT_RANGE1_PIN;
                    GPIO_BRR(CURRENT_RANGE2_PORT) = CURRENT_RANGE2_PIN;
                }
                else { /* range 1 */
                    /* disable watchdog low threshold */
                    ADC_TR1(ADC1) = (ADC_TR1(ADC1) & ~ADC_TR1_LT);
                    
                    /* make-before-break! */
                    GPIO_BSRR(CURRENT_RANGE0_PORT) = CURRENT_RANGE0_PIN;
                    GPIO_BRR(CURRENT_RANGE1_PORT) = CURRENT_RANGE1_PIN;
                }
                delta--;
            }
        } else {        /* adc_sample > CURRENT_HIGHER_THRESHOLD */
            if ((range + delta) != 2) {
                if ((range + delta) == 0) {
                    /* make-before-break! */
                    GPIO_BSRR(CURRENT_RANGE1_PORT) = CURRENT_RANGE1_PIN;
                    GPIO_BRR(CURRENT_RANGE0_PORT) = CURRENT_RANGE0_PIN;

                    /* reenable watchdog low threshold */
                    ADC_TR1(ADC1) |= (ADC_TR1(ADC1) & ~ADC_TR1_LT) | ADC_TR1_LT_VAL(CURRENT_LOWER_THRESHOLD);
                } else {
                    /* make-before-break! */
                    GPIO_BSRR(CURRENT_RANGE2_PORT) = CURRENT_RANGE2_PIN;
                    GPIO_BRR(CURRENT_RANGE1_PORT) = CURRENT_RANGE1_PIN;
                }
                delta++;
            } else {
                led_act = 10;
            }
        }
        
        if ((range + delta) == 1) {
            /* restart data acquisition */
            TIM_CR1(TIM2) |= TIM_CR1_CEN;
            
            /* check if another range correction is needed */
            while (!(ADC_ISR(ADC1) & ADC_ISR_EOC));
            TIM_CR1(TIM2) &= ~TIM_CR1_CEN;
            adc_sample = ADC_DR(ADC1);
            
            /* reset ADC watchdog flag */
            ADC_ISR(ADC1) |= ADC_ISR_AWD1;

            /* clear possible pending watchdog interrupt */
            NVIC_ICPR(NVIC_ADC_COMP_IRQ / 32) = (1 << (NVIC_ADC_COMP_IRQ % 32));
        } else {
            break;
        }
    }
        
    uint16_t data[DMA_DATA_SIZE/2];
    uint16_t data_number = DMA_DATA_SIZE - DMA_CNDTR(DMA1, DMA_CHANNEL1);

    /* copy data to temporary array to process them after acquisition restart */
    if (data_number > DMA_DATA_SIZE/2) {
        memcpy((void *)data, (void *)&dma_data[DMA_DATA_SIZE/2], sizeof(data[0])*(data_number - DMA_DATA_SIZE/2));
        data_number -= DMA_DATA_SIZE/2;
    } else {
        memcpy((void *)data, (void *)dma_data, sizeof(data[0])*data_number);
    }

    /* reset DMA channel */
    DMA_CNDTR(DMA1, DMA_CHANNEL1) = DMA_DATA_SIZE;
    DMA_CCR(DMA1, DMA_CHANNEL1) |= DMA_CCR_EN;
    
    current_power_range += delta;
    
    /* restart acquisition timer */
    TIM_CR1(TIM2) |= TIM_CR1_CEN;
    
    uint32_t raw_data = 0;
    for (int i = 0; i < data_number; i++) {
        raw_data += data[i];
    }
    adc_data.raw_current[range] += raw_data;
    adc_data.count[range] += data_number;
}

static void console_command_parser(uint8_t *usb_command) {
    const char *help_period = "period <ms> - set period in milliseconds, 10 to 1000";
    const char *help_iface = "iface <on|off> - enable/disable UART and SWD interfaces";
    const char *help_power = "power <on|off> - enable/disable onboard DC/DC";
    const char *help_display = "display <N> - set display mode by number";
    const char *help_calibrate = "calibrate <mV> - calibrate voltage divider";
    const char *help_show = "show <SEC|VOL|CUR|AHR|WHR> - report values";
    const char *help_hide = "hide <SEC|VOL|CUR|AHR|WHR> - don't report values";
    const char *help_baudrate = "baudrate <bps> - set target console baudrate";

    int cmdlen;

    if (memcmp((char *)usb_command, "help", strlen("help")) == 0) {
        vcdc_println(help_period);
        vcdc_println(help_iface);
        vcdc_app_update();
        
        vcdc_println(help_power);
        vcdc_println(help_display);
        vcdc_app_update();
        
        vcdc_println(help_calibrate);
        vcdc_println(help_show);
        vcdc_app_update();
        
        vcdc_println(help_baudrate);
        vcdc_println(help_hide);
        vcdc_app_update();
    }
    else
    if (memcmp((char *)usb_command, "period ", cmdlen = strlen("period ")) == 0) {
        int period = strtol((char *)&usb_command[cmdlen], NULL, 10);

        if ((period >= 10) && (period <= 1000)) {
            vcdc_print("[INF] Period is ");
            char str[10];
            itoa(period, str, 10);
            vcdc_print(str);
            vcdc_println(" ms");
            
            emb_settings.period = period;
            save_settings();
        }
        else {
            vcdc_println(help_period);
            vcdc_send_buffer_space();
        }
    }
    if (memcmp((char *)usb_command, "baudrate ", cmdlen = strlen("baudrate ")) == 0) {
        int baudrate = strtol((char *)&usb_command[cmdlen], NULL, 10);

        if (baudrate != 0) {
            vcdc_print("[INF] Baudrate is ");
            char str[10];
            itoa(baudrate, str, 10);
            vcdc_print(str);
            vcdc_println(" bps");
            
            console_reconfigure(baudrate, 8, USART_STOPBITS_1, USART_PARITY_NONE);
            
            emb_settings.baudrate = baudrate;
            save_settings();
        }
        else {
            vcdc_println(help_baudrate);
            vcdc_send_buffer_space();
        }
    }
    else
    if (memcmp((char *)usb_command, "iface ", cmdlen = strlen("iface ")) == 0) {
        if (memcmp((char *)&usb_command[cmdlen], "on", 2) == 0) {
            is_interface_connected = false;
            button2_counter = 100;
        }
        else
        if (memcmp((char *)&usb_command[cmdlen], "off", 3) == 0) {
            is_interface_connected = true;
            button2_counter = 100;
        }
        else {
            vcdc_println(help_iface);
            vcdc_send_buffer_space();
        }
    }
    else
    if (memcmp((char *)usb_command, "power ", cmdlen = strlen("power ")) == 0) {
        if (memcmp((char *)&usb_command[cmdlen], "on", 2) == 0) {
            target_power_state = false;
            button1_counter = 100;
        }
        else
        if (memcmp((char *)&usb_command[cmdlen], "off", 3) == 0) {
            target_power_state = true;
            button1_counter = 100;
        }
        else
        {
            vcdc_println(help_power);
            vcdc_send_buffer_space();
        }
    }
    else
    if (memcmp((char *)usb_command, "display ", cmdlen = strlen("display ")) == 0) {
        display_mode = strtol((char *)&usb_command[cmdlen], NULL, 10);
    }
    else
    if (memcmp((char *)usb_command, "calibrate ", cmdlen = strlen("calibrate ")) == 0) {
        cal_voltage = strtol((char *)&usb_command[cmdlen], NULL, 10);
        if ((cal_voltage > 0) && (cal_voltage < 20000)) {
            /* Enable DC/DC power */
            gpio_set(POWER_OUTPUT_EN_PORT, POWER_OUTPUT_EN_PIN);
            /* delay before calibration 1500 ms */
            do_calibrate = 1500;
            
            char str[10] = { };
            strcpy(str, "CAL ");
            itoa(cal_voltage, &str[strlen(str)], 10);
            
            tic33m_display_string(&tic33m_dev, str, strlen(str));
            current_report_counter = 1;
        } else {
            vcdc_println(help_calibrate);
        }
    }
    else
    if (memcmp((char *)usb_command, "show ", cmdlen = strlen("show ")) == 0) {
        if (memcmp((char *)&usb_command[cmdlen], "SEC", 2) == 0) {
            if (!(emb_settings.show & SHOW_SECONDS)) {
                emb_settings.show |= SHOW_SECONDS;
                save_settings();
            }
        }
        else
        if (memcmp((char *)&usb_command[cmdlen], "VOL", 3) == 0) {
            if (!(emb_settings.show & SHOW_VOLTAGE)) {
                emb_settings.show |= SHOW_VOLTAGE;
                save_settings();
            }
        }
        else
        if (memcmp((char *)&usb_command[cmdlen], "CUR", 3) == 0) {
            if (!(emb_settings.show & SHOW_CURRENT)) {
                emb_settings.show |= SHOW_CURRENT;
                save_settings();
            }
        }
        else
        if (memcmp((char *)&usb_command[cmdlen], "AHR", 3) == 0) {
            if (!(emb_settings.show & SHOW_AMPEREHOURS)) {
                emb_settings.show |= SHOW_AMPEREHOURS;
                save_settings();
            }
        }
        else
        if (memcmp((char *)&usb_command[cmdlen], "WHR", 3) == 0) {
            if (!(emb_settings.show & SHOW_WATTHOURS)) {
                emb_settings.show |= SHOW_WATTHOURS;
                save_settings();
            }
        }
        else
        {
            vcdc_println(help_show);
            vcdc_send_buffer_space();
        }
    }
    else
    if (memcmp((char *)usb_command, "hide ", cmdlen = strlen("hide ")) == 0) {
        if (memcmp((char *)&usb_command[cmdlen], "SEC", 2) == 0) {
            if (emb_settings.show & SHOW_SECONDS) {
                emb_settings.show &= ~SHOW_SECONDS;
                save_settings();
            }
        }
        else
        if (memcmp((char *)&usb_command[cmdlen], "VOL", 3) == 0) {
            if (emb_settings.show & SHOW_VOLTAGE) {
                emb_settings.show &= ~SHOW_VOLTAGE;
                save_settings();
            }
        }
        else
        if (memcmp((char *)&usb_command[cmdlen], "CUR", 3) == 0) {
            if (emb_settings.show & SHOW_CURRENT) {
                emb_settings.show &= ~SHOW_CURRENT;
                save_settings();
            }
        }
        else
        if (memcmp((char *)&usb_command[cmdlen], "AHR", 3) == 0) {
            if (emb_settings.show & SHOW_AMPEREHOURS) {
                emb_settings.show &= ~SHOW_AMPEREHOURS;
                save_settings();
            }
        }
        else
        if (memcmp((char *)&usb_command[cmdlen], "WHR", 3) == 0) {
            if (emb_settings.show & SHOW_WATTHOURS) {
                emb_settings.show &= ~SHOW_WATTHOURS;
                save_settings();
            }
        }
        else
        {
            vcdc_println(help_hide);
            vcdc_send_buffer_space();
        }
    }
}

/* ticks every 1 ms */
void tim3_isr(void)
{
    current_report_counter++;
    
    /* every 10 ms */
    if (current_report_counter && (current_report_counter % 10 == 0)) {
        tic33m_lclk(&tic33m_dev);
    }
    
    /* every 100 ms */
    if (current_report_counter && (current_report_counter % 100 == 0)) {
        if (led_act) {
            if (--led_act) {
                gpio_clear(LED_ACT_GPIO_PORT, LED_ACT_GPIO_PIN);
            } else {
                gpio_set(LED_ACT_GPIO_PORT, LED_ACT_GPIO_PIN);
            }
        }

        /* console command parser */
        static uint8_t usb_command[USB_COMMAND_SIZE];
        if (vcdc_recv_buffered(usb_command, USB_COMMAND_SIZE) != 0) {
            console_command_parser(usb_command);
        }
    }
    
    if (do_calibrate) {
        do_calibrate--;
        
        if (!do_calibrate) {
            /* calibration at 3.0V */
            cmd_int |= CMD_INT_CALIBRATE;
        }
    }
    
    /* every 'emb_settings.period' ms */
    if (current_report_counter &&  (current_report_counter % emb_settings.period == 0)) {
        
        /* only once */
        if (!banner_displayed) {
            #if defined(BANNER_STR1)
                vcdc_println(BANNER_STR1);
            #endif
            #if defined(BANNER_STR2)
                vcdc_println(BANNER_STR2);
            #endif
            #if defined(BANNER_STR3)
                vcdc_println(BANNER_STR3);
            #endif
            banner_displayed = true;
            
            if (emb_settings.magic != FLASH_CONFIG_MAGIC) {
                vcdc_println("[ERR] Configuration NOT loaded");
            } else {
                vcdc_println("[INF] Configuration loaded");
            }
            
            char str[10];
            itoa(emb_settings.period, str, 10);
            vcdc_print("[INF] Period is ");
            vcdc_print(str);
            vcdc_println(" ms");
            
            vcdc_send_buffer_space();
        }

        /* calculate average ADC value */
        if (target_power_state && !target_start_measurements) {
            /* measure voltage */
            timer_disable_counter(TIM2);
            adc_data.voltage = adc_measure_voltage();

            /* convert to millivolts x10 */
            for (int i = 0; i < 3; i++) {
                if (adc_data.count[i]) {
                    adc_data.current[i] = (vdda * (DIV_ROUND_CLOSEST(10*adc_data.raw_current[i], adc_data.count[i]))) / 4095;
#if ENABLE_DEBUG
                    char str[10];
                    vcdc_print("RAW ADC: ");
                    itoa(i, str, 10);
                    vcdc_print(str);
                    vcdc_print(": ");
                    
                    itoa(adc_data.raw_current[i], str, 10);
                    vcdc_print(str);
                    vcdc_print(" / ");

                    itoa(adc_data.count[i], str, 10);
                    vcdc_print(str);
                    vcdc_print(" = ");

                    itoa(DIV_ROUND_CLOSEST(adc_data.raw_current[i], adc_data.count[i]), str, 10);
                    vcdc_println(str);
#endif
                } else {
                    adc_data.current[i] = 0;
                }
                adc_data.raw_current[i] = 0;
                adc_data.count[i] = 0;
            }
            
            adc_measure_current();
            
            cmd_int |= CMD_INT_CONSOLEOUT;
        }
    }
    
    if (target_release_reset) {
        if (--target_release_reset == 0) {
            /* Release reset */
#if nRESET_GPIO_INVERT
            gpio_clear(nRESET_GPIO_PORT, nRESET_GPIO_PIN);
#else
            gpio_set(nRESET_GPIO_PORT, nRESET_GPIO_PIN);
#endif
            vcdc_println("[INF] target reset");
        }
    }
    
    if (target_start_measurements) {
        if (--target_start_measurements == 0) {
            adc_measure_vdda();
            adc_measure_current();
        }
    }

    /* Blink a LED if target bootloader mode active */
    if (target_boot_state) {
        if (--blink_counter == 0) {
            blink_counter = blink_period_boot;
            gpio_toggle(LED_CON_GPIO_PORT, LED_CON_GPIO_PIN);
        }
    }

    if (target_release_boot) {
        if (--target_release_boot == 0) {
            /* Release boot */
            gpio_set(TARGET_BOOT_PORT, TARGET_BOOT_PIN); /* inverted */
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
    
    /* Check if DISPLAY MODE button is pressed */
    if (gpio_get(DISPLAY_MODE_BTN_PORT, DISPLAY_MODE_BTN_PIN) == 0) {
        if (++button3_counter >= 5000) {
            /* Enable DC/DC power */
            gpio_set(POWER_OUTPUT_EN_PORT, POWER_OUTPUT_EN_PIN);
            /* delay before calibration 1500 ms */
            do_calibrate = 1500;
            
            /* calibrate @ 3000 mV */
            cal_voltage = 3000;
            tic33m_display_string(&tic33m_dev, "CAL 3000", strlen("CAL 3000"));
            current_report_counter = 1;
        }
    } else {
        /* button released */
        if ((button3_counter >= 100) && (button3_counter < 5000)) {
            if (++display_mode == 3) {
                display_mode = 0;
            }
            update_display = true;
        }
        button3_counter = 0;
    } 

    /* Check if POWER button is pressed */
    if (gpio_get(nBOOT0_GPIO_PORT, nBOOT0_GPIO_PIN) != 0) {
        if ((++button1_counter == 1000) && target_power_state) {
            /* 1000 ms long press */
            target_boot_state = !target_boot_state;
            /* Reset target */
#if nRESET_GPIO_INVERT
            gpio_set(nRESET_GPIO_PORT, nRESET_GPIO_PIN);
#else
            gpio_clear(nRESET_GPIO_PORT, nRESET_GPIO_PIN);
#endif
        
            if (target_boot_state) {            
                /* Boot from system memory */
                gpio_clear(TARGET_BOOT_PORT, TARGET_BOOT_PIN); /* inverted */
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
            
            /* Enable green LED */
            gpio_clear(LED_CON_GPIO_PORT, LED_CON_GPIO_PIN);
        
            /* Toggle power */
            if (target_power_state) {
                /* Reset target */
#if nRESET_GPIO_INVERT
                gpio_set(nRESET_GPIO_PORT, nRESET_GPIO_PIN);
#else
                gpio_clear(nRESET_GPIO_PORT, nRESET_GPIO_PIN);
#endif
                /* Set boot from flash */
                gpio_set(TARGET_BOOT_PORT, TARGET_BOOT_PIN); /* inverted */
                /* Reset accumulated energy */
                energy_accumultated_uah = 0;
                energy_accumultated_uwh = 0;
                seconds_passed = 0;
                /* Enable DC/DC power */
                gpio_set(POWER_OUTPUT_EN_PORT, POWER_OUTPUT_EN_PIN);
                /* Enable current shunt (range 2 by default) */
                gpio_clear(CURRENT_RANGE0_PORT, CURRENT_RANGE0_PIN);
                gpio_set(CURRENT_RANGE1_PORT, CURRENT_RANGE1_PIN);
                gpio_clear(CURRENT_RANGE2_PORT, CURRENT_RANGE2_PIN);
                current_power_range = 1;
                current_max_ua = 0;
                
                /* enable green LED */
                gpio_clear(LED_CON_GPIO_PORT, LED_CON_GPIO_PIN);

                target_boot_state = false;
                target_power_failure = 0;
                
                target_release_reset = 100;
                
                /* start current monitoring in 100 ms */
                target_start_measurements = 100;
                
                vcdc_println("[INF] power enabled");
            } else {
                disable_power();
            }
        }
        button1_counter = 0;
    }
    
    /* every second */
    if (current_report_counter && (current_report_counter % 1000 == 0)) {
        if (target_power_state) {
            seconds_passed += 1;
        }
    }
    
    /* every 2 seconds */
    if (current_report_counter == 2000) {
        cmd_int |= CMD_INT_LCDOUT;
        current_report_counter = 0;
    }
    
    if (update_display) {
        cmd_int |= CMD_INT_LCDOUT;
    }

    if (timer_get_flag(TIM3, TIM_SR_CC1IF)) {
        /* Clear compare interrupt flag. */
        timer_clear_flag(TIM3, TIM_SR_CC1IF);

        /* Calculate and set the next compare value. */
        uint16_t new_time = timer_get_counter(TIM3) + frequency;

        timer_set_oc_value(TIM3, TIM_OC1, new_time);
    }
}

void user_activity(void) {
    if (cmd_int & CMD_INT_CALIBRATE) {
        cmd_int &= ~CMD_INT_CALIBRATE;
        calibrate_voltage(cal_voltage);
        disable_power();
    }
    
    char cur_str[10] = { 0 };
    
    if (cmd_int & CMD_INT_CONSOLEOUT) {
        cmd_int &= ~CMD_INT_CONSOLEOUT;

        uint32_t current = 0;
        uint32_t n = 0;
        if (adc_data.current[0]) {
            n++;
        }
        if (adc_data.current[1]) {
            n++;
        }
        if (adc_data.current[2]) {
            n++;
        }
        
        current = (DIV_ROUND_CLOSEST(10 * adc_data.current[0], 102)) +
                  (100 * DIV_ROUND_CLOSEST(10 * adc_data.current[1], 102)) + 
                  (100 * 100 * DIV_ROUND_CLOSEST(10 * adc_data.current[2], 102));
        
        if (n > 1) {
            current /= n;
        }
        
        if (current > current_max_ua) {
            current_max_ua = current;
        }
        
        /* energy */
        energy_accumultated_uah += current;
        energy_accumultated_uwh += DIV_ROUND_CLOSEST(current * adc_data.voltage, 1000);
        
        /* convert to microampere-hours */
        energy_ahr = energy_accumultated_uah/(3600*10*(1000/emb_settings.period));
        /* convert to microwatt-hours */
        energy_whr = energy_accumultated_uwh/(3600*10*(1000/emb_settings.period));

        if (emb_settings.show & SHOW_SECONDS) {
            itoa(seconds_passed, cur_str, 10);
            vcdc_print("[SEC] ");
            vcdc_println(cur_str);
        }
        
        if (emb_settings.show & SHOW_VOLTAGE) {
            itoa(adc_data.voltage, cur_str, 10);
            vcdc_print("[VOL] ");
            vcdc_println(cur_str);
        }

        if (emb_settings.show & SHOW_CURRENT) {
            vcdc_print("[CUR] ");
            itoa(DIV_ROUND_CLOSEST(current, 10), cur_str, 10);
            vcdc_print(cur_str);
            itoa(current % 10, cur_str, 10);
            vcdc_print(".");
#if ENABLE_DEBUG
            vcdc_print(cur_str);

            itoa(adc_data.current[0], cur_str, 10);
            vcdc_print(" (");
            vcdc_print(cur_str);
            
            itoa(adc_data.current[1], cur_str, 10);
            vcdc_print(" - ");
            vcdc_print(cur_str);
            
            itoa(adc_data.current[2], cur_str, 10);
            vcdc_print(" - ");
            vcdc_print(cur_str);
            vcdc_println(")");
#else
            vcdc_println(cur_str);
#endif
        }
        
        if (emb_settings.show & SHOW_AMPEREHOURS) {
            itoa(energy_ahr, cur_str, 10);
            vcdc_print("[AHR] ");
            vcdc_println(cur_str);
        }
        
        if (emb_settings.show & SHOW_WATTHOURS) {
            itoa(energy_whr, cur_str, 10);
            vcdc_print("[WHR] ");
            vcdc_println(cur_str);
        }
    }

    if (cmd_int & CMD_INT_LCDOUT) {
        cmd_int &= ~CMD_INT_LCDOUT;
        if (display_counter || update_display) {
            char lcdtext[11] = { };
            memset(lcdtext, ' ', 10);
            
            int precision;
            uint32_t value;
            
            switch (display_mode) {
                case 1:
                    /* maximum current  */
                    value = DIV_ROUND_CLOSEST(current_max_ua, 10);
                    lcdtext[0] = 'I';
                    precision = 0;
                    break;
                case 2:
                    /* microwatt-hours */
                    value = energy_whr;
                    lcdtext[0] = 'P';
                    precision = 3;
                    break;
                default:
                    /* microampere-hours */
                    value = energy_ahr;
                    lcdtext[0] = 'C';
                    precision = 3;
                    break;
            }
            
            itoa(value, cur_str, 10);
            int len = strlen(cur_str);

            if ((len > 8) || ((len > 7) && (precision > 0))) {
                strcpy(&lcdtext[1], "       OL");
            } else {
                /* add forwarding zeros if needed */
                if (len <= precision) {
                    memmove(&cur_str[1 + precision - len], cur_str, len + 1);
                    memset(cur_str, '0', 1 + precision - len);
                    len = strlen(cur_str);
                }
                memcpy(&lcdtext[9 - len], cur_str, len - precision + 1);

                /* add decimal point if needed */
                if (precision) {
                    lcdtext[9 - precision] = '.';
                    memcpy(&lcdtext[9 - precision + 1], &cur_str[len - precision], precision + 1);
                }
            }

            tic33m_display_string(&tic33m_dev, lcdtext, strlen(lcdtext));
            
            if (!update_display) {
                display_counter = false;
            }
        } else {
            /* display time */
            tic33m_display_time(&tic33m_dev, seconds_passed);
            display_counter = true;
        }
        update_display = false;
    }
}

/* starts ADC conversion every ~3 us */
static void tim2_setup(void)
{
    rcc_periph_clock_enable(RCC_TIM2);

    /* Generates ~3 us clock */
    rcc_periph_reset_pulse(RST_TIM2);
    timer_set_mode(TIM2, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
    timer_set_period(TIM2, 1);
    timer_set_prescaler(TIM2, TIM2_PRESCALER);
    timer_set_clock_division(TIM2, 0x0);
    /* Generate TRGO on every update. */
    timer_set_master_mode(TIM2, TIM_CR2_MMS_UPDATE);
}

/* starts button and data processing every 1 ms */
static void tim3_setup(void)
{
    /* Enable TIM3 clock. */
    rcc_periph_clock_enable(RCC_TIM3);

    /* TIM3 is a lowest priority interrupt */
    nvic_set_priority(NVIC_TIM3_IRQ, 192);
    
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
     * Sets the prescaler to have the timer run at 1kHz
     */
    timer_set_prescaler(TIM3, (rcc_apb1_frequency / 1000));

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
    gpio_mode_setup(DISPLAY_MODE_BTN_PORT, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, DISPLAY_MODE_BTN_PIN);
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
    /* Enable interface by default */
    gpio_set(TARGET_IFACE_EN_PORT, TARGET_IFACE_EN_PIN);
    is_interface_connected = true;

    /* Output power enable pin */
    gpio_set_output_options(POWER_OUTPUT_EN_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_HIGH, POWER_OUTPUT_EN_PIN);
    gpio_mode_setup(POWER_OUTPUT_EN_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, POWER_OUTPUT_EN_PIN);
    
    /* Disable output power */
    gpio_clear(POWER_OUTPUT_EN_PORT, POWER_OUTPUT_EN_PIN);
    
    rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_SYSCFGCOMPEN);
    
    /* Current ranges select */
    gpio_set_output_options(CURRENT_RANGE0_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_HIGH, CURRENT_RANGE0_PIN);
    gpio_mode_setup(CURRENT_RANGE0_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, CURRENT_RANGE0_PIN);
    gpio_set_output_options(CURRENT_RANGE1_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_HIGH, CURRENT_RANGE1_PIN);
    gpio_mode_setup(CURRENT_RANGE1_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, CURRENT_RANGE1_PIN);
    gpio_set_output_options(CURRENT_RANGE2_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_HIGH, CURRENT_RANGE2_PIN);
    gpio_mode_setup(CURRENT_RANGE2_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, CURRENT_RANGE2_PIN);
    
    /* Disable all shunts */
    gpio_clear(CURRENT_RANGE0_PORT, CURRENT_RANGE0_PIN);
    gpio_clear(CURRENT_RANGE1_PORT, CURRENT_RANGE1_PIN);
    gpio_clear(CURRENT_RANGE2_PORT, CURRENT_RANGE2_PIN);
    current_power_range = 0;

    /* Power on in 100 ms by TIM3 */
    /* button1_counter = 100; */

    /* Setup timers */
    tim2_setup();
    tim3_setup();
    
    /* Setup ADC */
    adc_setup_common();
    
    memcpy((void *)&emb_settings, (void *)FLASH_CONFIG_ADDR, sizeof(emb_settings));
    if (emb_settings.magic != FLASH_CONFIG_MAGIC) {
        emb_settings.voltage_coeff = 0;
        emb_settings.period = 100;
        emb_settings.baudrate = DEFAULT_BAUDRATE;
    }
    
    if ((emb_settings.period < 10) || (emb_settings.period >= 1000)) {
        emb_settings.period = 100;
    }
    
    if ((emb_settings.baudrate == 0) || (emb_settings.baudrate > 1000000)) {
        emb_settings.baudrate = DEFAULT_BAUDRATE;
    }
    
    console_reconfigure(emb_settings.baudrate, 8, USART_STOPBITS_1, USART_PARITY_NONE);
    
    gpio_set_output_options(TIC33M_LCLK_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_HIGH, TIC33M_LCLK_PIN);
    gpio_mode_setup(TIC33M_LCLK_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, TIC33M_LCLK_PIN);
    gpio_clear(TIC33M_LCLK_PORT, TIC33M_LCLK_PIN);
    
    tic33m_dev.load_port    = TIC33M_LOAD_PORT;
    tic33m_dev.load_pin     = TIC33M_LOAD_PIN;
    tic33m_dev.din_port     = TIC33M_DIN_PORT;
    tic33m_dev.din_pin      = TIC33M_DIN_PIN;
    tic33m_dev.clk_port     = TIC33M_CLK_PORT;
    tic33m_dev.clk_pin      = TIC33M_CLK_PIN;
    tic33m_dev.lclk_port    = TIC33M_LCLK_PORT;
    tic33m_dev.lclk_pin     = TIC33M_LCLK_PIN;
    
    tic33m_init(&tic33m_dev);
    
    int len = strlen(FW_VERSION);
    if (emb_settings.magic != FLASH_CONFIG_MAGIC) {
        tic33m_display_string(&tic33m_dev, "ERR   " FW_VERSION, 6 + len);
    } else {
        tic33m_display_string(&tic33m_dev, "CAL   " FW_VERSION, 6 + len);
    }

    /* enable green LED */
    gpio_clear(LED_CON_GPIO_PORT, LED_CON_GPIO_PIN);
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
