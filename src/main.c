// Simple RTC code that will toggle a pin every second

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "nrf.h"
#include "bsp.h"

#include "app_uart.h"
#include "app_error.h"

#include "nrf_delay.h"

#include "nrf_clock.h"
#include "nrf_rtc.h"
#include "nrf_gpio.h"

#include "nrf_saadc.h"

// Autocompletion of nrf specific symbols and eldoc work much better when those
// files are included on top level
// This is not heeded if your code is using only SDK code and not accessing
// CMSIS stuff directly
#ifdef NRF51
#include "nrf51.h"
#include "core_cm0.h"
#endif

#ifdef NRF52
#include "nrf52.h"
#include "core_cm4.h"
#endif

#include "uart.h"

#define PIN_LED 19 // LED1
#define ADC_RESULT_BUFFER_SIZE 1

static unsigned irq_counter = 0;

#define NRF_SAADC_EVENTS_DONE offsetof(NRF_SAADC_Type, EVENTS_STARTED)

void RTC0_IRQHandler(void) {
    nrf_rtc_event_clear(NRF_RTC0,NRF_RTC_EVENT_TICK);
    irq_counter++;
    // every 8 IRQs, we toggle the led -> 1Hz rate
    if(irq_counter %8 == 0) nrf_gpio_pin_toggle(PIN_LED);
}

int main(void)
{

    nrf_saadc_value_t results[ADC_RESULT_BUFFER_SIZE];
    nrf_gpio_cfg_output(PIN_LED);
    nrf_gpio_pin_set(PIN_LED); // LED is off

    uint32_t err_code;
    const app_uart_comm_params_t comm_params =
        {
            RX_PIN_NUMBER,
            TX_PIN_NUMBER,
            RTS_PIN_NUMBER,
            CTS_PIN_NUMBER,
            APP_UART_FLOW_CONTROL_ENABLED,
            false,
            UART_BAUDRATE_BAUDRATE_Baud38400
        };

    APP_UART_FIFO_INIT(&comm_params,
                       UART_RX_BUF_SIZE,
                       UART_TX_BUF_SIZE,
                       uart_error_handle,
                       APP_IRQ_PRIORITY_LOW,
                       err_code);

    APP_ERROR_CHECK(err_code);


    printf("Hello world ! (%s)\n",__DATE__);






    // Irq setup
    NVIC_SetPriority(RTC0_IRQn, 15); // Lowes priority
    NVIC_ClearPendingIRQ(RTC0_IRQn);
    NVIC_EnableIRQ(RTC0_IRQn);

    // Start LFCLK clock
    nrf_clock_lf_src_set(NRF_CLOCK_LF_SRC_RC); // 32KHz RC
    nrf_clock_task_trigger(NRF_CLOCK_TASK_LFCLKSTART);


    // Set prescaler to the max value (12-bit)
    // -> 8Hz counter frequency
    nrf_rtc_prescaler_set(NRF_RTC0,(1<<12) -1);
    nrf_rtc_event_enable(NRF_RTC0, NRF_RTC_INT_TICK_MASK); /* yes INT mask must be used here */
    nrf_rtc_int_enable(NRF_RTC0,NRF_RTC_INT_TICK_MASK);
    nrf_rtc_task_trigger(NRF_RTC0,NRF_RTC_TASK_START);

    nrf_saadc_channel_config_t config;
    config.acq_time = NRF_SAADC_ACQTIME_20US;
    config.gain = NRF_SAADC_GAIN1;
    config.mode = NRF_SAADC_MODE_SINGLE_ENDED;
    config.pin_p = NRF_SAADC_INPUT_AIN0;
    config.pin_n = NRF_SAADC_INPUT_AIN0; // Single ended -> should be ground to zero
    config.reference = NRF_SAADC_REFERENCE_INTERNAL;
    config.resistor_p = NRF_SAADC_RESISTOR_DISABLED;
    config.resistor_n = NRF_SAADC_RESISTOR_DISABLED;

    nrf_saadc_enable();
    nrf_saadc_channel_init(NRF_SAADC_INPUT_AIN0, &config);
    nrf_saadc_buffer_init(results,ADC_RESULT_BUFFER_SIZE);
    nrf_saadc_task_trigger(NRF_SAADC_TASK_START);
    for (int i=0; i < ADC_RESULT_BUFFER_SIZE;i++) results[i] = 0;


    //for(int i=0;i<ADC_RESULT_BUFFER_SIZE;i++) {
        nrf_saadc_task_trigger(NRF_SAADC_TASK_SAMPLE);

        //};
        printf("One sample !\n");
        //while(!nrf_saadc_event_check(NRF_SAADC_EVENTS_DONE));


    for (int i=0; i < ADC_RESULT_BUFFER_SIZE;i++) {
        printf("ADC result[%i] = 0x%08x\n",i,(unsigned)results[i]);
    }


    while(1) {
        __WFI();
    };
}
