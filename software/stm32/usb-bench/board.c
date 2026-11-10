#include "bench_i2c.h"
#include "bench_sensors.h"
#include "replay.h"
#include "stm32h7xx.h"
#include "tusb.h"
#include <stdio.h>
#include <string.h>

uint32_t SystemCoreClock = 64000000;
static volatile uint32_t milliseconds;
static char report[1024];
static char sensor_report[128];
extern uint32_t stack_top, bss_start, bss_end;
void start(void);
static void tick(void) { ++milliseconds; }
static void usb_irq(void) { tud_int_handler(0); }
uint32_t tusb_time_millis_api(void) { return milliseconds; }

__attribute__((section(".vectors"), used))
static const uintptr_t vectors[16 + OTG_FS_IRQn + 1] = {
    [0] = (uintptr_t)&stack_top, [1] = (uintptr_t)start,
    [15] = (uintptr_t)tick, [16 + OTG_FS_IRQn] = (uintptr_t)usb_irq
};

static void init_clock(void) {
    RCC->CR |= RCC_CR_HSION;
    while (!(RCC->CR & RCC_CR_HSIRDY)) {}
    RCC->CFGR &= ~RCC_CFGR_SW;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_HSI) {}
    RCC->CR &= ~RCC_CR_HSIDIV;
    RCC->D1CFGR &= ~(RCC_D1CFGR_D1CPRE | RCC_D1CFGR_HPRE);
    RCC->CR |= RCC_CR_HSI48ON;
    while (!(RCC->CR & RCC_CR_HSI48RDY)) {}
    RCC->D2CCIP2R |= RCC_D2CCIP2R_USBSEL;  // HSI48 to USB2.
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->LAR = 0xc5acce55;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    SysTick_Config(SystemCoreClock / 1000);
}

static void init_pins(void) {
    RCC->AHB4ENR |= RCC_AHB4ENR_GPIOAEN | RCC_AHB4ENR_GPIOCEN;
    (void)RCC->AHB4ENR;
    GPIOC->BSRR = 1u << 13;
    GPIOC->MODER = (GPIOC->MODER & ~(3u << 26)) | (1u << 26);
    for (unsigned pin = 11; pin <= 12; ++pin) {
        unsigned shift = (pin - 8) * 4;
        GPIOA->AFR[1] = (GPIOA->AFR[1] & ~(15u << shift)) | (10u << shift);
        GPIOA->MODER = (GPIOA->MODER & ~(3u << (pin * 2))) | (2u << (pin * 2));
        GPIOA->OSPEEDR |= 3u << (pin * 2);
        GPIOA->PUPDR &= ~(3u << (pin * 2));
        GPIOA->OTYPER &= ~(1u << pin);
    }
    PWR->CR3 |= PWR_CR3_USB33DEN;
    RCC->AHB1ENR |= RCC_AHB1ENR_USB2OTGFSEN;
    (void)RCC->AHB1ENR;
    RCC->AHB1RSTR |= RCC_AHB1RSTR_USB2OTGFSRST;
    RCC->AHB1RSTR &= ~RCC_AHB1RSTR_USB2OTGFSRST;
}

static bool run_bench(void) {
    replay_start();
    uint32_t next = DWT->CYCCNT;
    uint32_t peak = 0, max_late = 0;
    const uint32_t cycles_per_us = SystemCoreClock / 1000000;
    for (unsigned i = 0; i < SAMPLE_COUNT; ++i) {
        while ((int32_t)(DWT->CYCCNT - next) < 0) tud_task_ext(0, false);
        uint32_t began = DWT->CYCCNT;
        uint32_t late = began - next;
        replay_sample(i);
        uint32_t duration = DWT->CYCCNT - began;
        if (duration > peak) peak = duration;
        if (late > max_late) max_late = late;
        next += PERIOD_US * cycles_per_us;
    }
    unsigned faults = check_faults();
    strcpy(report, "TILT-BENCH-1\n");
    for (unsigned i = 0; i < 3; ++i) {
        size_t used = strlen(report);
        format_checkpoint(report + used, sizeof(report) - used, &checkpoints[i]);
    }
    bool ok = faults == 255 && replay_correct() &&
              max_late + peak < PERIOD_US * cycles_per_us;
    size_t used = strlen(report);
    snprintf(report + used, sizeof(report) - used,
        "fault_mask,%u\nperiod_us,%u\npeak_step_us,%lu\nmax_start_late_us,%lu\n%s\nEND\n",
        faults, PERIOD_US, (unsigned long)(peak / cycles_per_us),
        (unsigned long)(max_late / cycles_per_us), ok ? "PASS" : "FAIL");
    return ok;
}

void start(void) {
    __disable_irq();
    for (uint32_t *p = &bss_start; p < &bss_end; ++p) *p = 0;
    SysTick->CTRL = 0;
    for (unsigned i = 0; i < 8; ++i) {
        NVIC->ICER[i] = UINT32_MAX;
        NVIC->ICPR[i] = UINT32_MAX;
    }
    SCB->VTOR = (uintptr_t)vectors;
    SCB->CPACR |= 0xfu << 20;
    __DSB(); __ISB();
    init_clock();
    init_pins();
    __set_BASEPRI(0);
    __enable_irq();
    // Let macOS observe the bootloader's USB disconnect before enumerating CDC.
    while (milliseconds < 300) {}
    tusb_init();
    bench_i2c_init();
    bool passed = run_bench();
    size_t sent = strlen(report);
    bool send_sensor = false;
    for (;;) {
        tud_task_ext(0, false);
        if (tud_cdc_available()) {
            int command = tud_cdc_read_char();
            if (command == 'r') { sent = 0; send_sensor = false; }
            if (command == 'i') {
                imu_identify(sensor_report, sizeof sensor_report);
                sent = 0; send_sensor = true;
            }
            if (command == 'b') {
                barometer_measure(sensor_report, sizeof sensor_report);
                sent = 0; send_sensor = true;
            }
        }
        const char *message = send_sensor ? sensor_report : report;
        if (sent < strlen(message) && tud_cdc_connected()) {
            sent += tud_cdc_write(message + sent, strlen(message) - sent);
            tud_cdc_write_flush();
        }
        // A steady blue LED means pass. A repeating blink indicates failure.
        GPIOC->BSRR = (passed || (milliseconds / 250) % 2) ? 1u << 29 : 1u << 13;
    }
}
