// FK743M3-VGT6: run from AXI SRAM at 0x24010000 via the USB bootloader.
#include <stdint.h>

#define REG32(address) (*(volatile uint32_t *)(address))
#define RCC_AHB4ENR    REG32(0x580244e0)
#define GPIOC_MODER    REG32(0x58020800)
#define GPIOC_BSRR     REG32(0x58020818)
#define SYSTICK_CTRL   REG32(0xe000e010)
#define SYSTICK_LOAD   REG32(0xe000e014)
#define SYSTICK_VAL    REG32(0xe000e018)
#define LED_PIN        13

// The bootloader clock determines the exact duration of each tick.
static inline __attribute__((always_inline)) void wait_ticks(unsigned ticks) {
    SYSTICK_VAL = 0;
    while (ticks > 0) {
        if (SYSTICK_CTRL & (1u << 16)) {
            --ticks;
        }
    }
}

void blink(void) {
    __asm volatile("cpsid i");
    RCC_AHB4ENR |= 1u << 2;
    (void)RCC_AHB4ENR;
    GPIOC_MODER = (GPIOC_MODER & ~(3u << (LED_PIN * 2))) | (1u << (LED_PIN * 2));
    SYSTICK_LOAD = 64000 - 1;
    SYSTICK_VAL = 0;
    SYSTICK_CTRL = 5;

    for (;;) {
        for (unsigned group = 0; group < 3; ++group) {
            unsigned on_time = 150;
            if (group == 1) {
                on_time = 450;  // The middle group has long flashes.
            }

            for (unsigned flash = 0; flash < 3; ++flash) {
                GPIOC_BSRR = 1u << (LED_PIN + 16);  // LED on
                wait_ticks(on_time);
                GPIOC_BSRR = 1u << LED_PIN;         // LED off
                wait_ticks(150);
            }
            wait_ticks(300);
        }
        wait_ticks(600);
    }
}
