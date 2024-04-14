// -------------------------------------------------- //
// This file is autogenerated by pioasm; do not edit! //
// -------------------------------------------------- //

#pragma once

#if !PICO_NO_HARDWARE
#include "hardware/pio.h"
#endif

// ------ //
// joybus //
// ------ //

#define joybus_wrap_target 0
#define joybus_wrap 24

#define joybus_offset_joybusin 0u
#define joybus_offset_joybusout 10u

static const uint16_t joybus_program_instructions[] = {
            //     .wrap_target
    0xe080, //  0: set    pindirs, 0                 
    0xe027, //  1: set    x, 7                       
    0x20a0, //  2: wait   1 pin, 0                   
    0x2720, //  3: wait   0 pin, 0               [7] 
    0xa042, //  4: nop                               
    0x4001, //  5: in     pins, 1                    
    0x0042, //  6: jmp    x--, 2                     
    0xc001, //  7: irq    nowait 1                   
    0xb842, //  8: nop                    side 1     
    0x0001, //  9: jmp    1                          
    0xa742, // 10: nop                           [7] 
    0xf880, // 11: set    pindirs, 0      side 1     
    0x7a21, // 12: out    x, 1            side 1 [2] 
    0xf081, // 13: set    pindirs, 1      side 0     
    0x1232, // 14: jmp    !x, 18          side 0 [2] 
    0xf880, // 15: set    pindirs, 0      side 1     
    0x1eeb, // 16: jmp    !osre, 11       side 1 [6] 
    0x0015, // 17: jmp    21                         
    0xf081, // 18: set    pindirs, 1      side 0     
    0x16eb, // 19: jmp    !osre, 11       side 0 [6] 
    0xf880, // 20: set    pindirs, 0      side 1     
    0xfa80, // 21: set    pindirs, 0      side 1 [2] 
    0xf381, // 22: set    pindirs, 1      side 0 [3] 
    0xfb80, // 23: set    pindirs, 0      side 1 [3] 
    0x0000, // 24: jmp    0                          
            //     .wrap
};

#if !PICO_NO_HARDWARE
static const struct pio_program joybus_program = {
    .instructions = joybus_program_instructions,
    .length = 25,
    .origin = -1,
};

static inline pio_sm_config joybus_program_get_default_config(uint offset) {
    pio_sm_config c = pio_get_default_sm_config();
    sm_config_set_wrap(&c, offset + joybus_wrap_target, offset + joybus_wrap);
    sm_config_set_sideset(&c, 2, true, false);
    return c;
}

static inline void joybus_set_in(bool in, PIO pio, uint sm, uint offset, pio_sm_config *c, uint pin)
{
    // Disable SM
    pio_sm_clear_fifos(pio, sm);
    pio_sm_set_enabled(pio, sm, false);
    if (in)
    {
        pio_sm_set_consecutive_pindirs(pio, sm, pin, 1, false);
        pio_sm_init(pio, sm, offset + joybus_offset_joybusin, c);
    }
    else
    {
        pio_sm_set_consecutive_pindirs(pio, sm, pin, 1, true);
        pio_sm_init(pio, sm, offset + joybus_offset_joybusout, c);
    }
    pio_sm_set_enabled(pio, sm, true);
}
static inline void joybus_program_init(uint32_t clk_sys, PIO pio, uint sm, uint offset, uint pin, pio_sm_config *c) {
    *c = joybus_program_get_default_config(offset);
    gpio_init(pin);
    // Set this pin's GPIO function (connect PIO to the pad)
    pio_gpio_init(pio, pin);
    pio_sm_set_consecutive_pindirs(pio, sm, pin, 1, false);
    sm_config_set_in_pins(c, pin);
    sm_config_set_out_pins(c, pin, 1);
    sm_config_set_jmp_pin(c, pin);
    // Must run 12800000hz
    float div = 62.5f; //clock_get_hz(clk_sys) / (4000000);
    sm_config_set_clkdiv(c, div);
    // Set sideset pin
    sm_config_set_sideset_pins(c, pin);
    sm_config_set_in_shift(c, false, true, 8);
    sm_config_set_out_shift(c, false, true, 8);
    // Load our configuration, and jump to the start of the program
    pio_sm_init(pio, sm, offset, c);
    // Set the state machine running
    pio_sm_set_enabled(pio, sm, true);
}

#endif
