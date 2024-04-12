// Example file - Public Domain
// Need help? http://bit.ly/bluepad32-help

#include <btstack_run_loop.h>
#include <pico/cyw43_arch.h>
#include <pico/stdlib.h>
#include <uni.h>
#include "pico/multicore.h"

#include "hardware/pio.h"

#include "picoboot.pio.h"

#include "sdkconfig.h"

#include "gamecube.h"

#include "ipl.h"
#include "hardware/clocks.h"
#include "hardware/dma.h"
#include "hardware/structs/bus_ctrl.h"

const uint PIN_DATA_BASE = 6;           // Base pin used for output, 4 consecutive pins are used 
const uint PIN_CS = 4;                 // U10 chip select
const uint PIN_CLK = 5;                // EXI bus clock line

// Sanity check
#ifndef CONFIG_BLUEPAD32_PLATFORM_CUSTOM
#error "Pico W must use BLUEPAD32_PLATFORM_CUSTOM"
#endif

volatile bool picoboot_done = false;
uint picoboot_done_irq;

uint transfer_start_sm;
uint transfer_start_offset;
uint clocked_output_sm;
uint clocked_output_offset;

// Defined in my_platform.c
struct uni_platform* get_my_platform(void);

static void picoboot_done_isr(void)
{
    irq_set_enabled(picoboot_done_irq, false);
    if(pio_interrupt_get(pio0, 1))
    {
        picoboot_done = true;
        pio_interrupt_clear(pio0, 1);
    }
}

void gamecube_task(void)
{
    multicore_lockout_victim_init();
    while(1)
    {
        gamecube_comms_task();
    }
    
}

void bluepad_core_task()
{
    // initialize CYW43 driver architecture (will enable BT if/because CYW43_ENABLE_BLUETOOTH == 1)
	if (cyw43_arch_init()) {
		loge("failed to initialise cyw43_arch\n");
		return;
	}

	// Must be called before uni_main()
	uni_platform_set_custom(get_my_platform());

	// Initialize BP32
	uni_init(0, NULL);

	// Does not return.
	btstack_run_loop_execute();
}

int main() {
    stdio_init_all();

    // Set 250MHz clock to get more cycles in between CLK pulses.
    // This is the lowest value I was able to make the code work.
    // Should be still considered safe for most Pico boards.
    set_sys_clock_khz(SYS_CLK_SPEED_KHZ, true);

    // Prioritize DMA engine as it does the most work
    bus_ctrl_hw->priority = BUSCTRL_BUS_PRIORITY_DMA_W_BITS | BUSCTRL_BUS_PRIORITY_DMA_R_BITS;

    gpio_set_slew_rate(PIN_DATA_BASE, GPIO_SLEW_RATE_FAST);
    gpio_set_slew_rate(PIN_DATA_BASE + 1, GPIO_SLEW_RATE_FAST);
    gpio_set_slew_rate(PIN_DATA_BASE + 2, GPIO_SLEW_RATE_FAST);
    gpio_set_slew_rate(PIN_DATA_BASE + 3, GPIO_SLEW_RATE_FAST);

    gpio_set_drive_strength(PIN_DATA_BASE, GPIO_DRIVE_STRENGTH_8MA);
    gpio_set_drive_strength(PIN_DATA_BASE + 1, GPIO_DRIVE_STRENGTH_8MA);
    gpio_set_drive_strength(PIN_DATA_BASE + 2, GPIO_DRIVE_STRENGTH_8MA);
    gpio_set_drive_strength(PIN_DATA_BASE + 3, GPIO_DRIVE_STRENGTH_8MA);

    PIO pio = pio0;

    

    //
    // State Machine: Transfer Start
    //
    // Counts all consecutive transfers and sets IRQ 
    // when first 1 kilobyte transfer starts.
    //

    transfer_start_sm = pio_claim_unused_sm(pio, true);
    transfer_start_offset = pio_add_program(pio, &on_transfer_program);

    on_transfer_program_init(pio, transfer_start_sm, transfer_start_offset, PIN_CLK, PIN_CS, PIN_DATA_BASE);

    pio_sm_put(pio, transfer_start_sm, (uint32_t) 224); // CS pulses
    pio_sm_exec(pio, transfer_start_sm, pio_encode_pull(true, true));
    pio_sm_exec(pio, transfer_start_sm, pio_encode_mov(pio_x, pio_osr));
    pio_sm_exec(pio, transfer_start_sm, pio_encode_out(pio_null, 32));

    //
    // State Machine: Clocked Output
    // 
    // It waits for IRQ signal from first SM and samples clock signal
    // to output IPL data bits.
    //

    clocked_output_sm = pio_claim_unused_sm(pio, true);
    clocked_output_offset = pio_add_program(pio, &clocked_output_program);

    clocked_output_program_init(pio, clocked_output_sm, clocked_output_offset, PIN_DATA_BASE, PIN_CLK, PIN_CS);

    pio_sm_put(pio, clocked_output_sm, 8191); // 8192 bits, 1024 bytes, minus 1 because counting starts from 0 
    pio_sm_exec(pio, clocked_output_sm, pio_encode_pull(true, true));
    pio_sm_exec(pio, clocked_output_sm, pio_encode_mov(pio_y, pio_osr));
    pio_sm_exec(pio, clocked_output_sm, pio_encode_out(pio_null, 32));

    // Set up DMA for reading IPL to PIO FIFO
    int chan = dma_claim_unused_channel(true);

    dma_channel_config c = dma_channel_get_default_config(chan);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
    channel_config_set_read_increment(&c, true);
    channel_config_set_write_increment(&c, false);
    channel_config_set_dreq(&c, pio_get_dreq(pio, clocked_output_sm, true));

    dma_channel_configure(
        chan,
        &c,
        &pio->txf[clocked_output_sm],
        ipl,
        count_of(ipl),
        true // start immediately
    );

    // Start PIO state machines
    pio_sm_set_enabled(pio, transfer_start_sm, true);
    pio_sm_set_enabled(pio, clocked_output_sm, true);

    sleep_ms(800);

    // Clean up before we move to bluetooth nonsense
    pio_sm_set_enabled(pio0, transfer_start_sm, false);
    pio_sm_set_enabled(pio0, clocked_output_sm, false);

    pio_sm_unclaim(pio0, transfer_start_sm);
    pio_sm_unclaim(pio0, clocked_output_sm);

    pio_clear_instruction_memory(pio0);

    multicore_launch_core1(bluepad_core_task);

    for(;;)
    {  
        gamecube_task();
    }

    return 0;
}
