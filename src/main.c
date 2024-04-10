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

// Sanity check
#ifndef CONFIG_BLUEPAD32_PLATFORM_CUSTOM
#error "Pico W must use BLUEPAD32_PLATFORM_CUSTOM"
#endif

// Defined in my_platform.c
struct uni_platform* get_my_platform(void);

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

	// Turn-on LED. Turn it off once init is done.
	cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);

	// Must be called before uni_main()
	uni_platform_set_custom(get_my_platform());

	// Initialize BP32
	uni_init(0, NULL);

	// Does not return.
	btstack_run_loop_execute();
}

int main() {
    stdio_init_all();

    set_sys_clock_khz(SYS_CLK_SPEED_KHZ, true);
    
    multicore_launch_core1(bluepad_core_task);

    for(;;)
    {
        gamecube_task();
    }

    return 0;
}
