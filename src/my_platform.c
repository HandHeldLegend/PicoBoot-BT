// Example file - Public Domain
// Need help? https://tinyurl.com/bluepad32-help

#include <stddef.h>
#include <string.h>

#include <pico/cyw43_arch.h>
#include <pico/time.h>
#include <uni.h>
#include <pico/multicore.h>

#include "gamecube.h"
#include "types.h"

#include "sdkconfig.h"

// Sanity check
#ifndef CONFIG_BLUEPAD32_PLATFORM_CUSTOM
#error "Pico W must use BLUEPAD32_PLATFORM_CUSTOM"
#endif

uni_hid_device_t *_d[4];

// Declarations
static void trigger_event_on_gamepad(uni_hid_device_t* d);

//
// Platform Overrides
//
static void my_platform_init(int argc, const char** argv) {
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
    logi("my_platform: init()\n");

    uni_gamepad_mappings_t mappings = GAMEPAD_DEFAULT_MAPPINGS;
    // Invert A & B
    mappings.button_a = UNI_GAMEPAD_MAPPINGS_BUTTON_B;
    mappings.button_b = UNI_GAMEPAD_MAPPINGS_BUTTON_A;
    mappings.button_x = UNI_GAMEPAD_MAPPINGS_BUTTON_Y;
    mappings.button_y = UNI_GAMEPAD_MAPPINGS_BUTTON_X;
    uni_gamepad_set_mappings(&mappings);
}

static void my_platform_on_init_complete(void) {
    logi("my_platform: on_init_complete()\n");

    // Safe to call "unsafe" functions since they are called from BT thread

    // Start scanning
    uni_bt_enable_new_connections_unsafe(true);

    // Based on runtime condition, you can delete or list the stored BT keys.
    if (1)
        uni_bt_del_keys_unsafe();
    else
        uni_bt_list_keys_unsafe();

    // Turn off LED once init is done.
    //cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);

    //    uni_bt_service_set_enabled(true);

    uni_property_dump_all();
}

bool _is_connected[4];
bool _keep_alive[4];

static void my_platform_on_device_connected(uni_hid_device_t* d) {
    //logi("my_platform: device connected: %p\n", d);
    uint8_t idx = uni_hid_device_get_idx_for_instance(d);
    _d[idx] = d;
    _is_connected[idx] = true;
    _keep_alive[idx] = true;
    gamecube_controller_connect(idx, true);
}

static void my_platform_on_device_disconnected(uni_hid_device_t* d) {
    uint8_t idx = uni_hid_device_get_idx_for_instance(d);
    _is_connected[idx] = false;
    gamecube_controller_connect(idx, false);
    //logi("my_platform: device disconnected: %p\n", d);
}

static uni_error_t my_platform_on_device_ready(uni_hid_device_t* d) {
    //logi("my_platform: device ready: %p\n", d);

    // You can reject the connection by returning an error.
    return UNI_ERROR_SUCCESS;
}

volatile bool _should_rumble[4];
bool _is_rumbling[4];
interval_s rumbleinterval[4];
interval_s keepaliveinterval[4];


auto_init_mutex(rumble_mutex);

void my_platform_set_rumble(uint8_t idx, bool rumble)
{
    if(!_is_connected[idx]) return;

    uint32_t timestamp = gamecube_get_timestamp();

    if(rumble)
    {
        if(!_is_rumbling[idx])
        {
            _d[idx]->report_parser.play_dual_rumble(_d[idx], 0 /* delayed start ms */, 32 /* duration ms */, 128 /* weak magnitude */,
                                                40 /* strong magnitude */);
            interval_resettable_run(timestamp, 32000, true, &(rumbleinterval[idx]));
            _is_rumbling[idx] = true;
        }
        else
        {
            if(interval_resettable_run(timestamp, 32000, false, &(rumbleinterval[idx])))
            {
                _d[idx]->report_parser.play_dual_rumble(_d[idx], 0 /* delayed start ms */, 32 /* duration ms */, 128 /* weak magnitude */,
                                                40 /* strong magnitude */);
            }
        }
    }
    else _is_rumbling[idx] = false;


}

static void my_platform_on_controller_data(uni_hid_device_t* d, uni_controller_t* ctl) {
    static uint8_t leds = 0;
    static uint8_t enabled = true;
    static uni_controller_t prev = {0};
    uni_gamepad_t* gp;
    uint8_t idx = 0;

    //if (memcmp(&prev, ctl, sizeof(*ctl)) == 0) {
    //    return;
    //}
    //prev = *ctl;

    

    // Print device Id before dumping gamepad.
    //logi("(%p) id=%d ", d, idx);
    //uni_controller_dump(ctl);

    switch (ctl->klass) {
        case UNI_CONTROLLER_CLASS_GAMEPAD:
            idx = uni_hid_device_get_idx_for_instance(d);
            
            _keep_alive[idx] = true;

            gp = &ctl->gamepad;

            bool a = gp->buttons & BUTTON_A;
            bool b = gp->buttons & BUTTON_B;
            bool x = gp->buttons & BUTTON_X;
            bool y = gp->buttons & BUTTON_Y;

            switch(d->controller_type)
            {
                default:
                break;
                
                case k_eControllerType_XBox360Controller:
                case k_eControllerType_XBoxOneController:
                #define BUTTON_SUBTRACT_MASK ~(BUTTON_A | BUTTON_B | BUTTON_X | BUTTON_Y)
                gp->buttons &= 0b11110000;
                gp->buttons |= (a) ? BUTTON_B : 0;
                gp->buttons |= (b) ? BUTTON_A : 0;
                gp->buttons |= (x) ? BUTTON_Y : 0;
                gp->buttons |= (y) ? BUTTON_X : 0;
                break;
            }

            gamecube_comms_update(idx, gp);

            //// Debugging
            //// Axis ry: control rumble
            //if ((gp->buttons & BUTTON_A) && d->report_parser.play_dual_rumble != NULL) {
            //    d->report_parser.play_dual_rumble(d, 0 /* delayed start ms */, 250 /* duration ms */,
            //                                      128 /* weak magnitude */, 0 /* strong magnitude */);
            //}
            //if ((gp->buttons & BUTTON_B) && d->report_parser.play_dual_rumble != NULL) {
            //    d->report_parser.play_dual_rumble(d, 0 /* delayed start ms */, 250 /* duration ms */,
            //                                      0 /* weak magnitude */, 128 /* strong magnitude */);
            //}
            //// Buttons: Control LEDs On/Off
            //if ((gp->buttons & BUTTON_X) && d->report_parser.set_player_leds != NULL) {
            //    d->report_parser.set_player_leds(d, leds++ & 0x0f);
            //}
            //// Axis: control RGB color
            //if ((gp->buttons & BUTTON_Y) && d->report_parser.set_lightbar_color != NULL) {
            //    uint8_t r = (gp->axis_x * 256) / 512;
            //    uint8_t g = (gp->axis_y * 256) / 512;
            //    uint8_t b = (gp->axis_rx * 256) / 512;
            //    d->report_parser.set_lightbar_color(d, r, g, b);
            //}
            //// Toggle Bluetooth connections
            //if ((gp->buttons & BUTTON_SHOULDER_L) && enabled) {
            //    logi("*** Disabling Bluetooth connections\n");
            //    uni_bt_enable_new_connections_safe(false);
            //    enabled = false;
            //}
            //if ((gp->buttons & BUTTON_SHOULDER_R) && !enabled) {
            //    logi("*** Enabling Bluetooth connections\n");
            //    uni_bt_enable_new_connections_safe(true);
            //    enabled = true;
            //}
            //
            break;
        case UNI_CONTROLLER_CLASS_BALANCE_BOARD:
            // Do something
            uni_balance_board_dump(&ctl->balance_board);
            break;
        case UNI_CONTROLLER_CLASS_MOUSE:
            // Do something
            uni_mouse_dump(&ctl->mouse);
            break;
        case UNI_CONTROLLER_CLASS_KEYBOARD:
            // Do something
            uni_keyboard_dump(&ctl->keyboard);
            break;
        default:
            loge("Unsupported controller class: %d\n", ctl->klass);
            break;
    }
}

static const uni_property_t* my_platform_get_property(uni_property_idx_t idx) {
    ARG_UNUSED(idx);
    return NULL;
}

static void my_platform_on_oob_event(uni_platform_oob_event_t event, void* data) {
    return;
    
    switch (event) {
        case UNI_PLATFORM_OOB_GAMEPAD_SYSTEM_BUTTON:
            // Optional: do something when "system" button gets pressed.
            trigger_event_on_gamepad((uni_hid_device_t*)data);
            break;

        case UNI_PLATFORM_OOB_BLUETOOTH_ENABLED:
            // When the "bt scanning" is on / off. Could be triggered by different events
            // Useful to notify the user
            //logi("my_platform_on_oob_event: Bluetooth enabled: %d\n", (bool)(data));
            break;

        default:
            //logi("my_platform_on_oob_event: unsupported event: 0x%04x\n", event);
            break;
    }
}

//
// Helpers
//
static void trigger_event_on_gamepad(uni_hid_device_t* d) {
    if (d->report_parser.play_dual_rumble != NULL) {
        d->report_parser.play_dual_rumble(d, 0 /* delayed start ms */, 50 /* duration ms */, 128 /* weak magnitude */,
                                          40 /* strong magnitude */);
    }
}

//
// Entry Point
//
struct uni_platform* get_my_platform(void) {
    static struct uni_platform plat = {
        .name = "My Platform",
        .init = my_platform_init,
        .on_init_complete = my_platform_on_init_complete,
        .on_device_connected = my_platform_on_device_connected,
        .on_device_disconnected = my_platform_on_device_disconnected,
        .on_device_ready = my_platform_on_device_ready,
        .on_oob_event = my_platform_on_oob_event,
        .on_controller_data = my_platform_on_controller_data,
        .get_property = my_platform_get_property,
    };

    return &plat;
}
