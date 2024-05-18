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
#include "intercore.h"

// Sanity check
#ifndef CONFIG_BLUEPAD32_PLATFORM_CUSTOM
#error "Pico W must use BLUEPAD32_PLATFORM_CUSTOM"
#endif

typedef struct
{
    // Set this to keep a controller connection alive
    bool keep_alive;
    interval_s timeout_interval;
    bool connected;
    bool led_set;
    uni_hid_device_t *device_ptr;
} my_playform_player_s;

my_playform_player_s _players[4] = {0};

// MY_PLATFORM.C FUNCTIONS RUN ON CORE 1
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

static void my_platform_on_device_connected(uni_hid_device_t* d) {
    //logi("my_platform: device connected: %p\n", d);
    uint8_t idx = uni_hid_device_get_idx_for_instance(d);
    static intercore_msg_s core1playermsg = {.id = IC_MSG_CONNECT};
    
    switch(idx)
    {
        case 0 ... 3:
            _players[idx].connected = true;
            _players[idx].device_ptr = d;
            _players[idx].led_set = false;
            core1playermsg.data = idx;
            core0_send_message_safe(&core1playermsg);
        break;

        default:
        // Do nothing for other cases
        break;
    }
}

static void my_platform_on_device_disconnected(uni_hid_device_t* d) {
    //logi("my_platform: device connected: %p\n", d);
    uint8_t idx = uni_hid_device_get_idx_for_instance(d);
    static intercore_msg_s core1playermsg = {.id = IC_MSG_DISCONNECT};
    
    switch(idx)
    {
        case 0 ... 3:
            core1playermsg.data = idx;
            core0_send_message_safe(&core1playermsg);
            _players[idx].connected = false;
        break;

        default:
        // Do nothing for other cases
        break;
    }
}

static uni_error_t my_platform_on_device_ready(uni_hid_device_t* d) {
    //logi("my_platform: device ready: %p\n", d);

    // You can reject the connection by returning an error.
    return UNI_ERROR_SUCCESS;
}

void _my_platform_process_rumble(uint8_t idx, bool rumble)
{
    bool state = rumble;
    uni_hid_device_t *d = _players[idx].device_ptr;
    if(!_players[idx].connected | d->report_parser.play_dual_rumble == NULL) return;

    if(state)
    d->report_parser.play_dual_rumble(d, 0, 32, 128, 40);
    else
    d->report_parser.play_dual_rumble(d, 0, 0, 0, 0);
}

static void my_platform_on_controller_data(uni_hid_device_t* d, uni_controller_t* ctl) {
    uni_gamepad_t* gp;
    
    uint8_t idx = uni_hid_device_get_idx_for_instance(d);

    if(!_players[idx].led_set && (d->report_parser.set_player_leds!=NULL))
    {
        // Set player LEDs
        if(d->report_parser.set_player_leds != NULL)
        {
            d->report_parser.set_player_leds(d, ((1<<idx)&0xF));
            _players[idx].led_set = true;
        }
        
    }

    // Send input message if it's a gamepad of valid type
    switch (ctl->klass) {
        case UNI_CONTROLLER_CLASS_GAMEPAD:

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

            // Send intercore message for input update
            static intercore_msg_s inputmsg = {.id = IC_MSG_INPUT};
            static uint8_t inputcounter = 0; // Increment inputs
            
            inputmsg.gp.axis_x          = gp->axis_x;
            inputmsg.gp.axis_y          = gp->axis_y;
            inputmsg.gp.axis_rx         = gp->axis_rx;
            inputmsg.gp.axis_ry         = gp->axis_ry;
            inputmsg.gp.dpad            = gp->dpad;
            inputmsg.gp.brake           = gp->brake;
            inputmsg.gp.throttle        = gp->throttle;
            inputmsg.gp.buttons         = gp->buttons;
            inputmsg.gp.misc_buttons    = gp->misc_buttons;

            inputmsg.data = (idx&3) | (inputcounter<<2);
            core0_send_message_safe(&inputmsg);
            inputcounter = (inputcounter+1) % 0b111111;

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

static void my_platform_bt_loop_hook()
{
    static intercore_msg_s core1msg = {0};
    static uint8_t _rumble_status = 0;

    if(core1_get_message_safe(&core1msg))
    {
        switch(core1msg.id)
        {
            default:
            break;

            case IC_MSG_RUMBLE:
                _rumble_status = core1msg.data & 0b1111;
            break;
        }
    }

    _my_platform_process_rumble(0, (_rumble_status & 0b1000));
    _my_platform_process_rumble(1, (_rumble_status & 0b100));
    _my_platform_process_rumble(2, (_rumble_status & 0b10));
    _my_platform_process_rumble(3, (_rumble_status & 0b1));

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
        .safe_platform_hook = my_platform_bt_loop_hook
    };

    return &plat;
}
