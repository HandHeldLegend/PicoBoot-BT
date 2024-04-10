#include <stdint.h>

typedef struct
{
    union
    {
        struct
        {
            uint8_t button_a : 1;
            uint8_t button_b : 1;
            uint8_t button_x : 1;
            uint8_t button_y : 1;
            uint8_t button_start : 1;
            uint8_t blank_1 : 3;
        };
        uint8_t buttons_1;
    };

    union
    {
        struct
        {
            uint8_t dpad_left   : 1;
            uint8_t dpad_right  : 1;
            uint8_t dpad_down   : 1;
            uint8_t dpad_up     : 1;
            uint8_t button_z    : 1;
            uint8_t button_r    : 1;
            uint8_t button_l    : 1;
            uint8_t blank_2     : 1;
        };
        uint8_t buttons_2;
    };

    uint8_t stick_left_x;
    uint8_t stick_left_y;
    uint8_t stick_right_x;
    uint8_t stick_right_y;
    uint8_t analog_trigger_l;
    uint8_t analog_trigger_r;
} __attribute__ ((packed)) gamecube_input_s;

typedef struct 
{
  uint32_t this_time;
  uint32_t last_time;
} interval_s;