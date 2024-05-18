#ifndef INTERCORE_H
#define INTERCORE_H

#include <stdint.h>
#include <stdbool.h>
#include "uni_common.h"
#include <uni.h>
#include <uni_hid_device.h>

typedef enum
{
    IC_MSG_CONNECT = 1,
    IC_MSG_DISCONNECT,
    IC_MSG_RUMBLE,
    IC_MSG_INPUT,
    IC_MSG_MAX,
} intercore_id_t;

typedef struct
{
    union
    {
        struct
        {
            intercore_id_t  id      : 8;
            uint8_t         data    : 8;
        };
        uint16_t msg;
    };
    uni_gamepad_t   gp;
    bool            ready; // Message flagged to be read
} __attribute__ ((packed)) intercore_msg_s;

#include <stddef.h>
#include <string.h>
#include <pico/multicore.h>
#include <stdbool.h>
#include <stdint.h>

void core0_send_message_safe(intercore_msg_s *in);
bool core0_get_message_safe(intercore_msg_s *out);

void core1_send_message_safe(intercore_msg_s *in);
bool core1_get_message_safe(intercore_msg_s *out);
#endif