#include "intercore.h"

auto_init_mutex(intercore_core0_mutex);
auto_init_mutex(intercore_core1_mutex);

#define INTERCORE_QUEUE_LEN 32

volatile intercore_msg_s _core0_queue[INTERCORE_QUEUE_LEN];
volatile uint8_t _core0_write_idx = 0;

volatile intercore_msg_s _core1_queue[INTERCORE_QUEUE_LEN];
volatile uint8_t _core1_write_idx = 0;

void core0_send_message_safe(intercore_msg_s *in)
{
    mutex_enter_blocking(&intercore_core0_mutex);

    // Ensure it's not a duplicate message
    // to the last written
    if(_core0_queue[_core0_write_idx].msg != in->msg)
    {
        // msg contains the ID and data
        _core0_queue[_core0_write_idx].msg = in->msg;

        // Only waste cycles on gamepad data if we need to
        if(_core0_queue[_core0_write_idx].id == IC_MSG_INPUT)
        {
            _core0_queue[_core0_write_idx].gp.axis_x        = in->gp.axis_x;
            _core0_queue[_core0_write_idx].gp.axis_y        = in->gp.axis_y;
            _core0_queue[_core0_write_idx].gp.axis_rx       = in->gp.axis_rx;
            _core0_queue[_core0_write_idx].gp.axis_ry       = in->gp.axis_ry;
            _core0_queue[_core0_write_idx].gp.dpad          = in->gp.dpad;
            _core0_queue[_core0_write_idx].gp.brake         = in->gp.brake;
            _core0_queue[_core0_write_idx].gp.throttle      = in->gp.throttle;
            _core0_queue[_core0_write_idx].gp.buttons       = in->gp.buttons;
            _core0_queue[_core0_write_idx].gp.misc_buttons  = in->gp.misc_buttons;
        }

        _core0_queue[_core0_write_idx].ready = true;
        _core0_write_idx = (_core0_write_idx+1) % INTERCORE_QUEUE_LEN;
    }

    mutex_exit(&intercore_core0_mutex);
}

bool core0_get_message_safe(intercore_msg_s *out)
{
    mutex_enter_blocking(&intercore_core0_mutex);

    for(uint i = 0; i < INTERCORE_QUEUE_LEN; i++)
    {
        if(_core0_queue[i].ready)
        {
            out->msg = _core0_queue[i].msg;

            // Only waste cycles on gamepad data if we need to
            if(_core0_queue[i].id == IC_MSG_INPUT)
            {
                out->gp.axis_x       = _core0_queue[i].gp.axis_x;     
                out->gp.axis_y       = _core0_queue[i].gp.axis_y;     
                out->gp.axis_rx      = _core0_queue[i].gp.axis_rx;     
                out->gp.axis_ry      = _core0_queue[i].gp.axis_ry;     
                out->gp.dpad         = _core0_queue[i].gp.dpad;     
                out->gp.brake        = _core0_queue[i].gp.brake;     
                out->gp.throttle     = _core0_queue[i].gp.throttle;         
                out->gp.buttons      = _core0_queue[i].gp.buttons;     
                out->gp.misc_buttons = _core0_queue[i].gp.misc_buttons;             
            }

            _core0_queue[i].ready = false;
            mutex_exit(&intercore_core0_mutex);
            return true;
        }
    }

    mutex_exit(&intercore_core0_mutex);
    return false;
}

void core1_send_message_safe(intercore_msg_s *in)
{
    mutex_enter_blocking(&intercore_core1_mutex);

    // Ensure it's not a duplicate message
    // to the last written
    if(_core1_queue[_core1_write_idx].msg != in->msg)
    {
        for(uint i = 0; i < INTERCORE_QUEUE_LEN; i++)
        {
            if(!_core1_queue[i].ready)
            {
                _core1_queue[i].msg = in->msg;
                _core1_queue[i].ready = true;
                mutex_exit(&intercore_core1_mutex);
                return;
            }
        }
    }
    mutex_exit(&intercore_core1_mutex);
}

bool core1_get_message_safe(intercore_msg_s *out)
{
    mutex_enter_blocking(&intercore_core1_mutex);

    for(uint i = 0; i < INTERCORE_QUEUE_LEN; i++)
    {
        if(_core1_queue[i].ready)
        {
            out->msg = _core1_queue[i].msg;
            _core1_queue[i].ready = false;
            mutex_exit(&intercore_core1_mutex);
            return true;
        }
    }

    mutex_exit(&intercore_core1_mutex);
    return false;
}
