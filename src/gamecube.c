#include <stdint.h>
#include <stdbool.h>
#include "gamecube.h"
#include <hardware/pio.h>
#include <pico/multicore.h>
#include "hardware/dma.h"
#include <pico/cyw43_arch.h>

#include <uni.h>
#include <uni_hid_device.h>

#include "sdkconfig.h"

#include "joybus.pio.h"
#include "hardware/structs/pio.h"
#include "intercore.h"

// GAMECUBE.C FUNCTIONS RUN ON CORE 0

#define PIO_SM_0_IRQ 0b0001
#define PIO_SM_1_IRQ 0b0010
#define PIO_SM_2_IRQ 0b0100
#define PIO_SM_3_IRQ 0b1000

#define GAMEPAD_PIO pio0
#define GAMEPAD_SM 0

#define CLAMP_0_255(value) ((value) < 0 ? 0 : ((value) > 255 ? 255 : (value)))

#define JOYBUS_CHANNELS 4

const uint joybus_pins[4] = PINS_JOYBUS;

mutex_t timestamp_mutex;
volatile uint32_t time;
uint32_t gamecube_get_timestamp()
{
  uint32_t ret = time;
  mutex_enter_timeout_us(&timestamp_mutex, 100);
  time = time_us_32();
  ret = time;
  mutex_exit(&timestamp_mutex);

  return ret;
}

bool interval_run(uint32_t timestamp, uint32_t interval, interval_s *state)
{
  state->this_time = timestamp;

  // Clear variable
  uint32_t diff = 0;

  // Handle edge case where time has
  // looped around and is now less
  if (state->this_time < state->last_time)
  {
    diff = (0xFFFFFFFF - state->last_time) + state->this_time;
  }
  else if (state->this_time > state->last_time)
  {
    diff = state->this_time - state->last_time;
  }
  else
    return false;

  // We want a target rate according to our variable
  if (diff >= interval)
  {
    // Set the last time
    state->last_time = state->this_time;
    return true;
  }
  return false;
}

// This is a function that will return
// offers the ability to restart the timer
bool interval_resettable_run(uint32_t timestamp, uint32_t interval, bool reset, interval_s *state)
{
  state->this_time = timestamp;

  if (reset)
  {
    state->last_time = state->this_time;
    return false;
  }

  // Clear variable
  uint64_t diff = 0;

  // Handle edge case where time has
  // looped around and is now less
  if (state->this_time < state->last_time)
  {
    diff = (0xFFFFFFFF - state->last_time) + state->this_time;
  }
  else if (state->this_time > state->last_time)
  {
    diff = state->this_time - state->last_time;
  }
  else
    return false;

  // We want a target rate according to our variable
  if (diff >= interval)
  {
    // Set the last time
    state->last_time = state->this_time;
    return true;
  }
  return false;
}

uint _gamecube_irq_rx;
uint _gamecube_irq_tx;
uint _gamecube_offset;
pio_sm_config _gamecube_c[4];

volatile bool _gc_got_data[4];
volatile bool _gc_running = false;

volatile gamecube_input_s _in_buffer[4] = {0};
volatile gamecube_input_s _out_buffer[4] = {0};
volatile uint8_t _dmaOut[4][8];

typedef struct
{
  bool connected;
  int byteCounter; // How many bytes left to read before we must respond
  uint8_t workingCmd;  // The current working command we received
  uint8_t workingMode;
  bool lock; // Lock so it doesn't get interrupted
  bool rumble;         // if rumble is enabled or not
  int dma;
} joybus_state_s;

volatile joybus_state_s _joybus_state[4];

volatile uni_gamepad_t _incoming_gamepad[4];
volatile bool _in_buffer_update[4];

dma_channel_config _joybus_dma_config[4];

uint32_t _gamepad_owner_0;
uint32_t _gamepad_owner_1;

mutex_t _gamepad_mutex[4];

#define ALIGNED_JOYBUS_8(val) ((val) << 24)

uint8_t _cannedProbe[3] = {0x09, 0x00, 0x03};
uint8_t _cannedProbeTX[4][3];
uint8_t _cannedOrigin[10] = {0x00, 128, 128, 128, 128, 128, 0x00, 0x00, 0x02, 0x02};
uint8_t _cannedOriginTX[4][10];

void _gamecube_send_probe(uint sm)
{
  pio_sm_clear_fifos(GAMEPAD_PIO, sm);
  dma_channel_set_trans_count(_joybus_state[sm].dma, 3, false);
  dma_channel_set_read_addr(_joybus_state[sm].dma, &(_cannedProbeTX[sm]), true);
}

void _gamecube_send_origin(uint sm)
{
  pio_sm_clear_fifos(GAMEPAD_PIO, sm);
  dma_channel_set_trans_count(_joybus_state[sm].dma, 10, false);
  dma_channel_set_read_addr(_joybus_state[sm].dma, &(_cannedOriginTX[sm]), true);
}

void _gamecube_send_poll(uint sm)
{
  pio_sm_clear_fifos(GAMEPAD_PIO, sm);
  dma_channel_set_trans_count(_joybus_state[sm].dma, 8, false);
  dma_channel_set_read_addr(_joybus_state[sm].dma, &(_out_buffer[sm]), true);
}

void _gamecube_reset_state(uint sm)
{
  joybus_set_in(true, GAMEPAD_PIO, sm, _gamecube_offset, &(_gamecube_c[sm]), joybus_pins[sm]);
  _joybus_state[sm].byteCounter = 3;
  _joybus_state[sm].workingCmd = 0x00;
}

#define BYTECOUNT_DEFAULT 2
#define BYTECOUNT_NULL -1
#define BYTECOUNT_SWISS 10

// Returns true if we are at the end of a command area (start response)
bool __time_critical_func(_gamecube_command_handler)(uint sm, uint8_t mask)
{
  bool ret = false;
  uint8_t dat = 0;

  if(_joybus_state[sm].byteCounter==BYTECOUNT_NULL)
  {
    _joybus_state[sm].workingCmd = pio_sm_get(GAMEPAD_PIO, sm);
    switch(_joybus_state[sm].workingCmd)
    {
      default:
        _joybus_state[sm].byteCounter = BYTECOUNT_DEFAULT;
        break;
      case GCUBE_CMD_SWISS:
        _joybus_state[sm].byteCounter = BYTECOUNT_SWISS;
        break;
    }
  }
  else
  {
    dat = pio_sm_get(GAMEPAD_PIO, sm);
  }
 
  switch (_joybus_state[sm].workingCmd)
  {
  default:
    break;

  case GCUBE_CMD_SWISS:
  {
    if(!_joybus_state[sm].byteCounter)
    {
      _joybus_state[sm].byteCounter = BYTECOUNT_NULL;
      _gamecube_reset_state(sm);
      ret = true;
    }
  }
  break;

  case GCUBE_CMD_ORIGINEXT:
    if(!_joybus_state[sm].byteCounter)
    {
      _gc_got_data[sm] = true;
      _joybus_state[sm].byteCounter = BYTECOUNT_NULL;
      
      joybus_set_in(false, GAMEPAD_PIO, sm, _gamecube_offset, &(_gamecube_c[sm]), joybus_pins[sm]);
      pio_set_sm_mask_enabled(GAMEPAD_PIO, (1<<sm), false);
      _gamecube_send_origin(sm);
      pio_set_sm_mask_enabled(GAMEPAD_PIO, (1<<sm), true);

      ret = true;
    }
  break;


  case GCUBE_CMD_POLL:

    if(_joybus_state[sm].byteCounter==1)
    {
      _joybus_state[sm].workingMode = dat;
    }
    if (!_joybus_state[sm].byteCounter)
    {
      _joybus_state[sm].rumble = (dat & 0x1) ? true : false;

      _gc_got_data[sm] = true;
      _joybus_state[sm].byteCounter = BYTECOUNT_NULL;
      
      joybus_set_in(false, GAMEPAD_PIO, sm, _gamecube_offset, &(_gamecube_c[sm]), joybus_pins[sm]);
      pio_set_sm_mask_enabled(GAMEPAD_PIO, (1<<sm), false);
      _gamecube_send_poll(sm);
      pio_set_sm_mask_enabled(GAMEPAD_PIO, (1<<sm), true);
      
      ret = true;
    }

  break;

  
  case GCUBE_CMD_PROBE:
    _gc_got_data[sm] = true;
    _joybus_state[sm].byteCounter = BYTECOUNT_NULL;

    joybus_set_in(false, GAMEPAD_PIO, sm, _gamecube_offset, &(_gamecube_c[sm]), joybus_pins[sm]);
    pio_set_sm_mask_enabled(GAMEPAD_PIO, (1<<sm), false);
    _gamecube_send_probe(sm);
    pio_set_sm_mask_enabled(GAMEPAD_PIO, (1<<sm), true);
    
    ret = true;
    break;

  case GCUBE_CMD_ORIGIN:
    _gc_got_data[sm] = true;
    _joybus_state[sm].byteCounter = BYTECOUNT_NULL;
    
    joybus_set_in(false, GAMEPAD_PIO, sm, _gamecube_offset, &(_gamecube_c[sm]), joybus_pins[sm]);
    pio_set_sm_mask_enabled(GAMEPAD_PIO, (1<<sm), false);
    _gamecube_send_origin(sm);
    pio_set_sm_mask_enabled(GAMEPAD_PIO, (1<<sm), true);
    
    ret = true;
    break;
  }

  if(!ret)
  _joybus_state[sm].byteCounter -= 1;

  return ret;
}

static void _gamecube_isr_rxgot(void)
{
  irq_set_enabled(_gamecube_irq_rx, false);
  if (pio_interrupt_get(GAMEPAD_PIO, 1))
  {
    pio_interrupt_clear(GAMEPAD_PIO, 1);

    for(uint i = 0; i < JOYBUS_CHANNELS; i++)
    {
        if(!pio_sm_is_rx_fifo_empty(GAMEPAD_PIO, i) && _joybus_state[i].connected)
        {
          _gamecube_command_handler(i, 0);
        }
    }
    
  }
  irq_set_enabled(_gamecube_irq_rx, true);
}

static void _gamecube_isr_txdone(void)
{
  irq_set_enabled(_gamecube_irq_tx, false);

  if (pio_interrupt_get(GAMEPAD_PIO, 0))
  {
    pio_interrupt_clear(GAMEPAD_PIO, 0);
    //sleep_us(150);
    
    
    for(uint i = 0; i < 4; i++)
    {
      _joybus_state[i].byteCounter = 3;
      joybus_set_in(true, GAMEPAD_PIO, i, _gamecube_offset, &(_gamecube_c[i]), joybus_pins[i]);
    }
  }

  irq_set_enabled(_gamecube_irq_tx, true);
  
}

#define ANALOG_F_CONST (float) (0.21484375f)

uint8_t _gamecube_analog_helper(int32_t input)
{
  float   scaled = input*ANALOG_F_CONST;
  int     out = scaled+127;
  uint8_t val = CLAMP_0_255(out);
  return val;
}

uint8_t _gamecube_analog_trigger_helper(int32_t input)
{
  float   scaled = input*0.499f;
  int     out = scaled;
  uint8_t val = CLAMP_0_255(out);
  return val;
}

interval_s interval[4];

void _message_handle_connect(uint player, bool connected)
{
  if(connected && !_joybus_state[player].connected)
  {
    pio_sm_clear_fifos(GAMEPAD_PIO, player);
    _joybus_state[player].connected = true;
    _joybus_state[player].byteCounter = BYTECOUNT_DEFAULT;
    _joybus_state[player].workingCmd = 0x00;
    _joybus_state[player].rumble = false;
    joybus_program_init(SYS_CLK_SPEED_HZ, GAMEPAD_PIO, player, _gamecube_offset, joybus_pins[player], &(_gamecube_c[player]));
  }
  else if(!connected)
  {
    _joybus_state[player].connected = false;
  }
}

void _message_handle_input(uint player, intercore_msg_s *msg)
{
  _out_buffer[player].blank_2 = 1;
  _out_buffer[player].a        = (msg->gp.buttons & BUTTON_A) ? true : false;
  _out_buffer[player].b        = (msg->gp.buttons & BUTTON_B) ? true : false;
  _out_buffer[player].x        = (msg->gp.buttons & BUTTON_X) ? true : false;
  _out_buffer[player].y        = (msg->gp.buttons & BUTTON_Y) ? true : false;

  _out_buffer[player].start    = (msg->gp.misc_buttons & MISC_BUTTON_START) ? true : false;
  _out_buffer[player].start    |= (msg->gp.misc_buttons & MISC_BUTTON_SYSTEM) ? true : false;

  _out_buffer[player].z = (msg->gp.buttons & BUTTON_SHOULDER_R) ? true : false;

  uint8_t lt8 = 0;
  uint8_t rt8 = 0;
  uint8_t outl = 0;
  uint8_t outr = 0;

  uint8_t lx8 = 0;
  uint8_t ly8 = 0;
  uint8_t rx8 = 0;
  uint8_t ry8 = 0;

  lx8   = _gamecube_analog_helper(msg->gp.axis_x);
  ly8   = _gamecube_analog_helper(-msg->gp.axis_y);
  rx8   = _gamecube_analog_helper(msg->gp.axis_rx);
  ry8   = _gamecube_analog_helper(-msg->gp.axis_ry);

  _out_buffer[player].dpad_down     = (msg->gp.dpad & DPAD_DOWN) ? true : false;
  _out_buffer[player].dpad_left     = (msg->gp.dpad & DPAD_LEFT) ? true : false;
  _out_buffer[player].dpad_right    = (msg->gp.dpad & DPAD_RIGHT) ? true : false;
  _out_buffer[player].dpad_up       = (msg->gp.dpad & DPAD_UP) ? true : false;

  uint8_t al = _gamecube_analog_trigger_helper(msg->gp.brake);
  uint8_t ar = _gamecube_analog_trigger_helper(msg->gp.throttle);

  bool alb = (msg->gp.buttons & BUTTON_TRIGGER_L) ? true : false;
  bool arb = (msg->gp.buttons & BUTTON_TRIGGER_R) ? true : false;

  if(al>245)
  {
    if(alb) al = 255;
    alb = true;
  }
  else if (al>0)
  {
    alb = false;
  }
  else
  {
    al = (alb) ? 255 : 0;
  }

  if(ar>245)
  {
    if(arb) ar = 255;
    arb = true;
  }
  else if (ar>0)
  {
    arb = false;
  }
  else
  {
    ar = (arb) ? 255 : 0;
  }

  lt8 = al;
  rt8 = ar;
  _out_buffer[player].l = alb;
  _out_buffer[player].r = arb;
  _in_buffer_update[player] = false;

  switch(_joybus_state[player].workingMode)
  {
    // Default is mode 3
    default:
      _out_buffer[player].stick_left_x  = lx8;
      _out_buffer[player].stick_left_y  = ly8;
      _out_buffer[player].stick_right_x = rx8;
      _out_buffer[player].stick_right_y = ry8;
      _out_buffer[player].analog_trigger_l  = lt8;
      _out_buffer[player].analog_trigger_r  = rt8;
    break;

    case 0:
      _out_buffer[player].mode0.stick_left_x  = lx8;
      _out_buffer[player].mode0.stick_left_y  = ly8;
      _out_buffer[player].mode0.stick_right_x = rx8;
      _out_buffer[player].mode0.stick_right_y = ry8;
      _out_buffer[player].mode0.analog_trigger_l  = lt8>>4;
      _out_buffer[player].mode0.analog_trigger_r  = rt8>>4;
      _out_buffer[player].mode0.analog_a = 0; // 4bits
      _out_buffer[player].mode0.analog_b = 0; // 4bits
    break;

    case 1:
      _out_buffer[player].mode1.stick_left_x  = lx8;
      _out_buffer[player].mode1.stick_left_y  = ly8;
      _out_buffer[player].mode1.stick_right_x = rx8>>4;
      _out_buffer[player].mode1.stick_right_y = ry8>>4;
      _out_buffer[player].mode1.analog_trigger_l  = lt8;
      _out_buffer[player].mode1.analog_trigger_r  = rt8;
      _out_buffer[player].mode1.analog_a = 0; // 4bits
      _out_buffer[player].mode1.analog_b = 0; // 4bits
    break;

    case 2:
      _out_buffer[player].mode2.stick_left_x  = lx8;
      _out_buffer[player].mode2.stick_left_y  = ly8;
      _out_buffer[player].mode2.stick_right_x = rx8>>4;
      _out_buffer[player].mode2.stick_right_y = ry8>>4;
      _out_buffer[player].mode2.analog_trigger_l  = lt8>>4;
      _out_buffer[player].mode2.analog_trigger_r  = rt8>>4;
      _out_buffer[player].mode2.analog_a = 0;
      _out_buffer[player].mode2.analog_b = 0;
    break;

    case 4:
      _out_buffer[player].mode4.stick_left_x  = lx8;
      _out_buffer[player].mode4.stick_left_y  = ly8;
      _out_buffer[player].mode4.stick_right_x = rx8;
      _out_buffer[player].mode4.stick_right_y = ry8;
      _out_buffer[player].mode4.analog_a = 0;
      _out_buffer[player].mode4.analog_b = 0;
    break;
  }
}

void gamecube_comms_task()
{
  if (!_gc_running)
  {
    mutex_init(&timestamp_mutex);
    for(uint i = 0; i < 4; i++)
    {
      mutex_init(&(_gamepad_mutex[i]));
      _out_buffer[i].blank_2 = 1;
      _out_buffer[i].stick_left_x = 127;
      _out_buffer[i].stick_left_y = 127;

      _out_buffer[i].stick_right_x = 127;
      _out_buffer[i].stick_right_y = 127;
    }

    sleep_ms(150);
    _gamecube_offset = pio_add_program(GAMEPAD_PIO, &joybus_program);

    _gamecube_irq_tx = PIO0_IRQ_0;
    _gamecube_irq_rx = PIO0_IRQ_1;

    pio_set_irq0_source_enabled(GAMEPAD_PIO, pis_interrupt0, true);
    pio_set_irq1_source_enabled(GAMEPAD_PIO, pis_interrupt1, true);

    irq_set_exclusive_handler(_gamecube_irq_tx, _gamecube_isr_txdone);
    irq_set_exclusive_handler(_gamecube_irq_rx, _gamecube_isr_rxgot);

    for(uint i = 0; i < JOYBUS_CHANNELS; i++)
    {
      joybus_program_init(SYS_CLK_SPEED_HZ, GAMEPAD_PIO, i, _gamecube_offset, joybus_pins[i], &(_gamecube_c[i]));

      memcpy(&(_cannedProbeTX[i]), _cannedProbe, 3);
      memcpy(&(_cannedOriginTX[i]), _cannedOrigin, 10);

      _joybus_state[i].dma = dma_claim_unused_channel(true);
      _joybus_dma_config[i] = dma_channel_get_default_config(_joybus_state[i].dma);
      channel_config_set_transfer_data_size(&(_joybus_dma_config[i]), DMA_SIZE_8);
      channel_config_set_read_increment(&(_joybus_dma_config[i]), true);
      channel_config_set_dreq(&(_joybus_dma_config[i]), DREQ_PIO0_TX0+i);

      dma_channel_configure(
          _joybus_state[i].dma,
          &(_joybus_dma_config[i]),
          &pio0_hw->txf[i], // Write address (only need to set this once)
          &(_cannedProbeTX[i]),
          3, // Write the same value many times
          false             // Don't start yet
      );
    }

    //irq_set_enabled(_gamecube_irq_tx, true);
    irq_set_enabled(_gamecube_irq_rx, true);
    _gc_running = true;

    // DEBUG
    //_message_handle_connect(0, true);
  }
  else
  {
    
    uint32_t timestamp = gamecube_get_timestamp();

    for(uint i = 0; i < JOYBUS_CHANNELS; i++)
    {
      if (interval_resettable_run(timestamp, 100000, _gc_got_data[i], &(interval[i])))
      {
        _gamecube_reset_state(i);
      }
      if (_gc_got_data[i]) _gc_got_data[i]=false;
      
    }

    // Get any pending messages and act on them
    static intercore_msg_s core0msg = {0};
    static uni_gamepad_t core0msggp = {0};
    if(core0_get_message_safe(&core0msg))
    {
      switch(core0msg.id)
      {
        default:
        break;

        case IC_MSG_CONNECT:
          _message_handle_connect(core0msg.data, true);
        break;

        case IC_MSG_DISCONNECT:
          _message_handle_connect(core0msg.data, false);
        break;

        case IC_MSG_INPUT:
          _message_handle_input( (core0msg.data & 0b11), &core0msg);
        break;
      }
    }

    // Call our hook safely (16ms spacing)
    {
      static interval_s safe_interval = {0};
      static uint8_t newrumblestate = 0b0000;

      newrumblestate |= (_joybus_state[0].rumble<<3) | (_joybus_state[1].rumble<<2) | (_joybus_state[2].rumble<<1) | (_joybus_state[3].rumble);

      if(interval_run(timestamp, 16000, &safe_interval))
      {
        static intercore_msg_s core1outmsg = {0};
        static uint8_t currentrumble = 0;
        
        core1outmsg.id = IC_MSG_RUMBLE;

        if(currentrumble != newrumblestate)
        {
          core1outmsg.data = newrumblestate;
          core1_send_message_safe(&core1outmsg);
          currentrumble = newrumblestate;
          
        }
        newrumblestate = 0;
        
        uni_run_platform_hook_safe();
      }
    }

  }
}