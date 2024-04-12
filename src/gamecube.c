#include <stdint.h>
#include <stdbool.h>
#include "gamecube.h"
#include <hardware/pio.h>
#include "types.h"
#include <pico/multicore.h>
#include "hardware/dma.h"
#include <pico/cyw43_arch.h>

#include <uni.h>
#include <uni_hid_device.h>

#include "sdkconfig.h"

#include "joybus.pio.h"
#include "hardware/structs/pio.h"

#define PIO_SM_0_IRQ 0b0001
#define PIO_SM_1_IRQ 0b0010
#define PIO_SM_2_IRQ 0b0100
#define PIO_SM_3_IRQ 0b1000

#define GAMEPAD_PIO pio0
#define GAMEPAD_SM 0

#define CLAMP_0_255(value) ((value) < 0 ? 0 : ((value) > 255 ? 255 : (value)))

#define JOYBUS_CHANNELS 4

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
bool _gc_running = false;
bool _gc_rumble = false;


volatile gamecube_input_s _in_buffer[4] = {0};
volatile gamecube_input_s _out_buffer[4] = {0};
volatile uint8_t _dmaOut[4][8];

typedef struct
{
  uint8_t byteCounter; // How many bytes left to read before we must respond
  uint8_t workingCmd;  // The current working command we received
  bool lock; // Lock so it doesn't get interrupted
  bool rumble;         // if rumble is enabled or not
  int dma;
} joybus_state_s;

volatile joybus_state_s _joybus_state[4];

volatile uni_gamepad_t _incoming_gamepad[4];
volatile bool _in_buffer_update = false;

dma_channel_config _joybus_dma_config[4];

uint32_t _gamepad_owner_0;
uint32_t _gamepad_owner_1;
auto_init_mutex(_gamepad_mutex);

#define ALIGNED_JOYBUS_8(val) ((val) << 24)

volatile uint8_t _cannedProbe[3] = {0x09, 0x00, 0x03};
volatile uint8_t _cannedProbeTX[4][3];
volatile uint8_t _cannedOrigin[10] = {0x00, 128, 128, 128, 128, 128, 0x00, 0x00, 0x00, 0x00};
volatile uint8_t _cannedOriginTX[4][10];

void _gamecube_send_probe(uint sm)
{
  dma_channel_set_trans_count(_joybus_state[sm].dma, 3, false);
  dma_channel_set_read_addr(_joybus_state[sm].dma, &(_cannedProbeTX[sm]), true);
}

void _gamecube_send_origin(uint sm)
{
  dma_channel_set_trans_count(_joybus_state[sm].dma, 10, false);
  dma_channel_set_read_addr(_joybus_state[sm].dma, &(_cannedOriginTX[sm]), true);
}

void _gamecube_send_poll(uint sm)
{
  dma_channel_set_trans_count(_joybus_state[sm].dma, 8, false);
  dma_channel_set_read_addr(_joybus_state[sm].dma, &(_out_buffer[sm]), true);
}

void _gamecube_reset_state(uint sm)
{
  joybus_set_in(true, GAMEPAD_PIO, sm, _gamecube_offset, &(_gamecube_c[sm]), PIN_JOYBUS_BASE + sm);
  _joybus_state[sm].byteCounter = 3;
  _joybus_state[sm].workingCmd = 0x00;
}

#define BYTECOUNT_DEFAULT 2

// Returns true if we are at the end of a command area (start response)
bool __time_critical_func(_gamecube_command_handler)(uint sm, uint8_t mask)
{
  bool ret = false;
  uint8_t dat = 0;

  if(_joybus_state[sm].byteCounter==BYTECOUNT_DEFAULT)
  {
    _joybus_state[sm].workingCmd = pio_sm_get(GAMEPAD_PIO, sm);
  }
  else
  {
    dat = pio_sm_get(GAMEPAD_PIO, sm);
  }
 
  switch (_joybus_state[sm].workingCmd)
  {
  default:
    break;

  case 0x42:
    if(_joybus_state[sm].byteCounter == 0)
    {
      _gc_got_data[sm] = true;
      _joybus_state[sm].byteCounter = BYTECOUNT_DEFAULT;
      
      joybus_set_in(false, GAMEPAD_PIO, sm, _gamecube_offset, &(_gamecube_c[sm]), PIN_JOYBUS_BASE + sm);
      pio_set_sm_mask_enabled(GAMEPAD_PIO, (1<<sm), false);
      _gamecube_send_origin(sm);
      pio_set_sm_mask_enabled(GAMEPAD_PIO, (1<<sm), true);
    }
  break;


  case 0x40:

    if (_joybus_state[sm].byteCounter == 0)
    {
      _joybus_state[sm].rumble = ((dat & 0x1) > 0) ? true : false;

      _gc_got_data[sm] = true;
      _joybus_state[sm].byteCounter = BYTECOUNT_DEFAULT;
      
      joybus_set_in(false, GAMEPAD_PIO, sm, _gamecube_offset, &(_gamecube_c[sm]), PIN_JOYBUS_BASE + sm);
      pio_set_sm_mask_enabled(GAMEPAD_PIO, (1<<sm), false);
      _gamecube_send_poll(sm);
      pio_set_sm_mask_enabled(GAMEPAD_PIO, (1<<sm), true);
      
      ret = true;
    }

  break;

  
  case 0x00:
    _gc_got_data[sm] = true;
    _joybus_state[sm].byteCounter = BYTECOUNT_DEFAULT;
    joybus_set_in(false, GAMEPAD_PIO, sm, _gamecube_offset, &(_gamecube_c[sm]), PIN_JOYBUS_BASE + sm);
    pio_set_sm_mask_enabled(GAMEPAD_PIO, (1<<sm), false);
    _gamecube_send_probe(sm);
    pio_set_sm_mask_enabled(GAMEPAD_PIO, (1<<sm), true);
    
    ret = true;
    break;

  case 0x41:
    _gc_got_data[sm] = true;
    _joybus_state[sm].byteCounter = BYTECOUNT_DEFAULT;
    
    joybus_set_in(false, GAMEPAD_PIO, sm, _gamecube_offset, &(_gamecube_c[sm]), PIN_JOYBUS_BASE + sm);
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

volatile bool t = false;

static void _gamecube_isr_rxgot(void)
{
  irq_set_enabled(_gamecube_irq_rx, false);
  if (pio_interrupt_get(GAMEPAD_PIO, 1))
  {
    pio_interrupt_clear(GAMEPAD_PIO, 1);

    for(uint i = 0; i < JOYBUS_CHANNELS; i++)
    {
        if(!pio_sm_is_rx_fifo_empty(GAMEPAD_PIO, i))
        _gamecube_command_handler(i, 0);
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
      joybus_set_in(true, GAMEPAD_PIO, i, _gamecube_offset, &(_gamecube_c[i]), PIN_JOYBUS_BASE+i);
    }
  }

  irq_set_enabled(_gamecube_irq_tx, true);
  
}

void gamecube_comms_update(int idx, uni_gamepad_t *gp)
{
  mutex_enter_blocking(&_gamepad_mutex);
  _incoming_gamepad[0].buttons = gp->buttons;
  _incoming_gamepad[0].dpad = gp->dpad;
  _in_buffer_update = true;
  mutex_exit(&_gamepad_mutex);
}

interval_s interval[4];

void gamecube_comms_task()
{
  

  if (!_gc_running)
  {

    for(uint i = 0; i < 4; i++)
    {
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
      joybus_program_init(SYS_CLK_SPEED_HZ, GAMEPAD_PIO, i, _gamecube_offset, PIN_JOYBUS_BASE+i, &(_gamecube_c[i]));

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
  }
  else
  {
    uint32_t timestamp = time_us_32();

    for(uint i = 0; i < JOYBUS_CHANNELS; i++)
    {
      if (interval_resettable_run(timestamp, 100000, _gc_got_data[i], &(interval[i])))
      {
        _gamecube_reset_state(i);
      }
      if (_gc_got_data[i]) _gc_got_data[i]=false;
    }

      if (_in_buffer_update)
      {
        if(mutex_enter_timeout_ms(&_gamepad_mutex, 4))
        {
          _out_buffer[0].blank_2 = 1;
          _out_buffer[0].button_a        = (_incoming_gamepad[0].buttons & BUTTON_A) ? true : false;
          _out_buffer[0].button_b        = (_incoming_gamepad[0].buttons & BUTTON_B) ? true : false;
          _out_buffer[0].button_x        = (_incoming_gamepad[0].buttons & BUTTON_X) ? true : false;
          _out_buffer[0].button_y        = (_incoming_gamepad[0].buttons & BUTTON_Y) ? true : false;
          _out_buffer[0].button_start    = (_incoming_gamepad[0].misc_buttons & MISC_BUTTON_HOME) ? true : false;

          //_out_buffer.button_l = buttons->trigger_zl;
          //_out_buffer.button_r = buttons->trigger_zr;

          //float lx = (analog->lx * 0.0488f) + 28;
          //float ly = (analog->ly * 0.0488f) + 28;
          //float rx = (analog->rx * 0.0488f) + 28;
          //float ry = (analog->ry * 0.0488f) + 28;

          _out_buffer[0].stick_left_x = 127;//CLAMP_0_255(lx);
          _out_buffer[0].stick_left_y = 127;//CLAMP_0_255(ly);
          _out_buffer[0].stick_right_x = 127;//CLAMP_0_255(rx);
          _out_buffer[0].stick_right_y = 127;//CLAMP_0_255(ry);

          _out_buffer[0].dpad_down     = (_incoming_gamepad[0].dpad & DPAD_DOWN) ? true : false;
          _out_buffer[0].dpad_left     = (_incoming_gamepad[0].dpad & DPAD_LEFT) ? true : false;
          _out_buffer[0].dpad_right    = (_incoming_gamepad[0].dpad & DPAD_RIGHT) ? true : false;
          _out_buffer[0].dpad_up       = (_incoming_gamepad[0].dpad & DPAD_UP) ? true : false;

          int outl = 0;
          int outr = 0;
          _in_buffer_update = false;
          mutex_exit(&_gamepad_mutex);
        }
      }
  }
}