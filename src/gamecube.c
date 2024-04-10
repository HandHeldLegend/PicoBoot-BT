#include <stdint.h>
#include <stdbool.h>
#include "gamecube.h"
#include <hardware/pio.h>
#include "types.h"
#include <pico/multicore.h>
#include "hardware/dma.h"

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

volatile bool _gc_got_data = false;
bool _gc_running = false;
bool _gc_rumble = false;

uint8_t _gamecube_out_buffer[8] = {0};
volatile uint8_t _gamecube_in_buffer[8] = {0};
volatile gamecube_input_s _out_buffer[4] = {0};
volatile uint8_t _dmaOut[4][8];

typedef struct
{
  uint8_t byteCounter; // How many bytes left to read before we must respond
  uint8_t workingCmd;  // The current working command we received
  bool rumble;         // if rumble is enabled or not
  int dma;
} joybus_state_s;

volatile joybus_state_s _joybus_state[4];
volatile uni_gamepad_t _internal_gamepad[4];
dma_channel_config _joybus_dma_config[4];

uint32_t _gamepad_owner_0;
uint32_t _gamepad_owner_1;
auto_init_mutex(_gamepad_mutex);

#define ALIGNED_JOYBUS_8(val) ((val) << 24)

volatile uint8_t _cannedProbe[3] = {0x09, 0x00, 0x03};
volatile uint8_t _cannedOrigin[10] = {0x00, 0x80, 0x80, 0x80, 0x80, 0x80, 0x00, 0x00, 0x00, 0x00};

void _gamecube_send_probe(uint sm)
{
  dma_channel_configure(
        _joybus_state[sm].dma,
        &(_joybus_dma_config[sm]),
        &pio0_hw->txf[sm], // Write address (only need to set this once)
        _cannedProbe,
        3, // Write the same value many times
        true             // Don't start yet
    );
}

void _gamecube_send_origin(uint sm)
{
  dma_channel_configure(
        _joybus_state[sm].dma,
        &(_joybus_dma_config[sm]),
        &pio0_hw->txf[sm], // Write address (only need to set this once)
        _cannedOrigin,
        10, // Write the same value many times
        true             // Don't start yet
    );
}

void _gamecube_send_poll(uint sm)
{
  _out_buffer[sm].button_a = 1;

  dma_channel_configure(
        _joybus_state[sm].dma,
        &(_joybus_dma_config[sm]),
        &pio0_hw->txf[sm], // Write address (only need to set this once)
        &(_out_buffer[sm]),
        8, // Write the same value many times
        true             // Don't start yet
    );
}

void _gamecube_reset_state(uint sm)
{
  joybus_set_in(true, GAMEPAD_PIO, GAMEPAD_SM, _gamecube_offset, &(_gamecube_c[sm]), PIN_JOYBUS_BASE + sm);
}

void __time_critical_func(_gamecube_command_handler)(uint sm)
{
  if (_joybus_state[sm].workingCmd == 0x40)
  {

    _joybus_state[sm].byteCounter -= 1;
    uint8_t dat = pio_sm_get(GAMEPAD_PIO, sm);
    if (_joybus_state[sm].byteCounter == 0)
    {
      _joybus_state[sm].rumble = ((dat & 0x1) > 0) ? true : false;

      joybus_set_in(false, GAMEPAD_PIO, sm, _gamecube_offset, &(_gamecube_c[sm]), PIN_JOYBUS_BASE + sm);
      _gamecube_send_poll(sm);
    }

  }
  else
  {
    _joybus_state[sm].workingCmd = pio_sm_get(GAMEPAD_PIO, sm);
    switch (_joybus_state[sm].workingCmd)
    {
    default:
      break;
    case 0x00:

      joybus_set_in(false, GAMEPAD_PIO, sm, _gamecube_offset, &(_gamecube_c[sm]), PIN_JOYBUS_BASE + sm);
      _gamecube_send_probe(sm);
      break;

    case 0x40:
      _joybus_state[sm].byteCounter = 2;
      break;

    case 0x41:

      joybus_set_in(false, GAMEPAD_PIO, sm, _gamecube_offset, &(_gamecube_c[sm]), PIN_JOYBUS_BASE + sm);
      _gamecube_send_origin(sm);
      break;
    }
  }
}

static void _gamecube_isr_rxgot(void)
{
  if (pio_interrupt_get(GAMEPAD_PIO, 1))
  {
    _gamecube_command_handler(0);
    pio_interrupt_clear(GAMEPAD_PIO, 1);
  }
}

static void _gamecube_isr_txdone(void)
{
  if (pio_interrupt_get(GAMEPAD_PIO, 0))
  {
    _joybus_state[0].byteCounter = 3;
    joybus_set_in(true, GAMEPAD_PIO, 0, _gamecube_offset, &(_gamecube_c[0]), PIN_JOYBUS_BASE);
    pio_interrupt_clear(GAMEPAD_PIO, 0);
  }
}

void gamecube_comms_update(int idx, uni_gamepad_t *gp)
{
  mutex_enter_blocking(&_gamepad_mutex);
  _internal_gamepad[0].buttons = gp->buttons;
  // memcpy(&(_internal_gamepad[idx]), gp, sizeof(uni_gamepad_t));
  mutex_exit(&_gamepad_mutex);
}

void gamecube_comms_task()
{
  static interval_s interval = {0};

  if (!_gc_running)
  {
    sleep_ms(150);
    _gamecube_offset = pio_add_program(GAMEPAD_PIO, &joybus_program);

    _gamecube_irq_tx = PIO0_IRQ_0;
    _gamecube_irq_rx = PIO0_IRQ_1;

    pio_set_irq0_source_enabled(GAMEPAD_PIO, pis_interrupt0, true);
    pio_set_irq1_source_enabled(GAMEPAD_PIO, pis_interrupt1, true);

    irq_set_exclusive_handler(_gamecube_irq_tx, _gamecube_isr_txdone);
    irq_set_exclusive_handler(_gamecube_irq_rx, _gamecube_isr_rxgot);
    
    joybus_program_init(SYS_CLK_SPEED_HZ, GAMEPAD_PIO, GAMEPAD_SM, _gamecube_offset, PIN_JOYBUS_BASE, &(_gamecube_c[0]));
    _joybus_state[0].dma = dma_claim_unused_channel(true);
    _joybus_dma_config[0] = dma_channel_get_default_config(_joybus_state[0].dma);
    channel_config_set_transfer_data_size(&(_joybus_dma_config[0]), DMA_SIZE_8);
    channel_config_set_read_increment(&(_joybus_dma_config[0]), true);
    channel_config_set_dreq(&(_joybus_dma_config[0]), DREQ_PIO0_TX0);

    // joybus_program_init(SYS_CLK_SPEED_HZ, GAMEPAD_PIO, GAMEPAD_SM + 1, _gamecube_offset, PIN_JOYBUS_BASE + 1, &(_gamecube_c[1]));
    // joybus_program_init(SYS_CLK_SPEED_HZ, GAMEPAD_PIO, GAMEPAD_SM + 2, _gamecube_offset, PIN_JOYBUS_BASE + 2, &_gamecube_c);
    // joybus_program_init(SYS_CLK_SPEED_HZ, GAMEPAD_PIO, GAMEPAD_SM + 3, _gamecube_offset, PIN_JOYBUS_BASE + 3, &_gamecube_c);

    irq_set_enabled(_gamecube_irq_tx, true);
    irq_set_enabled(_gamecube_irq_rx, true);
    _gc_running = true;
  }
  else
  {
    uint32_t timestamp = time_us_32();
    if (interval_resettable_run(timestamp, 100000, _gc_got_data, &interval))
    {
      _gamecube_reset_state(0);
      sleep_ms(24);
    }
    else
    {
      if (mutex_enter_timeout_ms(&_gamepad_mutex, 1))
      {
        _out_buffer[0].blank_2 = 1;
        _out_buffer[0].button_a        = (_internal_gamepad[0].buttons & BUTTON_A) ? true : false;
        _out_buffer[0].button_b        = (_internal_gamepad[0].buttons & BUTTON_B) ? true : false;
        _out_buffer[0].button_x        = (_internal_gamepad[0].buttons & BUTTON_X) ? true : false;
        _out_buffer[0].button_y        = (_internal_gamepad[0].buttons & BUTTON_Y) ? true : false;
        _out_buffer[0].button_start    = (_internal_gamepad[0].misc_buttons & MISC_BUTTON_HOME) ? true : false;

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

        _out_buffer[0].dpad_down     = _internal_gamepad[0].dpad & DPAD_DOWN;
        _out_buffer[0].dpad_left     = _internal_gamepad[0].dpad & DPAD_LEFT;
        _out_buffer[0].dpad_right    = _internal_gamepad[0].dpad & DPAD_RIGHT;
        _out_buffer[0].dpad_up       = _internal_gamepad[0].dpad & DPAD_UP;

        int outl = 0;
        int outr = 0;
        mutex_exit(&_gamepad_mutex);
      }
    }
  }
}