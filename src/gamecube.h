#include <uni.h>
#include <uni_hid_device.h>
#include <stdint.h>
#include "types.h"

typedef struct 
{
  uint32_t this_time;
  uint32_t last_time;
} interval_s;

bool interval_resettable_run(uint32_t timestamp, uint32_t interval, bool reset, interval_s *state);
uint32_t gamecube_get_timestamp();
void my_platform_set_rumble(uint8_t idx, bool rumble);
void gamecube_controller_connect(int idx, bool connected);
void gamecube_comms_update(int idx, uni_gamepad_t *gp);
void gamecube_comms_task();