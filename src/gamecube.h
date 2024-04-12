#include <uni.h>
#include <uni_hid_device.h>
#include <stdint.h>

void gamecube_controller_connect(int idx, bool connected);
void gamecube_comms_update(int idx, uni_gamepad_t *gp);
void gamecube_comms_task();