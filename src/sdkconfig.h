//
// Emulate "menuconfig"
//
#define CONFIG_BLUEPAD32_MAX_DEVICES 4
#define CONFIG_BLUEPAD32_MAX_ALLOWLIST 4
#define CONFIG_BLUEPAD32_GAP_SECURITY 1
#define CONFIG_BLUEPAD32_ENABLE_BLE_BY_DEFAULT 1
// #define CONFIG_BLUEPAD32_ENABLE_VIRTUAL_DEVICE_BY_DEFAULT 1

#define CONFIG_BLUEPAD32_PLATFORM_CUSTOM
#define CONFIG_TARGET_PICO_W

// 2 == Info
#define CONFIG_BLUEPAD32_LOG_LEVEL 2

#define SYS_CLK_SPEED_HZ 250000000
#define SYS_CLK_SPEED_KHZ SYS_CLK_SPEED_HZ/1000
#define PIN_JOYBUS_BASE 12
