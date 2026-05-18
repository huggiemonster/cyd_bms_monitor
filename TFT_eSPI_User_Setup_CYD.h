// User_Setup.h - CYD (Cheap Yellow Display) ESP32 Configuration
//
// ESP32 Cheap Yellow Display - 2.4" TFT ILI9341 + XPT2046 Touch
//
// INSTALLATION: Copy this file to your TFT_eSPI library:
//   ~/.arduino15/packages/esp32/hardware/esp32/<version>/libraries/TFT_eSPI/User_Setup.h
//
// Or rename to Setup44_CYD.h, place in User_Setups/ folder, and uncomment:
//   #include <User_Setups/Setup44_CYD.h>

#include <stdint.h>

// === Display ===
#define ILI9341_DRIVER

#define TFT_WIDTH  240
#define TFT_HEIGHT 320

// === ESP32 GPIO Pin Definitions (CYD) ===
#define TFT_MOSI 23
#define TFT_SCLK 18
#define TFT_CS   5   // Chip select
#define TFT_DC   22  // Data/Command
#define TFT_RST  19  // Reset (can be -1 if connected to RST)
#define TFT_BL   21  // Backlight

// === Touchscreen (XPT2046) ===
#define TOUCH_CS  33
#define TOUCH_CLK 25
#define TOUCH_MOSI 32
#define TOUCH_MISO 39
#define TOUCH_IRQ  36

// === Calibration (per-unit — may need adjustment) ===
#define TOUCH_MIN_X 200
#define TOUCH_MAX_X 3700
#define TOUCH_MIN_Y 240
#define TOUCH_MAX_Y 3800

// === Other ===
#define TFT_INVERSION_OFF
#define SPI_FREQUENCY  40000000
