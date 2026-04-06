#pragma once

/*
 * LGFX_ELECROW70_V13.h
 *
 * LovyanGFX display driver for CrowPanel Advance 7.0" V1.3 hardware.
 *
 * V1.3 replaces the TCA9534 I/O expander (0x18) used in V1.0 with the
 * STC8H1K28 co-processor at I2C address 0x30. This co-processor controls:
 *   - Backlight brightness (0=max, 244=min, 245=off)
 *   - Buzzer (246=on, 247=off)
 *   - Speaker amplifier (248=unmute, 249=mute)
 *   - Touch screen reset (250)
 *
 * Display, touch, and RGB bus pins are identical to V1.0.
 *
 * Reference: Elecrow V1.3 factory source code
 *   factory_sourcecode/V1.2/HMI-bigInch7/HMI-bigInch7.ino (V1.2 commands)
 *   example/V1.3/example_code_arduino_7.0/lesson-03/BigInch_LVGL/BigInch_LVGL.ino (V1.3 commands)
 *   example/V1.3/7-1.3mic-spk.ino (V1.3 audio commands)
 */

#define LGFX_USE_V1
#include <LovyanGFX.hpp>
#include <Wire.h>
#include <lgfx/v1/platforms/esp32s3/Bus_RGB.hpp>
#include <lgfx/v1/platforms/esp32s3/Panel_RGB.hpp>

// STC8H1K28 co-processor I2C address (V1.2/V1.3)
#define STC8H_I2C_ADDR 0x30

// STC8H1K28 V1.3 command bytes
#define STC8H_V13_BACKLIGHT_MAX     0     // 0 = maximum brightness
#define STC8H_V13_BACKLIGHT_OFF     245
#define STC8H_V13_BUZZER_ON         246
#define STC8H_V13_BUZZER_OFF        247
#define STC8H_V13_SPEAKER_UNMUTE    248
#define STC8H_V13_SPEAKER_MUTE      249   // referenced but commented in factory code
#define STC8H_V13_TOUCH_RESET       250

#ifndef FREQ_WRITE
#define FREQ_WRITE 14000000
#endif

// Note: LovyanGFX 1.2.0 Panel_FrameBufferBase already calls Cache_WriteBack_Addr()
// per row inside writeImage() on ESP32-S3 — no additional cache flush is needed.
// The display tearing visible during rapid UI updates (e.g. incoming messages) is
// a fundamental limitation of the single-buffer RGB implementation in LovyanGFX
// 1.2.0: the GDMA scans the frame buffer continuously while LVGL writes to it.
// Double-buffering with vsync swap would be required to eliminate this.
using Panel_RGB_V13 = lgfx::Panel_RGB;

class LGFX_ELECROW70_V13 : public lgfx::LGFX_Device
{
    lgfx::Bus_RGB _bus_instance;
    Panel_RGB_V13 _panel_instance;
    lgfx::Touch_GT911 _touch_instance;

  public:
    const uint16_t screenWidth = 800;
    const uint16_t screenHeight = 480;

    bool hasButton(void) { return false; }

    // Send a single-byte command to the STC8H1K28 co-processor
    static bool sendCommand(uint8_t cmd)
    {
        Wire.beginTransmission(STC8H_I2C_ADDR);
        Wire.write(cmd);
        return (Wire.endTransmission() == 0);
    }

    // Check if a device responds at the given I2C address
    static bool i2cProbe(uint8_t addr)
    {
        Wire.beginTransmission(addr);
        return (Wire.endTransmission() == 0);
    }

    bool init_impl(bool use_reset, bool use_clear) override
    {
        // GT911 touch controller power-on sequence to select address 0x5D:
        // INT (GPIO1) must be held LOW *before* and *during* the reset cycle so
        // the GT911 latches address 0x5D.  Assert it first, then trigger the
        // reset via STC8H1K28 command 250, hold through settling, then release.
        pinMode(1, OUTPUT);
        digitalWrite(1, LOW);
        sendCommand(STC8H_V13_TOUCH_RESET); // reset while INT is already asserted
        delay(120);                          // hold INT low through reset + settling
        pinMode(1, INPUT);
        delay(50);

        // Wait for STC8H1K28 (0x30) and GT911 (0x5D) to be ready.
        int retries = 50; // ~5 seconds max
        while (retries-- > 0) {
            if (i2cProbe(STC8H_I2C_ADDR) && i2cProbe(0x5D)) {
                break;
            }
            delay(100);
        }

        // Mute speaker amplifier during init (matches factory behavior)
        sendCommand(STC8H_V13_SPEAKER_MUTE);

        // Turn on backlight at maximum brightness
        sendCommand(STC8H_V13_BACKLIGHT_MAX);

        return LGFX_Device::init_impl(use_reset, use_clear);
    }

    LGFX_ELECROW70_V13(void)
    {
        // --- Panel configuration (identical to V1.0) ---
        {
            auto cfg = _panel_instance.config();

            cfg.memory_width = screenWidth;
            cfg.memory_height = screenHeight;
            cfg.panel_width = screenWidth;
            cfg.panel_height = screenHeight;
            cfg.offset_x = 0;
            cfg.offset_y = 0;
            cfg.offset_rotation = 0;
            _panel_instance.config(cfg);
        }

        {
            auto cfg = _panel_instance.config_detail();
            cfg.use_psram = 1;
            _panel_instance.config_detail(cfg);
        }

        // --- RGB bus configuration (identical to V1.0) ---
        {
            auto cfg = _bus_instance.config();
            cfg.panel = &_panel_instance;
            cfg.pin_d0 = GPIO_NUM_21;  // B0
            cfg.pin_d1 = GPIO_NUM_47;  // B1
            cfg.pin_d2 = GPIO_NUM_48;  // B2
            cfg.pin_d3 = GPIO_NUM_45;  // B3
            cfg.pin_d4 = GPIO_NUM_38;  // B4
            cfg.pin_d5 = GPIO_NUM_9;   // G0
            cfg.pin_d6 = GPIO_NUM_10;  // G1
            cfg.pin_d7 = GPIO_NUM_11;  // G2
            cfg.pin_d8 = GPIO_NUM_12;  // G3
            cfg.pin_d9 = GPIO_NUM_13;  // G4
            cfg.pin_d10 = GPIO_NUM_14; // G5
            cfg.pin_d11 = GPIO_NUM_7;  // R0
            cfg.pin_d12 = GPIO_NUM_17; // R1
            cfg.pin_d13 = GPIO_NUM_18; // R2
            cfg.pin_d14 = GPIO_NUM_3;  // R3
            cfg.pin_d15 = GPIO_NUM_46; // R4

            cfg.pin_henable = GPIO_NUM_42;
            cfg.pin_vsync = GPIO_NUM_41;
            cfg.pin_hsync = GPIO_NUM_40;
            cfg.pin_pclk = GPIO_NUM_39;
            cfg.freq_write = FREQ_WRITE;

            cfg.hsync_polarity = 0;
            cfg.hsync_front_porch = 8;
            cfg.hsync_pulse_width = 4;
            cfg.hsync_back_porch = 8;

            cfg.vsync_polarity = 0;
            cfg.vsync_front_porch = 8;
            cfg.vsync_pulse_width = 4;
            cfg.vsync_back_porch = 8;

            cfg.pclk_idle_high = 1;

            _bus_instance.config(cfg);
        }
        _panel_instance.setBus(&_bus_instance);

        // --- Touch configuration (identical to V1.0) ---
        {
            auto cfg = _touch_instance.config();
            cfg.x_min = 0;
            cfg.x_max = 800;
            cfg.y_min = 0;
            cfg.y_max = 480;
            // GPIO1 is the GT911 interrupt output.  We drive it LOW manually
            // during init_impl() to latch address 0x5D, then release it to INPUT.
            // By the time LGFX_Device::init_impl() configures the touch controller
            // (after our override returns), GPIO1 is already an input and the GT911
            // drives it as an active-low IRQ.  Exposing it here lets task_handler()
            // use it as the ESP32 light-sleep wake source so the device can wake
            // from touch; without it task_handler() has no wake GPIO and leaves
            // the display stuck off with touch input disabled.
            cfg.pin_int = GPIO_NUM_1;
            cfg.pin_rst = -1;
            cfg.bus_shared = false; // GT911 is on I2C; RGB bus is independent
            cfg.offset_rotation = 0;

            cfg.i2c_port = 0;
            cfg.i2c_addr = 0x5D;
            cfg.pin_sda = GPIO_NUM_15;
            cfg.pin_scl = GPIO_NUM_16;
            cfg.freq = 400000;
            _touch_instance.config(cfg);
            _panel_instance.setTouch(&_touch_instance);
        }

        setPanel(&_panel_instance);
    }

    // --- Power management ---

    void sleep(void)
    {
        sendCommand(STC8H_V13_BACKLIGHT_OFF);
        // Do NOT call _panel->setSleep(true) — Panel_RGB on ESP32-S3 tears down
        // the DMA engine, which causes a crash if LVGL attempts a flush afterward.
    }

    void wakeup(void)
    {
        sendCommand(STC8H_V13_BACKLIGHT_MAX);
        // No _panel->setSleep(false) needed since we never called setSleep(true).
    }

    // --- Backlight brightness control ---
    // V1.3: 0 = maximum, 244 = minimum, 245 = off
    void setBacklightBrightness(uint8_t level)
    {
        if (level > 245) level = 245;
        sendCommand(level);
    }

    // --- Buzzer control (V1.3: I2C on/off, no frequency control) ---
    void buzzerOn(void)  { sendCommand(STC8H_V13_BUZZER_ON); }
    void buzzerOff(void) { sendCommand(STC8H_V13_BUZZER_OFF); }

    // --- Speaker amplifier control ---
    void speakerUnmute(void) { sendCommand(STC8H_V13_SPEAKER_UNMUTE); }
    void speakerMute(void)   { sendCommand(STC8H_V13_SPEAKER_MUTE); }
};
