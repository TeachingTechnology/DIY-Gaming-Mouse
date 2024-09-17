/*
 * Copyright (c) 2024, wareya
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <SPI.h>
#include <mbed.h>
#include <Adafruit_NeoPixel.h>

REDIRECT_STDOUT_TO(Serial);

#include <PluggableUSBHID.h>

#include "srom_dummy_blank.h"
#include "relmouse_16.h"
USBMouse16 mouse(false);

#define REG_PRODUCT_ID (0x00)
#define REG_REVISION_ID (0x01)
#define REG_MOTION (0x02)
#define REG_DELTA_X_L (0x03)
#define REG_DELTA_X_H (0x04)
#define REG_DELTA_Y_L (0x05)
#define REG_DELTA_Y_H (0x06)
#define REG_SQUAL (0x07)
#define REG_RAW_DATA_SUM (0x08)
#define REG_MAX_RAW_DATA (0x09)
#define REG_MIN_RAW_DATA (0x0A)
#define REG_SHUTTER_LOWER (0x0B)
#define REG_SHUTTER_UPPER (0x0C)
#define REG_CONTROL (0x0D)

#define REG_CONFIG1 (0x0F)
#define REG_CONFIG2 (0x10)
#define REG_ANGLE_TUNE (0x11)
#define REG_FRAME_CAPTURE (0x12)
#define REG_SROM_ENABLE (0x13)
#define REG_RUN_DOWNSHIFT (0x14)
#define REG_REST1_RATE_LOWER (0x15)
#define REG_REST1_RATE_UPPER (0x16)
#define REG_REST1_DOWNSHIFT (0x17)
#define REG_REST2_RATE_LOWER (0x18)
#define REG_REST2_RATE_UPPER (0x19)
#define REG_REST2_DOWNSHIFT (0x1A)
#define REG_REST3_RATE_LOWER (0x1B)
#define REG_REST3_RATE_UPPER (0x1C)

#define REG_OBSERVATION (0x24)
#define REG_DATA_OUT_LOWER (0x25)
#define REG_DATA_OUT_UPPER (0x26)

#define REG_RAW_DATA_DUMP (0x29)
#define REG_SROM_ID (0x2A)
#define REG_MIN_SQ_RUN (0x2B)
#define REG_RAW_DATA_THRESHOLD (0x2C)

#define REG_CONFIG5 (0x2F)

#define REG_POWER_UP_RESET (0x3A)
#define REG_SHUTDOWN (0x3B)

#define REG_INVERSE_PRODUCT_ID (0x3F)

#define REG_LIFTCUTOFF_TUNE3 (0x41)
#define REG_ANGLE_SNAP (0x42)

#define REG_LIFTCUTOFF_TUNE1 (0x4A)

#define REG_MOTION_BURST (0x50)

#define REG_LIFTCUTOFF_TUNE_TIMEOUT (0x58)

#define REG_LIFTCUTOFF_TUNE_MIN_LENGTH (0x5A)

#define REG_SROM_LOAD_BURST (0x62)
#define REG_LIFT_CONFIG (0x63)
#define REG_RAW_DATA_BURST (0x64)
#define REG_LIFTOFF_TUNE2 (0x65)

#define CONFIG2_REST_ENABLED (0x30)
#define CONFIG2_REPORT_MODE (0x04)


#define BUTTONS_ON 0
#define BUTTONS_OFF 1

#define BUTTON_M1 2
#define BUTTON_M2 3
#define BUTTON_M3 4
#define BUTTON_M4 6
#define BUTTON_M5 7
#define BUTTON_DPI 5

#define ENCODER_A 8
#define ENCODER_B 10
#define ENCODER_COM 9

#define PIN_NCS 17
#define PIN_MOSI 19
#define PIN_MISO 16

SPISettings spisettings(2000000, MSBFIRST, SPI_MODE3);
MbedSPI spi(PIN_MISO, PIN_MOSI, 18);

// DPI Range and Variables
#define MIN_DPI 3
#define MAX_DPI 11
uint8_t current_dpi = MAX_DPI;  // Start with the current DPI value set to 11

// Debouncing variables for BUTTON_DPI
unsigned long last_dpi_press_time = 0;
unsigned long debounce_delay = 200;  // 200ms debounce delay


// Which pin on the Arduino is connected to the NeoPixels?
#define PIN 28
// How many NeoPixels are attached to the Arduino?
#define NUMPIXELS 2
Adafruit_NeoPixel pixels(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);

void setup_buttons()
{
    pinMode(BUTTONS_OFF, INPUT);
    pinMode(BUTTONS_ON, INPUT);
    pinMode(BUTTON_M1, INPUT_PULLUP);
    pinMode(BUTTON_M2, INPUT_PULLUP);
    pinMode(BUTTON_M3, INPUT_PULLUP);
    pinMode(BUTTON_M4, INPUT_PULLUP);
    pinMode(BUTTON_M5, INPUT_PULLUP);
    pinMode(BUTTON_DPI, INPUT_PULLUP);
    
    pinMode(ENCODER_A, INPUT_PULLUP);
    pinMode(ENCODER_B, INPUT_PULLUP);
    pinMode(ENCODER_COM, OUTPUT);
    
    digitalWrite(ENCODER_COM, LOW);
}

void setup()
{
    Serial1.begin(1000000);
    Serial.begin(115200);  // Start Serial for logging
    
    delay(3000);
    
    // SCK, MISO, MOSI, SS
    SPI.begin();
    pinMode(PIN_NCS, OUTPUT);
    
    digitalWrite(PIN_NCS, HIGH);
    delayMicroseconds(1);
    digitalWrite(PIN_NCS, LOW);
    delayMicroseconds(1);
    digitalWrite(PIN_NCS, HIGH);
    delayMicroseconds(1);
    
    pmw3360_boot();
    
    delay(10);
    
    pmw3360_config();  // Set initial DPI value
    
    setup_buttons();

    pixels.begin(); // INITIALIZE NeoPixel strip object (REQUIRED)

}

uint32_t pins_state = 0;
uint8_t buttons = 0;
// 8ms latch time
uint8_t buttons_latch_max = 8;
uint8_t buttons_latch[5] = {0, 0, 0, 0, 0};

volatile uint32_t * pad_control = (uint32_t *)0x4001c000;
volatile uint32_t * gpio_oe_set = (uint32_t *)0xd0000024;
volatile uint32_t * gpio_oe_clr = (uint32_t *)0xd0000028;
volatile uint32_t * gpio_in = (uint32_t *)0xd0000004;





void update_buttons()
{
    // change BUTTONS_OFF to high impedance and BUTTONS_ON to output
    *gpio_oe_clr = 1 << BUTTONS_OFF;
    *gpio_oe_set = 1 << BUTTONS_ON;
    
    // read ON pins
    digitalWrite(BUTTONS_ON, LOW);
    delayMicroseconds(1);
    uint32_t pins_on = ~*gpio_in;
    
    // change BUTTONS_OFF to high impedance and BUTTONS_ON to output
    *gpio_oe_set = 1 << BUTTONS_OFF;
    *gpio_oe_clr = 1 << BUTTONS_ON;
    
    // read OFF pins
    digitalWrite(BUTTONS_OFF, LOW);
    delayMicroseconds(1);
    uint32_t pins_off = ~*gpio_in;
    
    // update pin state
    uint32_t pins_update = pins_on ^ pins_off;
    pins_state = (pins_state & (~pins_update)) | (pins_on & pins_update);
    
    uint8_t next_buttons =
          ((!!(pins_state & (1 << BUTTON_M1))))
        | ((!!(pins_state & (1 << BUTTON_M2))) << 1)
        | ((!!(pins_state & (1 << BUTTON_M3))) << 2)  // Only BUTTON_M3
        | ((!!(pins_state & (1 << BUTTON_M4))) << 3)
        | ((!!(pins_state & (1 << BUTTON_M5))) << 4);
    
    // update latch
    uint8_t ok_mask = 
          ((buttons_latch[0] == 0))
        | ((buttons_latch[1] == 0) << 1)
        | ((buttons_latch[2] == 0) << 2)
        | ((buttons_latch[3] == 0) << 3)
        | ((buttons_latch[4] == 0) << 4);
    
    next_buttons = (next_buttons & ok_mask) | (buttons & ~ok_mask);
    
    // update latch timings
    for (int i = 0; i < 5; i++)
    {
        if (((next_buttons ^ buttons) >> i) & 1)
            buttons_latch[i] = buttons_latch_max;
        else if (buttons_latch[i])
            buttons_latch[i] -= 1;
    }
    
    buttons = next_buttons;

    // Handle DPI button press separately with debounce
    if ((pins_state & (1 << BUTTON_DPI)) && (millis() - last_dpi_press_time > debounce_delay)) {
        change_dpi();
        last_dpi_press_time = millis();  // Update the last press time
    }
}

// Function to change DPI and update the sensor
void change_dpi() {
    // Increment DPI
    current_dpi += 1;
    if (current_dpi > MAX_DPI) {
        current_dpi = MIN_DPI;  // Wrap around back to 3
    }

    // Write the new DPI setting to the sensor
    spi_write(REG_CONFIG1, current_dpi);

    // Calculate new hue: blue (160) to red (0)
    uint16_t target_hue;
    if (current_dpi == MIN_DPI) {
        // Flash white for visual feedback before resetting to blue
        for (int i = 0; i < NUMPIXELS; i++) {
            pixels.setPixelColor(i, pixels.Color(255, 255, 255));  // Flash white
        }
        pixels.show();
        delay(100);  // Brief delay for flash effect

        target_hue = 160 * 256;  // Reset to blue when DPI is 3
    } else {
        // Map DPI from 3 to 11 to hue range (blue to red)
        target_hue = map(current_dpi, MIN_DPI, MAX_DPI, 160 * 256, 0);
    }

    // Get current color hue (assume it's stored in a variable `current_hue`)
    uint16_t current_hue = pixels.getPixelColor(0);  // For simplicity, assume the hue of the first pixel

    // Smoothly fade from current_hue to target_hue
    int steps = 30;  // Number of steps for smooth transition
    for (int i = 0; i <= steps; i++) {
        uint16_t interpolated_hue = current_hue + ((target_hue - current_hue) * i) / steps;  // Linear interpolation

        // Update NeoPixel color based on the interpolated hue
        for (int j = 0; j < NUMPIXELS; j++) {
            pixels.setPixelColor(j, pixels.ColorHSV(interpolated_hue, 255, 255));  // Full saturation, full brightness
        }
        pixels.show();  // Show the updated color
        pixels.setBrightness(50);

        delay(10);  // Small delay between steps for smooth fading effect
    }

    // Update the current_hue to the target_hue (if you're tracking the hue globally)
    // current_hue = target_hue;
    Serial.print("DPI changed to: ");
    Serial.println(current_dpi);
}

uint8_t wheel_state_a = 0;
uint8_t wheel_state_b = 0;
uint8_t wheel_state_output = 0;
int8_t wheel_progress = 0;

void update_wheel()
{
    uint8_t wheel_new_a = digitalRead(ENCODER_A);
    uint8_t wheel_new_b = digitalRead(ENCODER_B);
    
    if (wheel_new_a != wheel_state_a || wheel_new_b != wheel_state_b)
    {
        if (wheel_new_a == wheel_new_b && wheel_state_output != wheel_new_a)
        {
            // when scrolling up, B changes first. when scrolling down, A changes first
            if (wheel_new_b == wheel_state_b)
                wheel_progress += 1;
            else if(wheel_new_a == wheel_state_a)
                wheel_progress -= 1;
            
            wheel_state_output = wheel_new_a;
        }
        
        wheel_state_a = wheel_new_a;
        wheel_state_b = wheel_new_b;
    }
}

int n = 0;
int usb_hid_poll_interval = 1; 

struct MotionBurstData {
    uint8_t motion;
    uint8_t observation;
    int16_t x;
    int16_t y;
    uint8_t squal;
    uint8_t raw_sum;
    uint8_t raw_max;
    uint8_t raw_min;
    uint16_t shutter;
};

MotionBurstData spi_read_motion_burst(bool do_update_wheel);

void loop()
{
    update_wheel();
    delayMicroseconds(250);
    update_wheel();
    
    int16_t x = 0;
    int16_t y = 0;
    
    MotionBurstData data = spi_read_motion_burst(true);
    if (data.motion)
    {
        x = data.x;
        y = data.y;
    }
    
    update_wheel();
    update_buttons();
    
    int8_t wheel = wheel_progress;
    wheel_progress = 0;
    
    mouse.update(x, y, buttons, wheel);


}

void pmw3360_boot()
{
    spi_write(REG_POWER_UP_RESET, 0x5A);
    delay(50);
    
    spi_read(0x02);
    spi_read(0x03);
    spi_read(0x04);
    spi_read(0x05);
    spi_read(0x06);
    
    srom_upload();
}

void pmw3360_config()
{
    spi_write(REG_CONFIG1, current_dpi);  // Set DPI to current_dpi value
}

void spi_begin()
{
    SPI.beginTransaction(spisettings);
}
void spi_end()
{
    SPI.endTransaction();
}

void spi_write(byte addr, byte data)
{
    spi_begin();
    
    digitalWrite(PIN_NCS, LOW);
    delayMicroseconds(1); // 120 nanoseconds; t_NCS-SCLK
    
    SPI.transfer(addr | 0x80);
    SPI.transfer(data);
    
    delayMicroseconds(35); // t_SCLK-NCS(write)
    digitalWrite(PIN_NCS, HIGH);
    
    spi_end();
    
    delayMicroseconds(180); // max(t_SWW, t_SWR)
}

byte spi_read(byte addr)
{
    spi_begin();
    
    digitalWrite(PIN_NCS, LOW);
    delayMicroseconds(1); // 120 nanoseconds; t_NCS-SCLK
    
    SPI.transfer(addr & 0x7F);
    
    delayMicroseconds(160); // t_SRAD
    
    byte ret = SPI.transfer(0);
    
    delayMicroseconds(1); // 120 nanoseconds; t_SCLK-NCS(read)
    digitalWrite(PIN_NCS, HIGH);
    
    spi_end();
    
    delayMicroseconds(20); // max(t_SRW, t_SRR)
    
    return ret;
}

MotionBurstData spi_read_motion_burst(bool do_update_wheel)
{
    MotionBurstData ret = {0};
    
    spi_write(REG_MOTION_BURST, 0x00);
    if (do_update_wheel)
        update_wheel();
    
    spi_begin();
    
    digitalWrite(PIN_NCS, LOW);
    delayMicroseconds(1); // 120 nanoseconds; t_NCS-SCLK
    
    SPI.transfer(REG_MOTION_BURST);
    
    delayMicroseconds(35); // t_SRAD_MOTBR
    
    ret.motion = SPI.transfer(0);
    ret.observation = SPI.transfer(0);
    ret.x = SPI.transfer(0);
    ret.x |= ((uint16_t)SPI.transfer(0)) << 8;
    ret.y = SPI.transfer(0);
    ret.y |= ((uint16_t)SPI.transfer(0)) << 8;
    ret.squal = SPI.transfer(0);
    
    digitalWrite(PIN_NCS, HIGH);
    delayMicroseconds(1); // t_BEXIT = 500ns; wait 1000ns (1us)
    
    spi_end();
    
    return ret;
}

void srom_upload()
{
    spi_write(REG_CONFIG2, 0x00);
    spi_write(REG_SROM_ENABLE, 0x1D);
    delay(10);
    
    spi_write(REG_SROM_ENABLE, 0x18);
    
    spi_begin();
    
    digitalWrite(PIN_NCS, LOW);
    delayMicroseconds(1);
    
    SPI.transfer(REG_SROM_LOAD_BURST | 0x80);
    delayMicroseconds(15);
    
    for(size_t i = 0; i < SROM_LENGTH; i += 1)
    {
        SPI.transfer(srom[i]);
        delayMicroseconds(15);
    }
    
    digitalWrite(PIN_NCS, HIGH);
    delayMicroseconds(1);
    
    spi_end();
    
    delayMicroseconds(200);
    
    byte id = spi_read(REG_SROM_ID);
    printf("\n");
    printf("srom id: ");
    printf("%d", id);
    printf("\n");
    
    spi_write(REG_CONFIG2, 0x00);
}
