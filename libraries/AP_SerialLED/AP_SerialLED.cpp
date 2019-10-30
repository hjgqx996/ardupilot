/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
  AP_SerialLED for controlling serial connected LEDs using WS2812B protocol
 */

#include "AP_SerialLED.h"
#include <AP_Math/AP_Math.h>

extern const AP_HAL::HAL& hal;

AP_SerialLED AP_SerialLED::singleton;

AP_SerialLED::AP_SerialLED()
{
}

// set number of LEDs per pin
bool AP_SerialLED::set_num_LEDs(uint8_t chan, uint8_t num_leds)
{
    return hal.rcout->set_neopixel_num_LEDs(chan+1, num_leds);
}

void AP_SerialLED::set_RGB(uint8_t chan, uint32_t ledmask, uint8_t red, uint8_t green, uint8_t blue)
{
    hal.rcout->set_neopixel_rgb_data(chan+1, ledmask, red, green, blue);
}

// trigger sending of LED changes to LEDs
void AP_SerialLED::send(void)
{
    hal.rcout->neopixel_send();
}

/*
  set a color on a 0 to 1 scale following a classic rainbow. Useful for scripting
*/
void AP_SerialLED::set_Rainbow(uint8_t chan, uint32_t ledmask, float v)
{
    static const struct {
        uint8_t rgb[3];
    } rainbow[] = {
        { 255, 0, 0 },
        { 255, 127, 0 },
        { 255, 255, 0 },
        { 0,   255, 0 },
        { 0,   0,   255 },
        { 75,  0,   130 },
        { 143, 0,   255 },
    };
    const uint8_t num_rows = ARRAY_SIZE(rainbow);
    uint8_t row = constrain_int16(v * (num_rows-1), 0, num_rows-1);
    const float v0 = row / float(num_rows-1);
    const float v1 = (row+1) / float(num_rows-1);
    const float p = (v - v0) / (v1 - v0);
    uint8_t r = rainbow[row].rgb[0] + p * (float(rainbow[row+1].rgb[0]) - float(rainbow[row].rgb[0]));
    uint8_t g = rainbow[row].rgb[1] + p * (float(rainbow[row+1].rgb[1]) - float(rainbow[row].rgb[1]));
    uint8_t b = rainbow[row].rgb[2] + p * (float(rainbow[row+1].rgb[2]) - float(rainbow[row].rgb[2]));
    set_RGB(chan, ledmask, r, g, b);
}

