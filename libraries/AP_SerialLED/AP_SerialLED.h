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
#pragma once

#include <AP_HAL/AP_HAL.h>

class AP_SerialLED {
public:
    AP_SerialLED();

    // set number of LEDs per pin
    bool set_num_LEDs(uint8_t chan, uint8_t num_leds);

    // set RGB value on mask of LEDs
    void set_RGB(uint8_t chan, uint32_t ledmask, uint8_t red, uint8_t green, uint8_t blue);

    /*
      set a color on a 0 to 1 scale following a classic rainbow. Useful for scripting
    */
    void set_Rainbow(uint8_t chan, uint32_t ledmask, float v);

    // trigger sending of LED changes to LEDs
    void send(void);

    // singleton support
    static AP_SerialLED *get_singleton(void) {
        return &singleton;
    }

private:
    static AP_SerialLED singleton;
};
