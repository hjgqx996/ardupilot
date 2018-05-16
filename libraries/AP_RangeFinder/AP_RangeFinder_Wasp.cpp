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

#include <AP_HAL/AP_HAL.h>
#include "AP_RangeFinder_Wasp.h"
#include <AP_SerialManager/AP_SerialManager.h>
#include <GCS_MAVLink/GCS.h>
#include <ctype.h>

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AP_RangeFinder_Wasp::var_info[] = {

    // @Param: MAVG
    // @DisplayName: Moving Average Range
    // @Description: Sets the number of historic range results to use for calculating the current range result. When MAVG is greater than 1, the current range result will be the current measured value averaged with the N-1 previous results
    // @Range 0-255
    // @User: Advanced
    AP_GROUPINFO("WASP_MAVG", 1, AP_RangeFinder_Wasp, mavg, 1),
    AP_GROUPINFO("WASP_MAVF", 3, AP_RangeFinder_Wasp, mavf, 1),
};


/* 
   The constructor also initialises the rangefinder. Note that this
   constructor is not called until detect() returns true, so we
   already know that we should setup the rangefinder
*/
AP_RangeFinder_Wasp::AP_RangeFinder_Wasp(RangeFinder::RangeFinder_State &_state,
                                         AP_SerialManager &serial_manager,
                                         uint8_t serial_instance) :
    AP_RangeFinder_Backend(_state)
{
    uart = serial_manager.find_serial(AP_SerialManager::SerialProtocol_Rangefinder, serial_instance);
    if (uart != nullptr) {
        uart->begin(serial_manager.find_baudrate(AP_SerialManager::SerialProtocol_Rangefinder, serial_instance));
        uart->write((const uint8_t*)config, strlen(config));

        state.backend_var_info = var_info;
        AP_Param::load_object_from_eeprom(this, state.backend_var_info);
    }
}

/* 
   detect if a Lightware rangefinder is connected. We'll detect by
   trying to take a reading on Serial. If we get a result the sensor is
   there.
*/
bool AP_RangeFinder_Wasp::detect(AP_SerialManager &serial_manager, uint8_t serial_instance)
{
    return serial_manager.find_serial(AP_SerialManager::SerialProtocol_Rangefinder, serial_instance) != nullptr;
}

// read - return last value measured by sensor
bool AP_RangeFinder_Wasp::get_reading(uint16_t &reading_cm)
{
    if (uart == nullptr) {
        return false;
    }

    // read any available lines from the lidar
    float sum = 0;
    uint16_t count = 0;
    int16_t nbytes = uart->available();
    while (nbytes-- > 0) {
        char c = uart->read();
//        gcs().send_text(MAV_SEVERITY_INFO, "Bin: %x", (uint32_t)c & 0xff);
        if (c == '\n') {
            linebuf[linebuf_len] = 0;
            sum += (float)atof(linebuf);
            count++;
            linebuf_len = 0;
        } else if (isdigit(c) || c == '.') {
            linebuf[linebuf_len++] = c;
            if (linebuf_len == sizeof(linebuf)) {
                // too long, discard the line
                linebuf_len = 0;
            }
        }
    }

    if (count == 0) {
        return false;
    }
    reading_cm = 100 * sum / count;
//    gcs().send_text(MAV_SEVERITY_INFO, "WASP: %d", reading_cm);
    return true;
}

/* 
   update the state of the sensor
*/
void AP_RangeFinder_Wasp::update(void)
{
    if (get_reading(state.distance_cm)) {
        // update range_valid state based on distance measured
        last_reading_ms = AP_HAL::millis();
        update_status();
    } else if (AP_HAL::millis() - last_reading_ms > 200) {
        set_status(RangeFinder::RangeFinder_NoData);
    }
}
