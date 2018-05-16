#pragma once

#include "RangeFinder.h"
#include "RangeFinder_Backend.h"

class AP_RangeFinder_Wasp : public AP_RangeFinder_Backend
{

public:
    // constructor
    AP_RangeFinder_Wasp(RangeFinder::RangeFinder_State &_state,
                                   AP_SerialManager &serial_manager,
                                   uint8_t serial_instance);

    // static detection function
    static bool detect(AP_SerialManager &serial_manager, uint8_t serial_instance);

    // update state
    void update(void);

    static const struct AP_Param::GroupInfo var_info[];

protected:

    virtual MAV_DISTANCE_SENSOR _get_mav_distance_sensor_type() const override {
        return MAV_DISTANCE_SENSOR_LASER;
    }

private:
    // get a reading
    bool get_reading(uint16_t &reading_cm);

    AP_HAL::UARTDriver *uart = nullptr;
    uint32_t last_reading_ms = 0;
    char linebuf[10];
    uint8_t linebuf_len = 0;
    AP_Int8 mavg;
    AP_Int8 mavf;
    const char * config = "\n>FRQ 40\n>FMT ASCII\n>LBE LITTLE\n>THR 232\n>GO\n";
};
