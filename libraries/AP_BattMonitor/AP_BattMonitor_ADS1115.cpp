#include <AP_HAL/AP_HAL.h>
#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>
#include "AP_BattMonitor_ADS1115.h"

extern const AP_HAL::HAL& hal;

/// Constructor
AP_BattMonitor_ADS1115::AP_BattMonitor_ADS1115(AP_BattMonitor &mon,
                                             AP_BattMonitor::BattMonitor_State &mon_state,
                                             AP_BattMonitor_Params &params) :
    AP_BattMonitor_Backend(mon, mon_state, params)
{
    // always healthy
    _state.healthy = true;
}

void AP_BattMonitor_ADS1115::init(void)
{
    adc.init();
}

void AP_BattMonitor_ADS1115::read() {
    adc_report_s reports[6];
    adc.read(reports, ARRAY_SIZE(reports));

    // get voltage
    _state.voltage = reports[3].data * _params._volt_multiplier;

    // calculate time since last current read
    uint32_t tnow = AP_HAL::micros();
    float dt = tnow - _state.last_time_micros;

    // read current
    _state.current_amps = (reports[2].data-_params._curr_amp_offset)*_params._curr_amp_per_volt;

    // update total current drawn since startup
    if (_state.last_time_micros != 0 && dt < 2000000.0f) {
        // .0002778 is 1/3600 (conversion to hours)
        float mah = _state.current_amps * dt * 0.0000002778f;
        _state.consumed_mah += mah;
        _state.consumed_wh  += 0.001f * mah * _state.voltage;
    }

    // record time
    _state.last_time_micros = tnow;
}
