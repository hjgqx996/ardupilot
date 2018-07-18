#pragma once

#include <AP_ADC/AP_ADC_ADS1115.h>
#include "AP_BattMonitor.h"
#include "AP_BattMonitor_Backend.h"

class AP_BattMonitor_ADS1115 : public AP_BattMonitor_Backend
{
public:

    /// Constructor
    AP_BattMonitor_ADS1115(AP_BattMonitor &mon, AP_BattMonitor::BattMonitor_State &mon_state, AP_BattMonitor_Params &params);

    /// Read the battery voltage and current.  Should be called at 10hz
    void read();

    /// returns true if battery monitor provides consumed energy info
    bool has_consumed_energy() const override { return true; }

    /// returns true if battery monitor provides current info
    bool has_current() const override { return true; }

    void init(void) override;

protected:

    AP_ADC_ADS1115 adc;
};
