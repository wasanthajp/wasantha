// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
  additional arming checks for copter
 */
#include "arming_checks.h"
#include <GCS_MAVLink/GCS.h>

const AP_Param::GroupInfo AP_Arming_Copter::var_info[] PROGMEM = {
    // variables from parent vehicle
    AP_NESTEDGROUPINFO(AP_Arming, 0),

    AP_GROUPEND
};

/*
  additional arming checks for copter
 */
bool AP_Arming_Copter::pre_arm_checks(bool report)
{
    // call parent class checks
    bool ret = AP_Arming::pre_arm_checks(report);

    return ret;
}

bool AP_Arming_Copter::manual_transmitter_checks(bool report)
{
    // call parent class checks
    if (!AP_Arming::manual_transmitter_checks(report)) {
        return false;
    }

    // vehicle specific checks
    bool ret = true;

    // check if radio has been calibrated
    if (!_channel_throttle->radio_min.load() && !_channel_throttle->radio_max.load()) {
        ret = false;
    }

    // check channels 1 & 2 have min <= 1300 and max >= 1700
    if (_channel_roll->radio_min > 1300 || _channel_roll->radio_max < 1700 || _channel_pitch->radio_min > 1300 || _channel_pitch->radio_max < 1700) {
        ret = false;
    }

    // check channels 3 & 4 have min <= 1300 and max >= 1700
    if (_channel_throttle->radio_min > 1300 || _channel_throttle->radio_max < 1700 || _channel_yaw->radio_min > 1300 || _channel_yaw->radio_max < 1700) {
        ret = false;
    }

    // check channels 1 & 2 have trim >= 1300 and <= 1700
    if (_channel_roll->radio_trim < 1300 || _channel_roll->radio_trim > 1700 || _channel_pitch->radio_trim < 1300 || _channel_pitch->radio_trim > 1700) {
        ret = false;
    }

    // check channel 4 has trim >= 1300 and <= 1700
    if (_channel_yaw->radio_trim < 1300 || _channel_yaw->radio_trim > 1700) {
        ret = false;
    }

    // display failure
    if (!ret && report) {
        GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_CRITICAL,PSTR("PreArm: RC not calibrated"));
    }

    return ret;
}
