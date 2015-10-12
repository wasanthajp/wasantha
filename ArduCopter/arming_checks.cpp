// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
  additional arming checks for copter
 */
#include "arming_checks.h"
#include <AP_Math/AP_Math.h>
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

bool AP_Arming_Copter::barometer_checks(bool report)
{
    // call parent class checks
    if (!AP_Arming::barometer_checks(report)) {
        return false;
    }

    // Check baro & inav alt are within 1m if EKF is operating in an absolute position mode.
    // Do not check if intending to operate in a ground relative height mode as EKF will output a ground relative height
    // that may differ from the baro height due to baro drift.
    nav_filter_status filt_status;
    filt_status = _inav.get_filter_status();
    bool using_baro_ref = (!filt_status.flags.pred_horiz_pos_rel && filt_status.flags.pred_horiz_pos_abs);
    if (using_baro_ref) {
        if (fabsf(_inav.get_altitude() - barometer.get_altitude()) > COPTER_ARMING_CHECK_ALT_DISPARITY_MAX_CM) {
            if (report) {
                GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_CRITICAL,PSTR("PreArm: Altitude disparity"));
            }
            return false;
        }
    }

    // if we got here checks have passed
    return true;
}

bool AP_Arming_Copter::ins_checks(bool report)
{
    // call parent class checks
    if (!AP_Arming::ins_checks(report)) {
        return false;
    }

    // check ekf attitude, if bad it's usually the gyro biases have not settled
    if (!_inav.get_filter_status().flags.attitude) {
        if (report) {
            GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_CRITICAL,PSTR("PreArm: gyros still settling"));
        }
        return false;
    }

    return true;
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
