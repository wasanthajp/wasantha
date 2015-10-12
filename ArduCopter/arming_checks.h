// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
  additional arming checks for copter
 */

#ifndef __ARMING_CHECKS_H__
#define __ARMING_CHECKS_H__

#include <AP_Arming/AP_Arming.h>

/*
  copter specific arming class
 */
class AP_Arming_Copter : public AP_Arming
{
public:
    AP_Arming_Copter(const AP_AHRS &ahrs_ref, const AP_Baro &baro, Compass &compass, const enum HomeState &home_state) :
        AP_Arming(ahrs_ref, baro, compass, home_state)
    {
            AP_Param::setup_object_defaults(this, var_info);
    }

    bool pre_arm_checks(bool report);

    // var_info for holding Parameter information
    static const struct AP_Param::GroupInfo var_info[];
};

#endif //  __ARMING_CHECKS_H__
