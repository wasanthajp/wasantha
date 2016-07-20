/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "Copter.h"

/*
 * control_avoid.cpp - init and run calls for AP_Avoidance's AVOID flight mode
 *
 * This is a heavily-cut-down version of GUIDED mode
 */

// initialise avoid_adsb controller
bool Copter::avoid_adsb_init(const bool ignore_checks)
{
    // re-use guided mode
    // Note: this is safe from interference from GCSs and companion computer's whose guided mode
    //       position and velocity requests will be ignored while the vehicle is not in guided mode
    return guided_init(ignore_checks);
}

// avoid_set_destination - sets AVOID mode's target destination
// (distance from home, in cm)
bool Copter::avoid_adsb_set_destination(const Vector3f& destination)
{
    // check flight mode
    if (control_mode != AVOID_ADSB) {
        return false;
    }

    // re-use guided mode's position controller
    return guided_set_destination(destination);
}

bool Copter::avoid_adsb_set_velocity(const Vector3f& velocity_neu)
{
    // check flight mode
    if (control_mode != AVOID_ADSB) {
        return false;
    }

    // debug
    static uint32_t last_time_ms = 0;
    uint32_t now = AP_HAL::millis();
    if (now - last_time_ms > 500) {
        ::printf("Vel x:%4.2f y:%4.2f z:%4.2f\n",(double)velocity_neu.x,(double)velocity_neu.y,(double)velocity_neu.z);
        last_time_ms = now;
    }

    // re-use guided mode's velocity controller
    guided_set_velocity(velocity_neu);
    return true;
}

bool Copter::avoid_adsb_set_target_alt(int32_t target_alt_cm)
{
    //if (!wp_nav.set_wp_destination(destination, false)) {
    //    return false;
    //}

    // log target
    // Log_Write_GuidedTarget(guided_mode, destination, Vector3f());
    return true;
}

// runs the AVOID_ADSB controller
void Copter::avoid_adsb_run()
{
    // re-use guided mode's velocity controller
    // Note: this is safe from interference from GCSs and companion computer's whose guided mode
    //       position and velocity requests will be ignored while the vehicle is not in guided mode
    guided_run();
}
