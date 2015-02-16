/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#ifndef __AC_PRECLAND_IRLOCK_H__
#define __AC_PRECLAND_IRLOCK_H__

#include <AP_Common.h>
#include <AP_Math.h>
#include <AC_PrecLand_Backend.h>    // Precision Landing backend

/*
 * AC_PrecLand_IRLock - implements precision landing using target vectors provided
 *                         by a companion computer (i.e. Odroid) communicating via MAVLink
 */

class AC_PrecLand_IRLock : public AC_PrecLand_Backend
{
public:

    // Constructor
    AC_PrecLand_IRLock(const AC_PrecLand& frontend, AC_PrecLand::precland_state& state);

    // init - perform any required initialisation of backend controller
    void init();

    // get_target_rad - returns 2D body frame angles (in radians) to target
    //  x : body-frame roll direction, positive = target is to right (looking down)
    //  y : body-frame pitch direction, postiive = target is forward (looking down)
    Vector2f get_target_rad();

private:

    mavlink_channel_t   _chan;      // mavlink channel used to communicate with companion computer

};
#endif	// __AC_PRECLAND_IRLOCK_H__
