/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#include <AP_HAL.h>
#include <AC_PrecLand_Companion.h>

extern const AP_HAL::HAL& hal;

// Constructor
AC_PrecLand_Companion::AC_PrecLand_Companion(const AC_PrecLand& frontend, AC_PrecLand::precland_state& state)
: AC_PrecLand_Backend(frontend, state)
{
}

// init - perform initialisation of this backend
void AC_PrecLand_Companion::init()
{
    // set healthy
    _state.healthy = true;
}

// get_target_rad - returns 2D body frame angles (in radians) to target
//  x : body-frame roll direction, positive = target is to right (looking down)
//  y : body-frame pitch direction, postiive = target is forward (looking down)
Vector2f AC_PrecLand_Companion::get_target_rad()
{
    return Vector2f(0,0);
}
