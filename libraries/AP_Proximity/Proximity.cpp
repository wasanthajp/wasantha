// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
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

#include "Proximity.h"
#include "AP_Proximity_LightWareSF40C.h"

extern const AP_HAL::HAL &hal;

// table of user settable parameters
const AP_Param::GroupInfo Proximity::var_info[] = {
    // 1 is reserved for possible addition of an ENABLED parameter

    // @Param: _TYPE
    // @DisplayName: Proximity type
    // @Description: What type of proximity sensor is connected
    // @Values: 0:None,1:LightWareSF40C
    // @User: Standard
    AP_GROUPINFO("_TYPE",    2, Proximity, _type[0], 0),

#if PROXIMITY_MAX_INSTANCES > 1
    // @Param: 2_TYPE
    // @DisplayName: Second Proximity type
    // @Description: What type of proximity sensor is connected
    // @Values: 0:None,1:LightWareSF40C
    // @User: Advanced
    AP_GROUPINFO("2_TYPE",    12, Proximity, _type[1], 0),
#endif

    AP_GROUPEND
};

Proximity::Proximity(AP_SerialManager &_serial_manager) :
    primary_instance(0),
    num_instances(0),
    serial_manager(_serial_manager)
{
    AP_Param::setup_object_defaults(this, var_info);

    // init state and drivers
    memset(state,0,sizeof(state));
    memset(drivers,0,sizeof(drivers));
}

// initialise the Proximity class. We do detection of attached sensors here
// we don't allow for hot-plugging of sensors (i.e. reboot required)
void Proximity::init(void)
{
    if (num_instances != 0) {
        // init called a 2nd time?
        return;
    }
    for (uint8_t i=0; i<PROXIMITY_MAX_INSTANCES; i++) {
        detect_instance(i);
        if (drivers[i] != NULL) {
            // we loaded a driver for this instance, so it must be
            // present (although it may not be healthy)
            num_instances = i+1;
        }

        // initialise status
        state[i].status = Proximity_NotConnected;
    }
}

// update Proximity state for all instances. This should be called at a high rate by the main loop
void Proximity::update(void)
{
    for (uint8_t i=0; i<num_instances; i++) {
        if (drivers[i] != NULL) {
            if (_type[i] == Proximity_Type_None) {
                // allow user to disable a proximity sensor at runtime
                state[i].status = Proximity_NotConnected;
                continue;
            }
            drivers[i]->update();
        }
    }

    // work out primary instance - first sensor returning good data
    for (int8_t i=num_instances-1; i>=0; i--) {
        if (drivers[i] != NULL && (state[i].status == Proximity_Good)) {
            primary_instance = i;
        }
    }
}
    
//  detect if an instance of a proximity sensor is connected.
void Proximity::detect_instance(uint8_t instance)
{
    uint8_t type = _type[instance];
    if (type == Proximity_Type_SF40C) {
        if (AP_Proximity_LightWareSF40C::detect(*this, instance, serial_manager)) {
            state[instance].instance = instance;
            drivers[instance] = new AP_Proximity_LightWareSF40C(*this, instance, state[instance], serial_manager);
            return;
        }
    }
}

// get distance in cm in a particular direction in degrees (0 is forward, clockwise)
// returns true on successful read and places distance in distance_cm
bool Proximity::get_horizontal_distance(uint8_t instance, int16_t angle_deg, int16_t &distance_cm)
{
    if ((drivers[instance] == NULL) || (_type[instance] == Proximity_Type_None)) {
        // allow user to disable a proximity sensor at runtime
        state[instance].status = Proximity_NotConnected;
        return false;
    }
    // get distance from backend
    return drivers[instance]->get_horizontal_distance(angle_deg, distance_cm);
}
