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

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include "AP_Proximity.h"
#include "AP_Proximity_Backend.h"

/*
  base class constructor. 
  This incorporates initialisation as well.
*/
AP_Proximity_Backend::AP_Proximity_Backend(AP_Proximity &_frontend, AP_Proximity::Proximity_State &_state) :
        frontend(_frontend),
        state(_state)
{
}

// set status and update valid count
void AP_Proximity_Backend::set_status(AP_Proximity::Proximity_Status status)
{
    state.status = status;
}

// get ignore angle info
uint8_t AP_Proximity_Backend::get_ignore_angle_count() const
{
    // count number of ignore sectors
    uint8_t count = 0;
    for (uint8_t i=0; i < PROXIMITY_MAX_IGNORE; i++) {
        if (frontend._ignore_width_deg[i] != 0) {
            count++;
        }
    }
    return count;
}

// get next ignore angle
bool AP_Proximity_Backend::get_ignore_area(uint8_t index, uint16_t &angle_deg, uint8_t &width_deg) const
{
    if (index >= PROXIMITY_MAX_IGNORE) {
        return false;
    }
    angle_deg = frontend._ignore_angle_deg[index];
    width_deg = frontend._ignore_width_deg[index];
    return true;
}

// retrieve start or end angle of next ignore area (i.e. closest ignore area higher than the start_angle)
// start_or_end = 0 to get start, 1 to retrieve end
bool AP_Proximity_Backend::get_next_ignore_start_or_end(uint8_t start_or_end, int16_t start_angle, int16_t &ignore_start) const
{
    bool found = false;
    int16_t smallest_angle_diff = 0;
    int16_t smallest_angle_start = 0;

    for (uint8_t i=0; i < PROXIMITY_MAX_IGNORE; i++) {
        if (frontend._ignore_width_deg[i] != 0) {
            int16_t offset = start_or_end == 0 ? -frontend._ignore_width_deg[i] : +frontend._ignore_width_deg[i];
            int16_t ignore_start_angle = wrap_360(frontend._ignore_angle_deg[i] + offset/2.0f);
            int16_t ang_diff = wrap_360(ignore_start_angle - start_angle);
            if (!found || ang_diff < smallest_angle_diff) {
                smallest_angle_diff = ang_diff;
                smallest_angle_start = ignore_start_angle;
                found = true;
            }
        }
    }

    if (found) {
        ignore_start = smallest_angle_start;
    }
    return found;
}

