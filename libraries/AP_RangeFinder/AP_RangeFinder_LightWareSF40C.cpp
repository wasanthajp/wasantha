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

#include <AP_HAL/AP_HAL.h>
#include "AP_RangeFinder_LightWareSF40C.h"
#include <AP_SerialManager/AP_SerialManager.h>
#include <ctype.h>

extern const AP_HAL::HAL& hal;

/* 
   The constructor also initialises the rangefinder. Note that this
   constructor is not called until detect() returns true, so we
   already know that we should setup the rangefinder
*/
AP_RangeFinder_LightWareSF40C::AP_RangeFinder_LightWareSF40C(RangeFinder &_ranger, uint8_t instance,
                                                               RangeFinder::RangeFinder_State &_state,
                                                               AP_SerialManager &serial_manager) :
    AP_RangeFinder_Backend(_ranger, instance, _state)
{
    uart = serial_manager.find_serial(AP_SerialManager::SerialProtocol_Lidar, 0);
    if (uart != nullptr) {
        uart->begin(serial_manager.find_baudrate(AP_SerialManager::SerialProtocol_Lidar, 0));
    }
}

/* 
   detect if a Lightware rangefinder is connected. We'll detect by
   trying to take a reading on Serial. If we get a result the sensor is
   there.
*/
bool AP_RangeFinder_LightWareSF40C::detect(RangeFinder &_ranger, uint8_t instance, AP_SerialManager &serial_manager)
{
    return serial_manager.find_serial(AP_SerialManager::SerialProtocol_Lidar, 0) != nullptr;
}

/*
   update the state of the sensor
*/
void AP_RangeFinder_LightWareSF40C::update(void)
{
    if (check_for_reply()) {
        // set global distance to shortest detected distance mostly for reporting purposes
        int16_t shortest_cm = 0;
        bool shortest_set = false;
        for (uint8_t i = 0; i<RANGEFINDER_SF40C_QUADRANTS; i++) {
            if (_distance_valid[i] && (!shortest_set || (_distance_cm[i] < shortest_cm))) {
                shortest_cm = _distance_cm[i];
                shortest_set = true;
            }
        }
        if (shortest_set) {
            state.distance_cm = shortest_cm;
        }
        update_status();
    } else if (AP_HAL::millis() - _last_distance_received_ms > 200) {
        set_status(RangeFinder::RangeFinder_NoData);
    }
}

// get distance in cm in a particular direction in degrees (0 is forward, clockwise)
bool AP_RangeFinder_LightWareSF40C::get_horizontal_distance(int16_t angle_deg, int16_t &distance_cm)
{
    return false;
}

// set speed of rotating motor
bool AP_RangeFinder_LightWareSF40C::set_motor_speed(bool on_off)
{
    return send_request(RequestType_MotorSpeed);
}

// send request for something from sensor
bool AP_RangeFinder_LightWareSF40C::send_request(RequestType req_type)
{
    // exit immediately if we are already waiting on another request
    if (_last_request_type != RequestType_None) {
        return false;
    }

    bool success = true;
    switch (req_type) {
        case RequestType_Health:
            break;
        case RequestType_MotorSpeed:
            uart->write("?MBS,3\r\n");  // send request to spin motor at 4.5hz
            uart->write("?MBS\r\n");    // request update on motor speed
            break;
        case RequestType_None:
        case RequestType_DistanceMeasurement:
        default:
            // these messages not supported by this method - do nothing
            success = false;
            break;
    }

    if (success) {
        _last_request_type = req_type;
        _last_request_ms = AP_HAL::millis();
    }

    return success;
}

bool AP_RangeFinder_LightWareSF40C::send_request_for_distance(uint8_t sector)
{
    bool success = true;

    switch (sector) {
        case 0: // forward
            uart->write("?TS,45,0\r\n");
            break;
        case 1: // forward right
            uart->write("?TS,45,45\r\n");
            break;
        case 2: // right
            uart->write("?TS,45,90\r\n");
            break;
        case 3: // back right
            uart->write("?TS,45,135\r\n");
            break;
        case 4: // back
            uart->write("?TS,45,180\r\n");
            break;
        case 5: // back left
            uart->write("?TS,45,225\r\n");
            break;
        case 6: // left
            uart->write("?TS,45,270\r\n");
            break;
        case 7: // forward left
            uart->write("?TS,45,315\r\n");
            break;
        default:
            // invalid sector, do nothing
            success = false;
            break;
    }

    if (success) {
        _last_request_type = RequestType_DistanceMeasurement;
        _last_request_ms = AP_HAL::millis();
        _last_sector = sector;
    }
    return success;
}

// check for replies from sensor
bool AP_RangeFinder_LightWareSF40C::check_for_reply()
{
    if (uart == nullptr || _last_request_type == RequestType_None) {
        return false;
    }

    // read any available lines from the lidar
    float sum = 0;
    uint16_t count = 0;
    int16_t nbytes = uart->available();
    while (nbytes-- > 0) {
        char c = uart->read();
        if (c == '\r') {
            linebuf[linebuf_len] = 0;
            sum += (float)atof(linebuf);
            count++;
            linebuf_len = 0;
        } else if (isdigit(c) || c == '.') {
            linebuf[linebuf_len++] = c;
            if (linebuf_len == sizeof(linebuf)) {
                // too long, discard the line
                linebuf_len = 0;
            }
        }
    }

    if (count == 0) {
        return false;
    }
    //reading_cm = 100 * sum / count;

    // update time last distance measurement was successful received
    _last_distance_received_ms = AP_HAL::millis();

    // request new distance measurement
    uint8_t next_sector = _last_sector++;
    if (next_sector >= RANGEFINDER_SF40C_QUADRANTS) {
        next_sector = 0;
    }
    send_request_for_distance(next_sector);

    return true;
}
