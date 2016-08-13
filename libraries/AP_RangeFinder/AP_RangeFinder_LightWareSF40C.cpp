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
#include <stdio.h>

// debug
#include <GCS_MAVLink/GCS.h>

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

// get distance in cm in a particular direction in degrees (0 is forward, clockwise)
bool AP_RangeFinder_LightWareSF40C::get_horizontal_distance(int16_t angle_deg, int16_t &distance_cm)
{
    uint8_t sector;
    if (convert_angle_to_sector(angle_deg, sector)) {
        if (_distance_valid[sector]) {
            distance_cm = _distance_cm[sector];
            return true;
        }
    }
    return false;
}

/*
   update the state of the sensor
*/
void AP_RangeFinder_LightWareSF40C::update(void)
{
    if (uart == nullptr) {
        return;
    }

    // initialise sensor if necessary
    bool initialised = initialise();

    // process incoming messages
    if (check_for_reply()) {
        // set global distance to shortest detected distance mostly for reporting purposes
        int16_t shortest_cm = 0;
        bool shortest_set = false;
        for (uint8_t i = 0; i<RANGEFINDER_SF40C_SECTORS; i++) {
            if (_distance_valid[i] && (!shortest_set || (_distance_cm[i] < shortest_cm))) {
                shortest_cm = _distance_cm[i];
                shortest_set = true;
            }
        }
        if (shortest_set) {
            state.distance_cm = shortest_cm;
        }
        update_status();
    }

    // request new data from sensor
    if (initialised) {
        request_new_data();
    }

    // check for timeout
    if (AP_HAL::millis() - _last_distance_received_ms > RANGEFINDER_SF40C_TIMEOUT_MS) {
        set_status(RangeFinder::RangeFinder_NoData);
    }
}

// initialise sensor (returns true if sensor is succesfully initialised)
bool AP_RangeFinder_LightWareSF40C::initialise()
{
    // request motors turn on for first 30 seconds then give up
    if (_motor_speed == 0) {
        // request motors spin up once per second
        if ((_last_request_ms == 0) || AP_HAL::millis() - _last_request_ms > 1000) {
            set_motor_speed(true);
        }
        return false;
    }
    return true;
}

// set speed of rotating motor
void AP_RangeFinder_LightWareSF40C::set_motor_speed(bool on_off)
{
    // exit immediately if no uart
    if (uart == nullptr) {
        return;
    }

    if (on_off) {
        uart->write("#MBS,3\r\n");  // send request to spin motor at 4.5hz
        GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_CRITICAL,"Set Mot Speed");
    } else {
        uart->write("#MBS,0\r\n");  // send request to stop motor
    }

    // re-request update motor speed
    uart->write("?MBS\r\n");
    _last_request_type = RequestType_MotorSpeed;
    _last_request_ms = AP_HAL::millis();
}

// request new data if required
void AP_RangeFinder_LightWareSF40C::request_new_data()
{
    if (uart == nullptr) {
        return;
    }

    // after timeout assume no reply will ever come
    uint32_t now = AP_HAL::millis();
    if ((_last_request_type != RequestType_None) && ((now - _last_request_ms) > RANGEFINDER_SF40C_TIMEOUT_MS)) {
        _last_request_type = RequestType_None;
        _last_request_ms = 0;
        //GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_CRITICAL,"TO-ClearRequest");
    }

    // if we are not waiting for a reply, ask for something
    if (_last_request_type == RequestType_None) {
        _request_count++;
        if (_request_count >= 5) {
            send_request_for_health();
            _request_count = 0;
        } else {
            // request new distance measurement
            send_request_for_distance();
        }
        _last_request_ms = now;
    }
}

// send request for sensor health
void AP_RangeFinder_LightWareSF40C::send_request_for_health()
{
    if (uart == nullptr) {
        return;
    }

    uart->write("?GS\r\n");
    _last_request_type = RequestType_Health;
    _last_request_ms = AP_HAL::millis();

    // debug
    //GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_CRITICAL,"GetH");
}

bool AP_RangeFinder_LightWareSF40C::send_request_for_distance()
{
    if (uart == nullptr) {
        return false;
    }

    // increment sector
    _last_sector++;
    if (_last_sector >= RANGEFINDER_SF40C_SECTORS) {
        _last_sector = 0;
    }

    // send request
    switch (_last_sector) {
        case 0:
            uart->write("?TS,45,0\r\n");    // forward
            break;
        case 1:
            uart->write("?TS,45,45\r\n");   // forward right
            break;
        case 2:
            uart->write("?TS,45,90\r\n");   // right
            break;
        case 3:
            uart->write("?TS,45,135\r\n");  // back right
            break;
        case 4:
            uart->write("?TS,45,180\r\n");  // back
            break;
        case 5:
            uart->write("?TS,45,225\r\n");  // back left
            break;
        case 6:
            uart->write("?TS,45,270\r\n");  // left
            break;
        case 7:
            uart->write("?TS,45,315\r\n");  // forward left
            break;
        default:
            // invalid sector do nothing
            break;
    }

    _last_request_type = RequestType_DistanceMeasurement;
    _last_request_ms = AP_HAL::millis();

    // debug
    //GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_CRITICAL,"GetD");

    return true;
}

// check for replies from sensor
bool AP_RangeFinder_LightWareSF40C::check_for_reply()
{
    if (uart == nullptr) {
        return false;
    }

    // debug
    char print_buf[25];
    char print_processed_buf[25];
    uint8_t counter = 0;
    memset(print_buf, 0, sizeof(print_buf));
    memset(print_processed_buf, 0, sizeof(print_processed_buf));

    // read any available lines from the lidar
    // if see CR (i.e. \r), LF (\n) or Space it means we have consumed a line
    //    if zero length throw it away
    //    comma separated first_element, optional 2nd element
    //    stage = first element or second element
    //    send to processor based on what it's looking for
    // To-Do: handle timeout
    uint16_t count = 0;
    int16_t nbytes = uart->available();
    while (nbytes-- > 0) {
        char c = uart->read();
        // check for end of packet
        if (c == '\r' || c == '\n') {
            if ((element_len[0] > 0)) {
                if (process_reply()) {
                    count++;
                }
                // debug
                print_processed_buf[counter] = 'D';
            } else {
                // debug
                print_processed_buf[counter] = 'X';
            }
            // clear buffers after processing
            clear_buffers();
            ignore_reply = false;
            wait_for_space = false;

        // if message starts with # ignore it
        } else if (c == '#' || ignore_reply) {
            ignore_reply = true;
            // debug
            print_processed_buf[counter] = 'I';

        // if waiting for <space>
        } else if (c == '?') {
            wait_for_space = true;
            // debug
            print_processed_buf[counter] = 'w';

        } else if (wait_for_space) {
            if (c == ' ') {
                wait_for_space = false;
                // debug
                print_processed_buf[counter] = 'h';
            } else {
                // debug
                print_processed_buf[counter] = '-';
            }

        // if comma, move onto filling in 2nd element
        } else if (c == ',') {
            if ((element_num == 0) && (element_len[0] > 0)) {
                element_num++;
                print_processed_buf[counter] = ',';
            } else {
                // don't support 3rd element so clear buffers
                clear_buffers();
                ignore_reply = true;
                print_processed_buf[counter] = 'Z';
            }

        // if part of a number, add to element buffer
        } else if (isdigit(c) || c == '.') {
            element_buf[element_num][element_len[element_num]] = c;
            element_len[element_num]++;
            print_processed_buf[counter] = c;
            if (element_len[element_num] >= sizeof(element_buf[element_num])-1) {
                // too long, discard the line
                clear_buffers();
                ignore_reply = true;
                print_processed_buf[counter] = 'L';
            }
        } else {
            print_processed_buf[counter] = 'i';
        }

        // debug
        if (counter < sizeof(print_buf)-1) {
            print_buf[counter] = c;
            if (c == '\r') {
                print_buf[counter] = 'R';
            }
            if (c == '\n') {
                print_buf[counter] = 'N';
            }
            if (c == ' ') {
                print_buf[counter] = '*';
            }
            counter++;
        }
        ////////
    }

    if (counter > 0) {
        GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_CRITICAL,"Buf:%s",print_buf);
        GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_CRITICAL,"PrB:%s",print_processed_buf);
    }

    return (count > 0);
}

// process reply
bool AP_RangeFinder_LightWareSF40C::process_reply()
{
    if (uart == nullptr) {
        return false;
    }

    bool success = false;

    // debug
    if (element_len[0] > 0) {
        GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_CRITICAL,"e1:%d/%s",(int)element_len[0],element_buf[0]);
    }
    if (element_len[1] > 0) {
        GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_CRITICAL,"e2:%d/%s",(int)element_len[1],element_buf[1]);
    }

    switch (_last_request_type) {
        case RequestType_None:
            break;

        case RequestType_Health:
            // expect result in the form "0xhhhh"
            if (element_len[0] > 0) {
                int result;
                if (sscanf(element_buf[0], "%x", &result) > 0) {
                    _sensor_status.value = result;
                    success = true;
                }
            }
            GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_CRITICAL,"Health OK");
            break;

        case RequestType_MotorSpeed:
            _motor_speed = atoi(element_buf[0]);
            GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_CRITICAL,"Mot Speed OK");
            break;

        case RequestType_DistanceMeasurement:
        {
            float angle_deg = (float)atof(element_buf[0]);
            float distance_m = (float)atof(element_buf[1]);
            uint8_t sector;
            if (convert_angle_to_sector(angle_deg, sector)) {
                _distance_cm[sector] = distance_m * 100;
                _distance_valid[sector] = true;
                _last_distance_received_ms = AP_HAL::millis();
            }
            GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_CRITICAL,"Dist OK %4.2f/%4.2f",(double)angle_deg,(double)distance_m);
            GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_CRITICAL,"DV:%d/%d/%d/%d/%d/%d/%d/%d",
                    (int)_distance_valid[0],
                    (int)_distance_valid[1],
                    (int)_distance_valid[2],
                    (int)_distance_valid[3],
                    (int)_distance_valid[4],
                    (int)_distance_valid[5],
                    (int)_distance_valid[6],
                    (int)_distance_valid[7]);
            break;
        }

        default:
            break;
    }

    return success;
}

// clear buffers ahead of processing next message
void AP_RangeFinder_LightWareSF40C::clear_buffers()
{
    element_len[0] = 0;
    element_len[1] = 0;
    element_num = 0;
    memset(element_buf, 0, sizeof(element_buf));
}

bool AP_RangeFinder_LightWareSF40C::convert_angle_to_sector(float angle_degrees, uint8_t &sector)
{
    // sanity check angle
    if (angle_degrees > 360.0f || angle_degrees < 0.0f) {
        return false;
    }

    // increase angle by 1/2 sector size
    angle_degrees += (RANGEFINDER_SF40C_SECTOR_WIDTH_DEG / 2.0f);
    if (angle_degrees > 360.0f) {
        angle_degrees -= 360.0f;
    }

    // convert and range check sector
    uint16_t sec = (angle_degrees / RANGEFINDER_SF40C_SECTOR_WIDTH_DEG);
    if (sec < RANGEFINDER_SF40C_SECTORS) {
        sector = sec;
        return true;
    }

    return false;
}
