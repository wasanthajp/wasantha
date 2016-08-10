// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#pragma once

#include "RangeFinder.h"
#include "RangeFinder_Backend.h"

#define RANGEFINDER_SF40C_QUADRANTS             8                                   // number of quadrants
#define RANGEFINDER_SF40C_QUADRANT_WIDTH_DEG    (360/RANGEFINDER_SF40C_QUADRANTS)   // angular width of each quadrant
#define RANGEFINDER_SF40C_TIMEOUT_MS            200

class AP_RangeFinder_LightWareSF40C : public AP_RangeFinder_Backend
{

public:
    // constructor
    AP_RangeFinder_LightWareSF40C(RangeFinder &ranger, uint8_t instance, RangeFinder::RangeFinder_State &_state,
                                   AP_SerialManager &serial_manager);

    // static detection function
    static bool detect(RangeFinder &ranger, uint8_t instance, AP_SerialManager &serial_manager);

    // update state
    void update(void);

    // get distance in cm in a particular direction in degrees (0 is forward, clockwise)
    // returns true on successful read and places distance in distance_cm
    bool get_horizontal_distance(int16_t angle_deg, int16_t &distance_cm);

private:

    enum RequestType {
        RequestType_None = 0,
        RequestType_Health,
        RequestType_MotorSpeed,
        RequestType_DistanceMeasurement
    };

    // set speed of rotating motor
    bool set_motor_speed(bool on_off);

    // send request for something from sensor
    bool send_request(RequestType req_type);
    bool send_request_for_distance(uint8_t sector);

    // check for replies from sensor
    bool check_for_reply();

    // reply related variables
    AP_HAL::UARTDriver *uart = nullptr;
    char linebuf[10];
    uint8_t linebuf_len;

    // request related variables
    enum RequestType _last_request_type;    // last request made to sensor
    uint8_t  _last_sector;                  // last sector requested
    uint32_t _last_request_angle;           // angle of last request for distance measurement
    uint32_t _last_request_ms;              // system time of last request
    uint32_t _last_distance_received_ms;    // system time of last distance measurement received from sensor

    // recorded distances
    int16_t _distance_cm[RANGEFINDER_SF40C_QUADRANTS];
    int16_t _distance_valid[RANGEFINDER_SF40C_QUADRANTS];
};
