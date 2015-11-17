/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#ifndef __AC_PRECLAND_H__
#define __AC_PRECLAND_H__

#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>
#include <AC_PID/AC_PI_2D.h>
#include <AP_InertialNav/AP_InertialNav.h>

// definitions
#define AC_PRECLAND_SPEED_XY_DEFAULT            100.0f  // maximum horizontal speed
#define PRECLAND_P                              2.0f    // velocity controller P gain default
#define PRECLAND_I                              1.0f    // velocity controller I gain default
#define PRECLAND_IMAX                           1.0f    // velocity controller IMAX default
#define PRECLAND_UPDATE_TIME                    0.02f   // precland runs at 50hz
#define PRECLAND_SENSOR_TIMEOUT_MS              1000    // velocity slows to zero if sensor updates not received for a second
#define PRECLAND_DESVEL_FILTER_HZ               5.0f    // filter frequency of desired velocity filter

// declare backend classes
class AC_PrecLand_Backend;
class AC_PrecLand_Companion;
class AC_PrecLand_IRLock;

class AC_PrecLand
{
    // declare backends as friends
    friend class AC_PrecLand_Backend;
    friend class AC_PrecLand_Companion;
    friend class AC_PrecLand_IRLock;

public:

    // precision landing behaviours (held in PRECLAND_ENABLED parameter)
    enum PrecLandBehaviour {
        PRECLAND_BEHAVIOUR_DISABLED,
        PRECLAND_BEHAVIOR_ALWAYSLAND,
        PRECLAND_BEHAVIOR_CAUTIOUS
    };

    // types of precision landing (used for PRECLAND_TYPE parameter)
    enum PrecLandType {
        PRECLAND_TYPE_NONE = 0,
        PRECLAND_TYPE_COMPANION,
        PRECLAND_TYPE_IRLOCK
    };

    // Constructor
    AC_PrecLand(const AP_AHRS& ahrs, const AP_InertialNav& inav, AC_PI_2D& pi_precland_xy);

    // init - perform any required initialisation of landing controllers
    void init();

    // healthy - returns true if precision landing is healthy
    bool healthy() { return _backend_state.healthy; }

    // update - give chance to driver to get updates from sensor
    void update(float alt_above_terrain_cm);

    // initialise desired velocity
    void set_desired_velocity(const Vector3f &des_vel);

    // calculated desired 3D velocity towards target
    const Vector3f& calc_desired_velocity(float land_speed_cms);

    // returns last calculated desired velocity (for logging)
    const Vector3f& get_last_desired_velocity() { return _desired_vel; }

    // handle_msg - Process a LANDING_TARGET mavlink message
    void handle_msg(mavlink_message_t* msg);

    // accessors for logging
    bool enabled() const { return _enabled; }
    const Vector2f& last_bf_angle_to_target() const { return _angle_to_target; }
    const Vector3f& last_vec_to_target_ef() const { return _vec_to_target_ef; }

    // parameter var table
    static const struct AP_Param::GroupInfo var_info[];

private:

    // calc_angles - converts sensor's body-frame angles to earth-frame angles
    //  angles stored in _angle_to_target
    //  earth-frame angles stored in _ef_angle_to_target
    void calc_angles();

    // get_behaviour - returns enabled parameter as an behaviour
    enum PrecLandBehaviour get_behaviour() const { return (enum PrecLandBehaviour)(_enabled.get()); }

    // references to inertial nav and ahrs libraries
    const AP_AHRS&              _ahrs;
    const AP_InertialNav&       _inav;
    AC_PI_2D&                   _pi_precland_xy;    // horizontal velocity PI controller

    // parameters
    AP_Int8                     _enabled;           // enabled/disabled and behaviour
    AP_Int8                     _type;              // precision landing controller type
    AP_Float                    _speed_xy;          // maximum horizontal speed in cm/s

    // output from sensor (stored for logging)
    Vector2f                    _angle_to_target;   // last raw sensor angle to target
    Vector3f                    _vec_to_target_ef;  // last earth-frame angle to target
    uint32_t                    _capture_time_ms;   // system time in milliseconds of last sensor update

    bool                        _have_estimate : 1; // true if we have a recent estimated position offset
    bool                        _limit_xy      : 1; // true if controller has hit horizontal speed limit

    // output from controller
    Vector3f                    _desired_vel;       // desired velocity towards target in earth-frame
    LowPassFilterVector3f       _desired_vel_filter;    // desired velocity

    // backend state
    struct precland_state {
        bool    healthy;
    } _backend_state;
    AC_PrecLand_Backend         *_backend;  // pointers to backend precision landing driver
};
#endif	// __AC_PRECLAND_H__
