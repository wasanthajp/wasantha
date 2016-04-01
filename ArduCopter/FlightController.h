#pragma once

// this class is #included into the Copter:: namespace

class FlightController {
    friend class Copter;

public:

    FlightController(Copter &copter) :
        _copter(copter),
        g(copter.g),
        wp_nav(_copter.wp_nav),
        pos_control(_copter.pos_control),
        inertial_nav(_copter.inertial_nav),
        ahrs(_copter.ahrs),
        attitude_control(_copter.attitude_control),
        motors(_copter.motors),
        channel_roll(_copter.channel_roll),
        channel_pitch(_copter.channel_pitch),
        channel_throttle(_copter.channel_throttle),
        channel_yaw(_copter.channel_yaw),
        ap(_copter.ap)
        { };

protected:

    virtual bool init(bool ignore_checks) = 0; // should be called at 100hz or more
    virtual void run() = 0; // should be called at 100hz or more

    virtual bool is_autopilot() const { return false; }
    virtual bool requires_GPS() const = 0;
    virtual bool has_manual_throttle() const = 0;
    virtual bool allows_arming(bool from_gcs) const = 0;
    void print_FlightMode(AP_HAL::BetterStream *port) const {
        port->print(name());
    }
    virtual const char *name() const = 0;

    Copter &_copter;

    // convenience references to avoid code churn in conversion:
    Parameters &g;
    AC_WPNav &wp_nav;
    AC_PosControl &pos_control;
    AP_InertialNav &inertial_nav;
    AP_AHRS &ahrs;
    AC_AttitudeControl_t &attitude_control;
    MOTOR_CLASS &motors;
    RC_Channel *&channel_roll;
    RC_Channel *&channel_pitch;
    RC_Channel *&channel_throttle;
    RC_Channel *&channel_yaw;
    ap_t &ap;

    // pass-through functions to reduce code churn on conversion;
    // these are candidates for moving into the FlightController base
    // class.
    virtual float get_throttle_pre_takeoff(float input_thr) {
        return _copter.get_throttle_pre_takeoff(input_thr);
    }
    virtual void get_pilot_desired_lean_angles(float roll_in, float pitch_in, float &roll_out, float &pitch_out, float angle_max) {
        _copter.get_pilot_desired_lean_angles(roll_in, pitch_in, roll_out, pitch_out, angle_max);
    }
    virtual float get_surface_tracking_climb_rate(int16_t target_rate, float current_alt_target, float dt) {
        return _copter.get_surface_tracking_climb_rate(target_rate, current_alt_target, dt);
    }
    virtual float get_pilot_desired_yaw_rate(int16_t stick_angle) {
        return _copter.get_pilot_desired_yaw_rate(stick_angle);
    }
    virtual float get_pilot_desired_climb_rate(float throttle_control) {
        return _copter.get_pilot_desired_climb_rate(throttle_control);
    }
    virtual float get_pilot_desired_throttle(int16_t throttle_control) {
        return _copter.get_pilot_desired_throttle(throttle_control);
    }
    virtual void update_simple_mode(void) {
        _copter.update_simple_mode();
    }
    virtual float get_smoothing_gain() {
        return _copter.get_smoothing_gain();
    }
    // end pass-through functions
};
