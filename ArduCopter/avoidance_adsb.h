#pragma once

#include <AP_Avoidance/AP_Avoidance.h>

// Provide Copter-specific implementation of avoidance.  While most of
// the logic for doing the actual avoidance is present in
// AP_Avoidance, this class allows Copter to override base
// functionality - for example, not doing anything while landed.
class AP_Avoidance_Copter : public AP_Avoidance {

public:

    AP_Avoidance_Copter(AP_AHRS &ahrs, class AP_ADSB &adsb) :
        AP_Avoidance(ahrs, adsb) { }

protected:

    // override avoidance handler
    MAV_COLLISION_ACTION handle_avoidance(const AP_Avoidance::Obstacle *obstacle, MAV_COLLISION_ACTION requested_action) override;

    // override recovery handler
    void handle_recovery(uint8_t recovery_action) override;

    // perpendicular avoidance handler
    bool handle_avoidance_perpendicular(const AP_Avoidance::Obstacle *obstacle, bool allow_mode_change);

    // handle tcas based avoidance
    bool handle_avoidance_tcas(const AP_Avoidance::Obstacle *obstacle, bool allow_mode_change);

    // tcas avoidance handling
    // different types of TCAS resolution
    typedef enum {
        tcas_resolution_descend = 2,
        tcas_resolution_ascend = 3,
    } tcas_resolution_t ;

    // get best target destination using tcas avoidance
    bool tcas_get_target_alt(const AP_Avoidance::Obstacle *obstacle, int32_t &target_alt);

    // returns the action we should take based on the TCAS algorithm
    tcas_resolution_t tcas_get_resolution(const AP_Avoidance::Obstacle *obstacle);

    // returns an identifier for this aircraft corresponding to
    // supplied SRC.  For example, for if the source is mavlink then
    // the identifier would be this aircraft's mavlink src id
    uint32_t my_src_id(const MAV_COLLISION_SRC src) const;

    // send new destination to avoid_adsb mode's controller (and throttle updates if necessary)
    void set_avoid_adsb_destination(const Vector3f &dest);

    // lowest height avoidance will send the aircraft, in metres
    static const uint8_t _minimum_avoid_height = 10;
    uint32_t _last_wp_update = 0;
    control_mode_t prev_control_mode = RTL;
};
