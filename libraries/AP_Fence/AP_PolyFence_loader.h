#pragma once

#include <stdint.h>

#include <AP_Math/AP_Math.h>
#include <GCS_MAVLink/GCS.h>

class AP_PolyFence
{

public:

    AP_PolyFence();

    // enable or disable fence
    bool set_enabled(bool enable);

    // check if fence is enabled
    bool enabled() const;

    // check for breaches
    void check();

    // returns true if last check found vehicle has breached the fence
    bool breached();

    // parameter table
    static const struct AP_Param::GroupInfo        var_info[];

protected:

    // maximum number of fence points we can store
    uint8_t max_points();

    // allocate memory and load fence points from eeprom
    void load();

    // load or save individual points to or from eeprom
    Vector2l load_point_from_eeprom(uint16_t i);
    void save_point_to_eeprom(uint16_t i, Vector2l &point);

    // parameters
    AP_Int8 _total;

    /*
     *  The state of geo-fencing. This structure is dynamically allocated
     *  the first time it is used. This means we only pay for the pointer
     *  and not the structure on systems where geo-fencing is not being
     *  used.
     *
     *  We store a copy of the boundary in memory as we need to access it
     *  very quickly at runtime
     */
    struct GeofenceState {
        bool breached_polygon;          // true if vehicle is currently outside the polygon fence
        uint16_t breach_count;
        uint8_t breach_type;
        uint32_t breach_time;

        // boundary state
        uint8_t boundary_num_points;    // number of boundary points in boundary array
        bool boundary_uptodate;         // boundary array is completely loaded with all points from eeprom
        bool boundary_valid;            // boundary is a complete polygon
        Vector2l *boundary;             // array of boundary points.  Note: point 0 is the return point
    } *geofence_state;

    bool _enabled = false;

    virtual void gcs_send_text(MAV_SEVERITY severity, const char *str) = 0;
    virtual void gcs_send_message(enum ap_message id) = 0;

};

