#include "AP_PolyFence.h"

extern const AP_HAL::HAL& hal;

#define MIN_GEOFENCE_POINTS 5 // 3 to define a minimal polygon (triangle)
                              // + 1 for return point and +1 for last
                              // pt (same as first)

const AP_Param::GroupInfo AP_PolyFence::var_info[] = {
    // @Param: _TOTAL
    // @DisplayName: Fence total number of points
    // @Description: Number of geofence points currently loaded
    // @User: Advanced
    AP_GROUPINFO("_TOTAL", 1, AP_PolyFence, g.fence_total, 0),

    AP_GROUPEND
};

static const StorageAccess fence_storage(StorageManager::StorageFence);

AP_PolyFence::AP_PolyFence() :
{
    // initialise parameter defaults
    AP_Param::setup_object_defaults(this, var_info);
}

// enable or disable fence
bool AP_PolyFence::set_enabled(bool enable)
{
    _enabled = enable;
    return enabled();
}

// check if fence is enabled
bool AP_PolyFence::enabled() const
{
    // check if state has been created
    if (geofence_state == NULL) {
        return false;
    }

    // check all points have been loaded from eeprom and validated
    if (!geofence_state->boundary_uptodate || !geofence_state->boundary_valid) {
        return false;
    }

    return _enabled;
}

/*
 *  fence boundaries fetch/store
 */
Vector2l AP_PolyFence::load_point_from_eeprom(uint16_t i)
{
    Vector2l ret;

    // return empty Vector if point is invalid
    if (i > _total || i >= max_points()) {
        return Vector2l(0,0);
    }

    // read fence point
    ret.x = fence_storage.read_uint32(i * sizeof(Vector2l));
    ret.y = fence_storage.read_uint32(i * sizeof(Vector2l) + 4);

    return ret;
}

// save a fence point to eeprom
void AP_PolyFence::save_point_to_eeprom(uint16_t i, Vector2l &point)
{
    if (i >= _total || i >= max_points()) {
        // not allowed
        return;
    }

    // write point to eeprom
    fence_storage.write_uint32(i * sizeof(Vector2l), point.x);
    fence_storage.write_uint32(i * sizeof(Vector2l)+4, point.y);

    // flag that we need to check boundary is complete
    if (geofence_state != NULL) {
        geofence_state->boundary_uptodate = false;
    }
}

/*
 *  allocate and fill the geofence state structure
 */
void AP_PolyFence::load()
{
    // return immediately if geofence is setup correctly
    if (geofence_state != NULL && geofence_state->boundary_uptodate) {
        return;
    }

    // allocate memory if required
    if (geofence_state == NULL) {
        uint16_t boundary_size = sizeof(Vector2l) * max_points();
        if (hal.util->available_memory() < 100 + boundary_size + sizeof(struct GeofenceState)) {
            // too risky to enable as we could run out of stack
            goto failed;
        }

        // allocate state
        geofence_state = (struct GeofenceState *)calloc(1, sizeof(struct GeofenceState));
        if (geofence_state == NULL) {
            // not much we can do here except disable it
            goto failed;
        }

        // allocate boundary points
        geofence_state->boundary = (Vector2l *)calloc(1, boundary_size);
        if (geofence_state->boundary == NULL) {
            free(geofence_state);
            geofence_state = NULL;
            goto failed;
        }

        // initialise state
        geofence_state->breached_polygon = false;
        geofence_state->breach_count = 0;
        geofence_state->breach_type = FENCE_BREACH_NONE;
        geofence_state->breach_time = 0;
        geofence_state->boundary_num_points = 0;
        geofence_state->boundary_uptodate = false;
        geofence_state->boundary_valid = false;
    }

    // exit immediately if no points in eeprom
    if (_total <= 0) {
        _total.set(0);
        return;
    }

    // load points from eeprom into boundary array
    if (!geofence_state->boundary_uptodate) {
        for (uint16_t i=0; i<_total; i++) {
            geofence_state->boundary[i] = load_point_from_eeprom(i);
        }
        geofence_state->boundary_num_points = i;
        geofence_state->boundary_uptodate = true;
        geofence_state->boundary_valid = false;

        // point 1 and last point must be the same.  Note: 0th point is reserved as the return point
        if (!Polygon_complete(&geofence_state->boundary[1], geofence_state->boundary_num_points-1)) {
            goto failed;
        }

        // check return point is within the fence
        if (Polygon_outside(geofence_state->boundary[0], &geofence_state->boundary[1], geofence_state->boundary_num_points-1)) {
            goto failed;
        }

        // mark boundary as valid
        geofence_state->boundary_valid = false;
    }

    geofence_state->fence_triggered = false;

    gcs_send_text(MAV_SEVERITY_INFO,"Geofence loaded");
    gcs_send_message(MSG_FENCE_STATUS);
    return;

failed:
    gcs_send_text(MAV_SEVERITY_WARNING,"Geofence setup error");
}

/*
 * return true if a geo-fence has been uploaded and
 * FENCE_ACTION is 1 (not necessarily enabled)
 */
bool AP_PolyFence::present() const
{
    //require at least a return point and a triangle
    //to define a geofence area:
    if (g.fence_action == FENCE_ACTION_NONE || g.fence_total < MIN_GEOFENCE_POINTS) {
        return false;
    }
    return true;
}

/*
 *
 */
void AP_PolyFence::send_status(mavlink_channel_t chan)
{
    if (geofence_enabled() && geofence_state != NULL) {
        mavlink_msg_fence_status_send(chan,
                                      (int8_t)geofence_state->fence_triggered,
                                      geofence_state->breach_count,
                                      geofence_state->breach_type,
                                      geofence_state->breach_time);
    }
}

/*
  return true if geofence has been breached
 */
bool AP_PolyFence::breached()
{
    return geofence_state ? geofence_state->fence_triggered : false;
}


/*
 *  check if we have breached the polygon fence
 */
void AP_PolyFence::check(const Location& loc)
{
    /* allocate the geo-fence state if need be */
    if (geofence_state == NULL || !geofence_state->boundary_uptodate) {
        load();
        if (!geofence_enabled()) {
            // may have been disabled by load
            return;
        }
    }

    // check if vehicle is within the polygon
    Vector2l location(loc.lat, loc.lng);
    bool outside = Polygon_outside(location, &geofence_state->boundary[1], geofence_state->boundary_num_points-1);

    // check for change in breached state
    if (outside != geofence_state->breached_polygon) {
        geofence_state->breached_polygon = outside;
        if (outside) {
            geofence_state->breach_type = FENCE_BREACH_BOUNDARY;
            geofence_state->breach_count++;
            geofence_state->breach_time = AP_HAL::millis();
            gcs_send_text(MAV_SEVERITY_NOTICE,"Geofence triggered");
        } else {
            geofence_state->breach_type = FENCE_BREACH_NONE;
            gcs_send_text(MAV_SEVERITY_INFO,"Geofence OK");
        }
        gcs_send_message(MSG_FENCE_STATUS);
    }
}

/*
  maximum number of fencepoints
 */
uint8_t AP_PolyFence::max_points()
{
    return MIN(255U, fence_storage.size() / sizeof(Vector2l));
}
