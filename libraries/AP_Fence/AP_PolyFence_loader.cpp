#include "AP_PolyFence_loader.h"

extern const AP_HAL::HAL& hal;

static const StorageAccess fence_storage(StorageManager::StorageFence);

/*
  maximum number of fencepoints
 */
uint8_t AP_PolyFence_loader::max_points()
{
    return MIN(255U, fence_storage.size() / sizeof(Vector2l));
}

// load boundary point from eeprom, returns true on successful load
bool AP_PolyFence_loader::load_point_from_eeprom(uint16_t i, Vector2l& point)
{
    // sanity check index
    if (i >= max_points()) {
        return false;
    }

    // read fence point
    point.x = fence_storage.read_uint32(i * sizeof(Vector2l));
    point.y = fence_storage.read_uint32(i * sizeof(Vector2l) + 4);
    return true;
}

// save a fence point to eeprom, returns true on successful save
bool AP_PolyFence_loader::save_point_to_eeprom(uint16_t i, const Vector2l& point)
{
    // sanity check index
    if (i >= max_points()) {
        return false;
    }

    // write point to eeprom
    fence_storage.write_uint32(i * sizeof(Vector2l), point.x);
    fence_storage.write_uint32(i * sizeof(Vector2l)+4, point.y);
    return true;
}

// validate array of boundary points (expressed as either floats or long ints)
//   contains_return_point should be true for plane which stores the return point as the first point in the array
//   returns true if boundary is valid
bool AP_PolyFence_loader::boundary_valid(uint16_t num_points, const Vector2l* points, bool contains_return_point)
{
    // start from 2nd point if boundary contains return point (as first point)
    uint8_t start_num = contains_return_point ? 1 : 0;

    // a boundary requires at least 4 point (a triangle and last point equals first)
    if (num_points < start_num + 4) {
        return false;
    }

    // point 1 and last point must be the same.  Note: 0th point is reserved as the return point
    if (!Polygon_complete(&points[start_num], num_points-1)) {
        return false;
    }

    // check return point is within the fence
    if (contains_return_point && Polygon_outside(points[0], &points[1], num_points-1)) {
        return false;
    }

    return true;
}

bool AP_PolyFence_loader::boundary_valid(uint16_t num_points, const Vector2f* points, bool contains_return_point)
{
    // start from 2nd point if boundary contains return point (as first point)
    uint8_t start_num = contains_return_point ? 1 : 0;

    // a boundary requires at least 4 point (a triangle and last point equals first)
    if (num_points < start_num + 4) {
        return false;
    }

    // point 1 and last point must be the same.  Note: 0th point is reserved as the return point
    if (!Polygon_complete(&points[start_num], num_points-1)) {
        return false;
    }

    // check return point is within the fence
    if (contains_return_point && Polygon_outside(points[0], &points[1], num_points-1)) {
        return false;
    }

    return true;
}

// check if a location (expressed as either floats or long ints) is within the boundary
//   contains_return_point should be true for plane which stores the return point as the first point in the array
//   returns true if location is outside the boundary
bool AP_PolyFence_loader::boundary_breached(const Vector2l& location, uint16_t num_points, const Vector2l* points, bool contains_return_point)
{
    // start from 2nd point if boundary contains return point (as first point)
    uint8_t start_num = contains_return_point ? 1 : 0;

    // check location is within the fence
    return Polygon_outside(location, &points[start_num], num_points-1);
}

bool AP_PolyFence_loader::boundary_breached(const Vector2f& location, uint16_t num_points, const Vector2f* points, bool contains_return_point)
{
    // start from 2nd point if boundary contains return point (as first point)
    uint8_t start_num = contains_return_point ? 1 : 0;

    // check location is within the fence
    return Polygon_outside(location, &points[start_num], num_points-1);
}
