#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>

class AP_PolyFence_loader
{

public:

    // maximum number of fence points we can store in eeprom
    uint8_t max_points();

    // load boundary point from eeprom, returns true on successful load
    bool load_point_from_eeprom(uint16_t i, Vector2l& point);

    // save a fence point to eeprom, returns true on successful save
    bool save_point_to_eeprom(uint16_t i, const Vector2l& point);

    // validate array of boundary points (expressed as either floats or long ints)
    //   contains_return_point should be true for plane which stores the return point as the first point in the array
    //   returns true if boundary is valid
    bool boundary_valid(uint16_t num_points, const Vector2l* points, bool contains_return_point);
    bool boundary_valid(uint16_t num_points, const Vector2f* points, bool contains_return_point);

    // check if a location (expressed as either floats or long ints) is within the boundary
    //   contains_return_point should be true for plane which stores the return point as the first point in the array
    //   returns true if location is outside the boundary
    bool boundary_breached(const Vector2l& location, uint16_t num_points, const Vector2l* points, bool contains_return_point);
    bool boundary_breached(const Vector2f& location, uint16_t num_points, const Vector2f* points, bool contains_return_point);

};

