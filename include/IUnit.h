#ifndef IUnit_h
#define IUnit_h

#include "Point.h"
#include "Orientation.h"
#include "IController.h"
#include "string"
#include <iostream>
#include <cmath>

class IUnit
{

public:
    virtual const Point &get_position() const = 0;

    virtual const Orientation &get_orientation() const = 0;

    virtual const Point &get_goal() const = 0;

    virtual void go_to(const Point &point, const Orientation &orientation) = 0;

    virtual float measure_delta_time() = 0;

    virtual void send_velocity_cmd(const float &vx, const float &vy, const float &vz, const float &wx, const float &wy, const float &wz) = 0;

    virtual void move(const float &time_now) = 0;

    virtual bool is_arrived(const bool &rot_needed) const = 0;

    virtual const std::string &get_id() const = 0;
};

#endif /* IUnit_h */