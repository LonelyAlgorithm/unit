#ifndef SimpleUnit_h
#define SimpleUnit_h

#include "IUnit.h"

class SimpleUnit : public IUnit
{

public:
    SimpleUnit(const std::string &id, const Point &initial_position, const Orientation &initial_orientation, std::shared_ptr<IController> controller, const float &speed, const float &space_tolerance, const float &orientation_tolerance);

    const Point &get_position() const override;

    const Orientation &get_orientation() const override;

    const Point &get_goal() const override;

    float measure_delta_time() override;

    void send_velocity_cmd(const float &vx, const float &vy, const float &vz, const float &wx, const float &wy, const float &wz) override;

    void move(const float &time_now) override;

    void go_to(const Point &point, const Orientation &orientation) override;

    bool operator==(const SimpleUnit &other_unit) const;

    const std::string &get_id() const override;

    bool is_arrived(const bool &rot_needed) const override;

private:
    std::string id_;
    Point position_;
    Orientation orientation_;
    Orientation initial_orientation_;
    Point goal_;
    Orientation goal_orientation_;
    float speed_;
    float time_old_;
    float time_now_;
    float space_tolerance_;
    float orientation_tolerance_;

    double wx_ = 0.0;
    double wy_ = 0.0;
    double wz_ = 0.0;

    std::shared_ptr<IController> controller_;
};

#endif /* SimpleUnit_h */