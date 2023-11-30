#include "SimpleUnit.h"
#include <Eigen/Dense>

SimpleUnit::SimpleUnit(const std::string &id, const Point &initial_position, const Orientation &initial_orientation, std::shared_ptr<IController> controller, const float &speed, const float &space_tolerance, const float &orientation_tolerance) : position_(initial_position), orientation_(initial_orientation), initial_orientation_(initial_orientation), goal_(initial_position), speed_(speed), space_tolerance_(space_tolerance), controller_(controller), goal_orientation_(initial_orientation), orientation_tolerance_(orientation_tolerance)
{
    id_ = id;
    time_now_ = 0.0;
    time_old_ = 0.0;
}

const Point &SimpleUnit::get_position() const
{
    return position_;
}

const Orientation &SimpleUnit::get_orientation() const
{
    return orientation_;
}

const Point &SimpleUnit::get_goal() const
{
    return goal_;
}

void SimpleUnit::go_to(const Point &point, const Orientation &orientation)
{
    goal_ = point;
    goal_orientation_ = orientation;
}

const std::string &SimpleUnit::get_id() const
{
    return id_;
}

bool SimpleUnit::operator==(const SimpleUnit &other_unit) const
{
    return id_ == other_unit.id_;
}

float SimpleUnit::measure_delta_time()
{
    if (time_old_ == 0.0)
    {
        time_old_ = time_now_;
    }

    float time_old = time_old_;
    time_old_ = time_now_;
    return time_now_ - time_old;
}

void SimpleUnit::send_velocity_cmd(const float &vx, const float &vy, const float &vz, const float &wx, const float &wy, const float &wz)
{

    float delta_time = measure_delta_time();

    wx_ += wx * delta_time;
    wy_ += wy * delta_time;
    wz_ += wz * delta_time;

    Eigen::Matrix3d rotation_world = Eigen::Matrix3d::Identity();

    Eigen::Matrix3d rotation_x_initial = Eigen::AngleAxisd(initial_orientation_.x, Eigen::Vector3d::UnitX()).toRotationMatrix();
    Eigen::Matrix3d rotation_y_initial = Eigen::AngleAxisd(initial_orientation_.y, Eigen::Vector3d::UnitY()).toRotationMatrix();
    Eigen::Matrix3d rotation_z_initial = Eigen::AngleAxisd(initial_orientation_.z, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    Eigen::Matrix3d initial_orientation = rotation_z_initial * rotation_y_initial * rotation_x_initial;

    Eigen::Matrix3d rotation_x_current = Eigen::AngleAxisd(wx_, Eigen::Vector3d::UnitX()).toRotationMatrix();
    Eigen::Matrix3d rotation_y_current = Eigen::AngleAxisd(wy_, Eigen::Vector3d::UnitY()).toRotationMatrix();
    Eigen::Matrix3d rotation_z_current = Eigen::AngleAxisd(wz_, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    Eigen::Matrix3d rotation_current = rotation_x_current * rotation_y_current * rotation_z_current;

    Eigen::Matrix3d omega_global = rotation_world * initial_orientation * rotation_current;
    Eigen::Vector3d euler_angles = omega_global.eulerAngles(0, 1, 2);

    orientation_.x = euler_angles[0];
    orientation_.y = euler_angles[1];
    orientation_.z = euler_angles[2];

    Eigen::VectorXd v(3);
    v << vx, vy, vz;

    Eigen::Matrix3d rotation_x = Eigen::AngleAxisd(orientation_.x, Eigen::Vector3d::UnitX()).toRotationMatrix();
    Eigen::Matrix3d rotation_y = Eigen::AngleAxisd(orientation_.y, Eigen::Vector3d::UnitY()).toRotationMatrix();
    Eigen::Matrix3d rotation_z = Eigen::AngleAxisd(orientation_.z, Eigen::Vector3d::UnitZ()).toRotationMatrix();

    Eigen::VectorXd p(3);
    p = rotation_z * rotation_y * rotation_x * v * delta_time;

    position_.x += p(0);
    position_.y += p(1);
    position_.z += p(2);

    position_.x = std::round(position_.x * 100.0) / 100.0;
    position_.y = std::round(position_.y * 100.0) / 100.0;
    position_.z = std::round(position_.z * 100.0) / 100.0;
}

void SimpleUnit::move(const float &time_now)
{
    time_now_ = time_now;

    float dx = goal_.x - position_.x;
    float dy = goal_.y - position_.y;
    float dz = goal_.z - position_.z;

    float dwx = goal_orientation_.x - orientation_.x;
    float dwy = goal_orientation_.y - orientation_.y;
    float dwz = goal_orientation_.z - orientation_.z;

    // velocities in the world frame
    std::vector<float> velocities = controller_->compute(dx, dy, dz, dwx, dwy, dwz, orientation_.x, orientation_.y, orientation_.z, speed_);
    Eigen::VectorXd v_world(3);
    v_world << velocities[0], velocities[1], velocities[2];

    Eigen::Matrix3d rotation_x = Eigen::AngleAxisd(orientation_.x, Eigen::Vector3d::UnitX()).toRotationMatrix();
    Eigen::Matrix3d rotation_y = Eigen::AngleAxisd(orientation_.y, Eigen::Vector3d::UnitY()).toRotationMatrix();
    Eigen::Matrix3d rotation_z = Eigen::AngleAxisd(orientation_.z, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    Eigen::Matrix3d rot = (rotation_z * rotation_y * rotation_x).inverse();

    Eigen::VectorXd v_robot(3);
    v_robot = rot * v_world;

    // velocities in the robot frame
    send_velocity_cmd(v_robot(0), v_robot(1), v_robot(2), velocities[3], velocities[4], velocities[5]);
}

bool SimpleUnit::is_arrived(const bool &rot_needed) const
{
    float dx = goal_.x - position_.x;
    float dy = goal_.y - position_.y;
    float dz = goal_.z - position_.z;

    float dwx = goal_orientation_.x - orientation_.x;
    float dwy = goal_orientation_.y - orientation_.y;
    float dwz = goal_orientation_.z - orientation_.z;

    double dist_norm = sqrt((dx * dx) + (dy * dy) + (dz * dz));
    double orientation_norm = sqrt((dwx * dwx) + (dwy * dwy) + (dwz * dwz));

    // std::cout << "orientation_norm" << orientation_norm << std::endl;

    if (dist_norm < space_tolerance_)
    {
        if (!rot_needed || orientation_norm < orientation_tolerance_ || abs(orientation_norm - M_PI) < orientation_tolerance_)
        {
            return true;
        }
        else
        {
            return false;
        }
    }
    else
    {
        return false;
    }
}
