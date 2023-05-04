#ifndef FTG_NODE_H
#define FTG_NODE_H

#include <rclcpp/rclcpp.hpp>
// #include <ros/package.h>
#include <math.h>
#include <sys/time.h>
#include <tf2/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <yaml-cpp/yaml.h>

#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <cmath>
#include <geometry_msgs/msg/point32.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int8.hpp>
#include <thread>
#include <vector>
#include <visualization_msgs/msg/marker.hpp>

#include "utils/CSpline2D.h"

struct vector3d {
    // std::vector<double> *xs, *ys, *speed;
    double *xs, *ys, *speed;
    int length;
};

class Obstacle {
   public:
    Obstacle(float _angle, float _radius, float _distance) {
        angle = _angle;
        distance = _distance;  // distance could be measured in a better way... distance between
                               // center and radius
        radius = (_radius > distance - 0.01) ? distance - 0.01 : _radius;
        float theta = std::asin(radius / distance);
        angle_left = angle + theta;
        angle_right = angle - theta;
        // std::cout << "angle_left: " << angle_left << " angle_right: " << angle_right << "\n";
        center_x = distance * std::cos(angle);
        center_y = distance * std::sin(angle);
    }
    void setAngle(int val) { angle = val; }
    void setRadius(int val) { radius = val; }
    void setDistance(int val) { distance = val; }
    float getDistance() const { return distance; }
    int getAngle() const { return angle; }
    int getRadius() const { return radius; }

    float angle;
    float radius;
    float distance;
    float angle_left, angle_right;

    float center_x, center_y;
};

class Gap {
    float start_angle;
    float end_angle;
    float dist;
    float start_dist, end_dist;
    int size;

   public:
    Gap() = default;
    Gap(float _start_angle, float _end_angle, float _dist, float _start_dist, float _end_dist) {
        start_angle = _start_angle;
        end_angle = _end_angle;
        start_dist = _start_dist;
        end_dist = _end_dist;
        size = std::abs(start_angle - end_angle);
        dist = _dist;
    }
    void setSize(int val) { size = val; }
    float getDistance() const { return dist; }
    float getStartAngle() const { return start_angle; }
    float getEndAngle() const { return end_angle; }
    float getStartDistance() const { return start_dist; }
    float getEndDistance() const { return end_dist; }
    float getSize() const { return size; }
};

class DriveControl {
   private:
    float _actual_speed, _distance, _actual_steer;
    float _long_dist, _middle_dist, _short_dist;
    float _high_speed, _middle_speed, _low_speed;
    float _actual_throttle = 0.0;

    float _max_throttle = 3;

   public:
    DriveControl() = default;
    DriveControl(float max_throttle, float long_dist, float middle_dist, float short_dist,
                 float high_speed, float middle_speed, float low_speed)
        : _max_throttle(max_throttle),
          _long_dist(long_dist),
          _middle_dist(middle_dist),
          _short_dist(short_dist),
          _high_speed(high_speed),
          _middle_speed(middle_speed),
          _low_speed(low_speed){};

    float calcSteer(float angle1, float angle2);
    float calcThrottle(float distance);
    float calcThrottleNaive(float distance);
    float calcThrottleLerp(float distance);

    void setSpeed(float speed) { _actual_speed = speed; }
    float getSpeed() const { return _actual_speed; }
    void setSteer(float steer) { _actual_steer = steer; }
    float getSteer() const { return _actual_steer; }
    void setDistance(float distance) { _distance = distance; }
    float getDistance() const { return _distance; }
};

class FollowTheGapNode {
   private:
    rclcpp::Node::SharedPtr node;
    // rclcpp::Node::SharedPtr privateNode;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr pub_drive;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_estop;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr activate_ftg;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher_visualize_largest_gap;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr
        publisher_visualize_final_heading_angle;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher_visualize_obstacles;

    rclcpp::Subscription<geometry_msgs::msg::Point32>::SharedPtr driveSub;

   public:
    FollowTheGapNode() = default;
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan);
    void estop_callback(const std_msgs::msg::Bool::SharedPtr data);
    void driveCallback(const geometry_msgs::msg::Point32::SharedPtr msg);
#endif
    void start();
    void start_with_speeds();
    void PublishVisualizeFinalHeadingAngle(float final_heading_angle);
    void PublishVisualizeLargestGap(Gap const &largest_gap);
    void PublishVisualizeObstacles();
    int findNearest(float x_pos, float y_pos);
    bool load_flag_path(std::ifstream &is);
    Gap gap_rectify(Gap gap);

    std::vector<Gap> gaps;
    std::vector<Obstacle> obstacles;
    DriveControl drive_controller;

    Gap _last_gap;
    Gap _reduced_gap;
    bool _obstacle_check;
    bool _gap_reduced = false;

    int _range_method;
    float _frontal_reverse_distance;
    float _reverse_speed;

    float speed_to_follow = 0.0;
    int _speed_method = 0;
    std::string _directory = "";

    float _max_speed;

    struct vector3d *xyspeed;

    double heading_angle;
    float _field_of_view = M_PI / 1.5;
    float _min_range;
    float _max_range;
    float _min_angle_to_steer;
    float _safety_dist;
    float _min_gap_size;
    float target_speed = 0.0;
    float _long_dist;
    float _middle_dist;
    float _short_dist;

    float _actual_speed = 0.0;
    float _actual_steer = 0.0;
    float _max_rate_of_change = 0.4;
    float _k_accel;
    float _k_decel;
    float _diff_speed_accel;

    bool _in_reverse = false;
    bool _straight_check = false;

    bool _reverse_middle_state = false;

    float _max_steering_angle;

    float _obstacle_radius = 0.3;

    bool stop = true;

    ackermann_msgs::msg::AckermannDriveStamped drive_msg;

    bool can_publish = false;

    float _steering_angle_weight = 1;

    void pp_callback(const std_msgs::msg::Bool::SharedPtr data);
    bool pp_request = false;
    bool _pp_ftg = false;
};

void lookupTwist(const tf2_ros::Buffer &tfBuffer, const std::string &tracking_frame,
                 const std::string &observation_frame, const std::string &reference_frame,
                 const std::string &reference_point_frame, const rclcpp::Time &time,
                 const rclcpp::Duration &averaging_interval,
                 geometry_msgs::msg::Twist::SharedPtr twist);

void lookupTwist(const tf2_ros::Buffer &tfBuffer, const std::string &tracking_frame,
                 const std::string &observation_frame, const rclcpp::Time &time,
                 const rclcpp::Duration &averaging_interval,
                 geometry_msgs::msg::Twist::SharedPtr twist);


