#include "ftg_node.h"

#include <math.h>

#include <cstdio>
#include <fstream>
#include <iostream>
#include <string>

using std::placeholders::_1;

bool FollowTheGapNode::load_flag_path(std::ifstream& is)
{
    std::string line;
    size_t pos = 0;
    std::string temp;
    int i;
    // file format:
    // - Name: Flag 0
    // X: 0.442536
    // Y: 0.320954
    // Z: 4.49081
    xyspeed = (struct vector3d*)malloc(sizeof(struct vector3d));
    xyspeed->xs = (double*)malloc(sizeof(double) * 1000);
    xyspeed->ys = (double*)malloc(sizeof(double) * 1000);
    xyspeed->speed = (double*)malloc(sizeof(double) * 1000);
    try {
        for (i = 0; std::getline(is, line); i++) // Read useless line
        {
            // read x
            std::getline(is, line);
            // std::cout << line << "\n";
            pos = line.find(": ") + 2;
            temp = line.substr(pos, std::string::npos);
            // std::cout << temp << "\n";
            xyspeed->xs[i] = std::stod(temp);
            // read y
            std::getline(is, line);
            // std::cout << line << "\n";
            pos = line.find(": ") + 2;
            // std::cout << line.substr(pos, std::string::npos).c_str() << "\n";
            xyspeed->ys[i] = atof(line.substr(pos, std::string::npos).c_str());
            // read speed
            std::getline(is, line);
            // std::cout << line << "\n";
            pos = line.find(": ") + 2;
            // std::cout << line.substr(pos, std::string::npos).c_str() << "\n";
            xyspeed->speed[i] = atof(line.substr(pos, std::string::npos).c_str());
        }
        xyspeed->length = i;
    } catch (const std::exception& e) {
        std::cout << "Error loading flag path \n";
        return false;
    }
    return true;
}

float DriveControl::calcThrottle(float distance)
{
    // float distance = getDistance();
    float steer = getSteer();
    float throttle_perc = 0.3;
    if (_actual_speed > 10) // in order to avoid wrong values from the unity sim
    {
        return _actual_throttle;
    } else if (distance < _short_dist) {
        if (_actual_speed > _low_speed) {
            throttle_perc = 0;
        } else {
            throttle_perc = 0.2;
        }
    } else if (distance < _middle_dist) {
        if (_actual_speed > _middle_dist) {
            throttle_perc = 0;
        } else {
            throttle_perc = 0.4;
        }
    } else if (distance < _long_dist) {
        if (_actual_speed > _middle_dist) {
            throttle_perc = 0;
        } else {
            throttle_perc = 0.7;
        }
    } else // distance > _long_dist
    {
        throttle_perc = 1;
    }
    _actual_throttle = _max_throttle * throttle_perc;
    return _actual_throttle;
}

float DriveControl::calcThrottleNaive(float distance)
{
    float speed = _low_speed;
    float perc;
    if (distance > _long_dist) {
        speed = _high_speed;
    } else if (distance > _middle_dist) {
        speed = _middle_speed;
    } else if (distance > 0.1) {
        speed = _low_speed;
    } else // distance > _long_dist
    {
        speed = 0;
    }
    //_long_dist : _high_speed = distance : x
    speed = (distance * _high_speed) / _long_dist;
    if (speed < _low_speed) {
        speed = _low_speed;
    }
    return speed;
}

float DriveControl::calcThrottleLerp(float distance)
{
    float speed;
    double fraction;
    if (distance > _long_dist) {
        speed = _high_speed;
    } else if (distance > _middle_dist) {
        fraction = std::abs((distance - _middle_dist) / (_long_dist - _middle_dist));
        speed = _middle_speed + fraction * (_high_speed - _middle_speed);
    } else if (distance > _short_dist) {
        fraction = std::abs((distance - _short_dist) / (_middle_dist - _short_dist));
        speed = _low_speed + fraction * (_middle_speed - _low_speed);
    } else {
        fraction = std::abs((distance) / (_short_dist));
        speed = fraction * (_low_speed);
    }

    return speed;
}

float calcThrottle(float central_distance)
{
    float throttle = 0.4;
    if (central_distance < 3) {
        throttle = 0.2;
    } else if (central_distance < 5) {
        throttle = 0.3;
    }
    return throttle;
}

Gap gap_rectify(Gap gap)
{
    // float diff = std::abs(gap.start_angle) - std::abs(gap.end_angle);
    /*if(diff < 0.2 && diff > -0.2)
    {
        if(diff > 0)
        {

        }
        else
        {

        }
    }*/
    return gap;
}

float DriveControl::calcSteer(float angle1, float angle2)
{
    float steer = (angle1 + angle2) / 2;
    float diff = std::abs(angle1) - std::abs(angle2);
    // std::cout << "diff: " << diff << "\n";
    /*if(diff > 0.5)
    {
      steer = angle1;
    }
    else if(diff < -0.5)
    {
      steer = angle2;
    }*/
    return (angle1 + angle2) / 2;
    // return steer;
}

void FollowTheGapNode::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan)
{
    float angle = scan->angle_min;
    bool good_result = true;
    bool virtual_left = true;
    bool virtual_right = true;
    float maximum_range, minimum_range;
    int center = scan->ranges.size() / 2;
    float frontal_distance = scan->ranges[center];
    bool gap_founded = false;
    bool can_i_go = true;
    stop = false;

    // Check if the car is frontaly blocked or near to be
    float turn_back = _frontal_reverse_distance;
    if (_in_reverse) {
        turn_back = _frontal_reverse_distance + 0.3;
    }
    if (!_pp_ftg) {
        can_i_go = !stop;
    } else {
        can_i_go = pp_request;
    }
    if (!can_i_go) {
        return;
    }
    if (frontal_distance < turn_back) {
        _in_reverse = true;
        if (can_i_go) {
            can_publish = true;
            if (_reverse_middle_state) {
                drive_msg.drive.speed = -_reverse_speed;
            } else {
                _reverse_middle_state = true;
                drive_msg.drive.speed = 0;
            }
            drive_msg.drive.steering_angle = 0;
            pub_drive->publish(drive_msg);
        }
        return;
    } else {
        _in_reverse = false;
        _reverse_middle_state = false;
    }
    if (_straight_check && scan->ranges[center] > _long_dist * 2 &&
        scan->ranges[center + 40] > _long_dist && scan->ranges[center - 40] > _long_dist) {
        float speed, steer;
        steer = 0.0;
        if (_speed_method == 0) {
            speed = drive_controller.calcThrottleLerp(scan->ranges[center]);
        } else {
            speed = speed_to_follow;
        }
        if (speed > _max_speed) {
            speed = _max_speed;
        }
        PublishVisualizeFinalHeadingAngle(steer);
        if (can_i_go) {
            can_publish = true;
            drive_msg.drive.speed = speed;
            drive_msg.drive.steering_angle = steer;
        }
        if (can_publish) {
            pub_drive->publish(drive_msg);
        }
        return;
    }
    if (_range_method == 0) {
        // should be chosen the nearest gap
        maximum_range = _max_range;
        minimum_range = _min_range;
    } else {
        // dynamic range in relation to the front_distance
        float dist;
        if (frontal_distance > _long_dist) {
            dist = _long_dist;
        } else if (frontal_distance > _middle_dist) {
            dist = _middle_dist;
        } else {
            dist = _short_dist;
        }
        maximum_range = dist + 0.5;
        minimum_range = dist - 0.5;
    }
    // Follow the gap algorithm repeated until at least one gap is founded
    // while(!gap_founded )
    //{
    //-----------------------------Populate obstacles vector
    for (int i = 0; i < scan->ranges.size(); ++i, angle += scan->angle_increment) {
        float actual_range = scan->ranges[i];
        // Do some checks
        if (std::isnan(actual_range) || angle < scan->angle_min || angle > scan->angle_max ||
            std::abs(angle) > _field_of_view || actual_range > maximum_range ||
            actual_range < minimum_range || actual_range < scan->range_min ||
            actual_range > scan->range_max) {
            // ignore the actual range
            continue;
        }
        // std::cout << "added obstacle, angle: " << angle << " distance: " << actual_range << "\n";
        if (angle > 0) {
            virtual_left = false;
        } else if (angle < 0) {
            virtual_right = false;
        }
        obstacles.emplace_back(angle, _obstacle_radius, actual_range);
    }
    if (virtual_left) {
        for (int i = 0; i < obstacles.size(); i++) {
            float angle = obstacles[i].angle;
            if (std::abs(angle) > M_PI / 6) {
                angle = -angle;
                obstacles.emplace_back(angle, _obstacle_radius, _max_range - 0.5);
                std::cout << "added LEFT VIRTUAL obstacle, angle: " << angle
                         << " distance: " << _max_range << "\n";
                break;
            }
        }
    }
    std::reverse(obstacles.begin(), obstacles.end());
    if (virtual_right) {
        for (int i = 0; i < obstacles.size(); i++) {
            float angle = obstacles[i].angle;
            if (std::abs(angle) > M_PI / 6) {
                angle = -angle;
                obstacles.emplace_back(angle, _obstacle_radius, _max_range - 0.5);
                std::cout << "added RIGHT VIRTUAL obstacle, angle: " << angle
                         << " distance: " << _max_range << "\n";
                break;
            }
        }
    }
    // It should be better to do some filtering...
    //--------------------------------------------------------
    // std::cout << "obstacle vector populated, size: " << obstacles.size() << "\n";
    //------------------Populate Gaps vector
    for (int i = 1; i < obstacles.size(); i++) {
        // check if obstacles are not connected
        if (obstacles[i - 1].angle_right > obstacles[i].angle_left) {
            float distance_between_obstacles =
              std::hypot(obstacles[i - 1].center_x - obstacles[i].center_x,
                         obstacles[i - 1].center_y - obstacles[i].center_y);
            // std::cout << "distance_between_obstacles: " << distance_between_obstacles << "\n";
            float distance_obs_edges =
              distance_between_obstacles - obstacles[i].radius - obstacles[i - 1].radius;
            if (distance_between_obstacles > _min_gap_size) {
                // create gap
                Gap gap(obstacles[i - 1].angle_right,
                        obstacles[i].angle_left,
                        distance_between_obstacles,
                        obstacles[i - 1].distance,
                        obstacles[i].distance);
                // Gap gap(obstacles[i-1].angle_right, obstacles[i].angle_right,
                // distance_between_obstacles,
                //         obstacles[i-1].distance, obstacles[i].distance);
                gaps.emplace_back(gap);
            }
        }
    }
    // std::cout << "gaps vector populated, size: " << gaps.size() << "\n";
    // if(gaps.size() > 0)
    // {
    //   gap_founded = true;
    // }
    // else
    // {
    //   std::cout << "ci entro... \n";
    //   gaps.clear();
    //   obstacles.clear();
    //   std::vector<Gap>().swap(gaps);
    //   std::vector<Obstacle>().swap(obstacles);
    //   maximum_range += 0.5;
    //   angle = scan->angle_min;
    // }
    // STEP1: Find the largest gap
    int index_largest = 0;
    for (int i = 0; i < gaps.size(); i++) {
        if (gaps[i].getSize() > gaps[index_largest].getSize()) {
            index_largest = i;
        }
    }
    // STEP2: Calculate gap center angle
    double gap_center_angle;
    double d1 = gaps[index_largest].getEndDistance();
    double d2 = gaps[index_largest].getStartDistance();
    double angle1 = gaps[index_largest].getEndAngle();
    double angle2 = gaps[index_largest].getStartAngle();
    // Check difference from the last gap
    // if(_obstacle_check && !stop)
    // {
    //     if(_last_gap  > gaps[index_largest].size + 0.2)//gap ridotto
    //     {
    //         _gap_reduced = true;
    //         _reduced_gap = _last_gap;
    //     }
    //     else if(_last_gap  < gaps[index_largest].size - 0.2)//gap aumentato
    //     {
    //         if(_gap_reduced)//si era ridotto
    //         {
    //             float angle_to_check;
    //             //Check if the angle is safe
    //             if(abs(_reduced_gap.angle_left) > abs(_reduced_gap.angle_right))
    //             {
    //                 angle_to_check = angle_left;
    //             }
    //             else
    //             {
    //                 angle_to_check = angle_right;
    //             }
    //             float angle;
    //             if (angle_to_check > 0)
    //             {
    //                 angle = 0;
    //             }
    //             else
    //             {
    //             }
    //         }
    //     }
    // }
    // std::cout << "d1: " << d1 << " d2: " << d2 << " angle1: " << angle1 << " angle2: " << angle2
    // << "\n";
    double upper_part = d1 + d2 * std::cos(angle1 + angle2);
    double lower_part = std::sqrt(d1 * d1 + 2 * d1 * d2 * std::cos(angle1 + angle2));
    // std::cout << "upper part: " << upper_part << " lower part: " << lower_part << "\n";
    gap_center_angle = std::acos(upper_part / lower_part);
    // std::cout << "ftg center angle" << gap_center_angle << "\n";
    if (std::isnan(gap_center_angle)) {
        // gap not suitable
        // try this naive version
        // std::cout << "naive version of gap center\n";
        // gap_center_angle = (angle1 + angle2) / 2;
        gap_center_angle = drive_controller.calcSteer(angle1, angle2);
    }
    // std::cout << "gap center angle: " << gap_center_angle << "\n";
    // STEP3: Calculate final heading angle
    int index_nearest = 0;
    for (int i = 0; i < obstacles.size(); i++) {
        if (obstacles[i].distance > obstacles[index_nearest].distance) {
            index_nearest = i;
        }
    }
    float alpha_coeff = 100;
    // should be added the goal angle...
    heading_angle =
      ((alpha_coeff / obstacles[index_nearest].distance) * gap_center_angle) /
      ((alpha_coeff / obstacles[index_nearest].distance) + 1); // should be added a 1...
    // std::cout << "final heading angle: " << heading_angle << "\n";
    if (std::isnan(heading_angle)) {
        good_result = false;
    }
    if (good_result) {
        _last_gap = gaps[index_largest];
        float speed, steer;
        steer = heading_angle;
        int center = scan->ranges.size() / 2;
        if (steer > _max_steering_angle) {
            steer = _max_steering_angle;
        } else if (steer < -_max_steering_angle) {
            steer = -_max_steering_angle;
        }
        drive_controller.setSteer(steer);
        if (_speed_method == 0) {
            speed = drive_controller.calcThrottleLerp(scan->ranges[center]);
        } else {
            speed = speed_to_follow;
        }
        if (speed > _max_speed) {
            speed = _max_speed;
        }
        // std::cout << "central_distance: " << scan->ranges[center] << " ||| speed: " << speed
        //          << " ||| steer: " << steer << "\n";
        // std::cout << "min steer: " << _min_angle_to_steer << "\n";
        PublishVisualizeFinalHeadingAngle(heading_angle);
        PublishVisualizeLargestGap(gaps[index_largest]);
        PublishVisualizeObstacles();
        if (can_i_go) {
            can_publish = true;
            drive_msg.drive.speed = speed;
            // limit accel
            float diff_speed = std::abs(_actual_speed - speed);
            if (diff_speed > _diff_speed_accel) {
                if (speed > _actual_speed) {
                    speed = _actual_speed + _k_accel * diff_speed;
                } else {
                    speed = _actual_speed - _k_decel * diff_speed;
                }
            }
            /*if(std::abs(steer) < _min_angle_to_steer)
            {
              steer = 0.0;
            }*/
            steer = steer * _steering_angle_weight;
            // limit rate of change
            float diff_steer = std::abs(_actual_steer - steer);
            if (diff_steer > _max_rate_of_change) {
                if (steer > _actual_steer) {
                    steer = _actual_steer + _max_rate_of_change;
                } else {
                    steer = _actual_steer + _max_rate_of_change;
                }
            }
            // reduce steer - can be removed maybe
            steer *= _steering_angle_weight;
            drive_msg.drive.steering_angle = steer; // left positive
        }
    }
    if (can_publish) {
        pub_drive->publish(drive_msg);
    }
    gaps.clear();
    obstacles.clear();
}

void FollowTheGapNode::PublishVisualizeObstacles()
{
    visualization_msgs::msg::Marker obstacle_points;
    obstacle_points.header.stamp = node->get_clock()->now();
    obstacle_points.type = visualization_msgs::msg::Marker::POINTS;
    obstacle_points.header.frame_id = "/ego_racecar/laser";
    obstacle_points.scale.x = 0.05;
    obstacle_points.scale.y = 0.05;
    obstacle_points.color.r = 0;
    obstacle_points.color.g = 1;
    obstacle_points.color.b = 0;
    obstacle_points.color.a = 1;
    for (auto const obs : obstacles) {
        geometry_msgs::msg::Point p;
        p.x = obs.center_x;
        p.y = obs.center_y;
        obstacle_points.points.push_back(p);
    }
    publisher_visualize_obstacles->publish(obstacle_points);
}

void FollowTheGapNode::PublishVisualizeFinalHeadingAngle(float final_heading_angle)
{
    geometry_msgs::msg::PoseStamped angle_message;
    angle_message.header.frame_id = "/ego_racecar/laser";
    tf2::Quaternion angle_quaternion;
    angle_quaternion.setEuler(0, 0, final_heading_angle);
    angle_quaternion.normalize();
    // tf2::quaternionTFToMsg(angle_quaternion, angle_message.pose.orientation);
    angle_message.pose.orientation = tf2::toMsg(angle_quaternion);
    publisher_visualize_final_heading_angle->publish(angle_message);
}

void FollowTheGapNode::PublishVisualizeLargestGap(Gap const& largest_gap)
{
    geometry_msgs::msg::PointStamped p0, p1, robot_point;
    robot_point.header.frame_id = "/ego_racecar/laser";
    robot_point.point.x = 0;
    robot_point.point.y = 0;
    p0.point.x = largest_gap.getStartDistance() * std::cos(largest_gap.getEndAngle());
    p0.point.y = largest_gap.getStartDistance() * std::sin(largest_gap.getEndAngle());
    p0.header.frame_id = "/ego_racecar/laser";
    p1.point.x = largest_gap.getEndDistance() * std::cos(largest_gap.getStartAngle());
    p1.point.y = largest_gap.getEndDistance() * std::sin(largest_gap.getStartAngle());
    p1.header.frame_id = "/ego_racecar/laser";
    visualization_msgs::msg::Marker line;
    line.type = visualization_msgs::msg::Marker::LINE_LIST;
    line.header.frame_id = "/ego_racecar/laser";
    line.scale.x = 0.2;
    line.color.r = 1.0;
    line.color.a = 1.0;
    line.points.push_back(p0.point);
    line.points.push_back(p1.point);
    publisher_visualize_largest_gap->publish(line);
}

void FollowTheGapNode::estop_callback(const std_msgs::msg::Bool::SharedPtr data)
{
    if (data->data == false)
        stop = false;
}

void FollowTheGapNode::pp_callback(const std_msgs::msg::Bool::SharedPtr data)
{
    pp_request = data->data;
}

void FollowTheGapNode::driveCallback(const geometry_msgs::msg::Point32::SharedPtr msg)
{
    _actual_speed = msg->x;
    _actual_steer = msg->y;
}

float distance(float x1, float y1, float x2, float y2)
{
    return std::sqrt(std::pow(x2 - x1, 2) + std::pow(y2 - y1, 2));
}

int FollowTheGapNode::findNearest(float x_pos, float y_pos)
{
    int min = 0;
    float dist_temp;
    float dist_min = distance(x_pos, y_pos, xyspeed->xs[min], xyspeed->ys[min]);
    for (int i = 0; i < xyspeed->length; ++i) {
        dist_temp = distance(x_pos, y_pos, xyspeed->xs[i], xyspeed->ys[i]);
        if (dist_temp < dist_min) {
            dist_min = dist_temp;
            min = i;
        }
    }
    return min;
}

void FollowTheGapNode::start_with_speeds()
{
    std::string param_file;
    node = std::make_shared<rclcpp::Node>("ftg_node");
    node->declare_parameter("drive_param");
    rclcpp::Parameter param = node->get_parameter("drive_param");
    param_file = param.as_string();
    YAML::Node conf = YAML::LoadFile(param_file);
    _pp_ftg = conf["pp_ftg"].as<bool>();
    _steering_angle_weight = conf["steering_angle_weight"].as<float>();
    _max_rate_of_change = conf["steering_angle_weight"].as<float>();
    _k_accel = conf["k_accel"].as<float>();
    _k_decel = conf["k_decel"].as<float>();
    _diff_speed_accel = conf["diff_speed_accel"].as<float>();
    _speed_method = conf["speed_method"].as<int>();
    _min_angle_to_steer = conf["min_angle_to_steer"].as<float>();
    _min_range = conf["min_range"].as<float>();
    _max_range = conf["max_range"].as<float>();
    _safety_dist = conf["safety_distance_from_obstacle"].as<float>();
    _max_steering_angle = conf["max_steer"].as<float>();
    _min_gap_size = conf["min_gap_size"].as<float>();
    _obstacle_check = conf["obstacle_check"].as<bool>();
    _field_of_view = conf["field_of_view"].as<float>();
    _range_method = conf["range_method"].as<int>();
    _frontal_reverse_distance = conf["frontal_reverse_distance"].as<float>();
    _reverse_speed = conf["reverse_speed"].as<float>();
    _obstacle_radius = conf["obstacle_radius"].as<float>();
    _long_dist = conf["long_distance"].as<float>();
    _middle_dist = conf["middle_distance"].as<float>();
    _short_dist = conf["short_distance"].as<float>();
    _max_speed = conf["max_speed"].as<float>();
    _straight_check = conf["straight_check"].as<bool>();
    _directory = conf["directory"].as<std::string>();
    if (_speed_method == 1) {
        std::cout << "FTG_NODE STARTED...\n";
        //-------------------------Load parameters-------------------------//
        float long_d = conf["long_distance"].as<float>();
        float middle_d = conf["middle_distance"].as<float>();
        float short_d = conf["short_distance"].as<float>();
        float high_s = conf["high_speed"].as<float>();
        float middle_s = conf["middle_speed"].as<float>();
        float low_s = conf["low_speed"].as<float>();
        float max_throttle = conf["max_throttle"].as<float>();
        // sub, pub
        scan_sub = node->create_subscription<sensor_msgs::msg::LaserScan>(
          "/scan", 1, std::bind(&FollowTheGapNode::scanCallback, this, _1));
        sub_estop = node->create_subscription<std_msgs::msg::Bool>(
          "/commands/stop", 1, std::bind(&FollowTheGapNode::estop_callback, this, _1));
        activate_ftg = node->create_subscription<std_msgs::msg::Bool>(
          "activate_ftg", 1, std::bind(&FollowTheGapNode::pp_callback, this, _1));
        driveSub = node->create_subscription<geometry_msgs::msg::Point32>(
          "/drive_act",
          rclcpp::SensorDataQoS(),
          std::bind(&FollowTheGapNode::driveCallback, this, _1));
        // pub_heading_angle = nodeHandler.advertise<std_msgs::Float32>("heading_angle", 100);
        pub_drive = node->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/drive", 1);
        publisher_visualize_largest_gap =
          node->create_publisher<visualization_msgs::msg::Marker>("/visualize_largest_gap", 1);
        publisher_visualize_final_heading_angle =
          node->create_publisher<geometry_msgs::msg::PoseStamped>("/visualize_final_heading_angle",
                                                                  1);
        publisher_visualize_obstacles =
          node->create_publisher<visualization_msgs::msg::Marker>("/visualize_obstacles", 1);
        drive_controller =
          DriveControl(max_throttle, long_d, middle_d, short_d, high_s, middle_s, low_s);
        //----------------------------------------------------------------//
        std::ifstream is(_directory, std::ifstream::in);
        if (!is)
            std::cout << "Cannot open path file\n";
        std::cout << "I'm loading path info... \n";
        load_flag_path(is);
        // CSpline2D path = CSpline2D();
        // path.init(xyspeed.xs, xyspeed.ys, xyspeed.length);
        rclcpp::Rate r(40);
        tf2_ros::Buffer tfBuffer(node->get_clock());
        tf2_ros::TransformListener listener(tfBuffer);
        geometry_msgs::msg::Twist twist;
        tf2::Stamped<tf2::Transform> transform;
        geometry_msgs::msg::TransformStamped geoTransform;
        float pos_x, pos_y;
        int path_idx;
        while (rclcpp::ok()) {
            try {
                geoTransform = tfBuffer.lookupTransform("/map", "/base_link", rclcpp::Time(0));
                tf2::fromMsg(geoTransform, transform);
            } catch (const std::exception& e) {
                std::cout << "NO tf \n";
                continue;
            }
            // std::cout << "ok \n";
            float speed = std::sqrt(std::pow(twist.linear.x, 2) + std::pow(twist.linear.y, 2));
            float angular_vel = std::atan2(twist.linear.y, twist.linear.x);
            // std::cout << "speed: " << speed << " | angular_vel: " << angular_vel << "\n";
            drive_controller.setSpeed(speed);
            pos_x = transform.getOrigin().x();
            pos_y = transform.getOrigin().y();
            // path.update_current_s(pos_x, pos_y);
            // float p_pos = path.cur_s + 0.32;
            // int speed_idx = int((p_pos / path.s[xyspeed.length-1])*xyspeed.length);
            // float target_speed = xyspeed.speed[speed_idx % xyspeed.length];
            path_idx = findNearest(pos_x, pos_y);
            target_speed = xyspeed->speed[path_idx];
            // float target_speed = xyspeed.speed[speed_idx % xyspeed.length];
            // std::cout << "target speed: " << target_speed << "\n";
            speed_to_follow = target_speed;
            rclcpp::spin_some(node);
            r.sleep();
        }
    } else {
        start();
    }
}

void FollowTheGapNode::start()
{
    std::cout << "FTG_NODE STARTED..." << std::endl;
    // rclcpp::Node::SharedPtr privateNode = std::make_shared<rclcpp::Node>("~");
    // sub, pub
    scan_sub = node->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", 1, std::bind(&FollowTheGapNode::scanCallback, this, _1));
    sub_estop = node->create_subscription<std_msgs::msg::Bool>(
      "commands/stop", 1, std::bind(&FollowTheGapNode::estop_callback, this, _1));
    activate_ftg = node->create_subscription<std_msgs::msg::Bool>(
      "activate_ftg", 1, std::bind(&FollowTheGapNode::pp_callback, this, _1));
    // pub_heading_angle = nodeHandler.advertise<std_msgs::Float32>("heading_angle", 100);
    pub_drive = node->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/drive", 1);
    publisher_visualize_largest_gap =
      node->create_publisher<visualization_msgs::msg::Marker>("/visualize_largest_gap", 1);
    publisher_visualize_final_heading_angle =
      node->create_publisher<geometry_msgs::msg::PoseStamped>("/visualize_final_heading_angle", 1);
    publisher_visualize_obstacles =
      node->create_publisher<visualization_msgs::msg::Marker>("/visualize_obstacles", 1);
    //-------------------------Load parameters-------------------------//
    std::string param_file;
    rclcpp::Parameter param = node->get_parameter("drive_param");
    param_file = param.as_string();
    YAML::Node conf = YAML::LoadFile(param_file);
    float long_d = conf["long_distance"].as<float>();
    float middle_d = conf["middle_distance"].as<float>();
    float short_d = conf["short_distance"].as<float>();
    float high_s = conf["high_speed"].as<float>();
    float middle_s = conf["middle_speed"].as<float>();
    float low_s = conf["low_speed"].as<float>();
    float max_throttle = conf["max_throttle"].as<float>();
    drive_controller =
      DriveControl(max_throttle, long_d, middle_d, short_d, high_s, middle_s, low_s);
    //----------------------------------------------------------------//
    rclcpp::Rate r(40);
    tf2_ros::Buffer tfBuffer(node->get_clock());
    tf2_ros::TransformListener listener(tfBuffer);
    tf2::Stamped<tf2::Transform> transform;
    geometry_msgs::msg::Twist twist;
    geometry_msgs::msg::TransformStamped geoTransform;
    while (rclcpp::ok()) {
        rclcpp::spin_some(node);
        r.sleep();
    }
}

void lookupTwist(const tf2_ros::Buffer& tfBuffer,
                 const std::string& tracking_frame,
                 const std::string& observation_frame,
                 const rclcpp::Time& time,
                 const rclcpp::Duration& averaging_interval,
                 geometry_msgs::msg::Twist::SharedPtr twist)
{
    // ref point is origin of tracking_frame, ref_frame = obs_frame
    lookupTwist(tfBuffer,
                tracking_frame,
                observation_frame,
                observation_frame,
                tracking_frame,
                time,
                averaging_interval,
                twist);
};

void lookupTwist(const tf2_ros::Buffer& tfBuffer,
                 const std::string& tracking_frame,
                 const std::string& observation_frame,
                 const std::string& reference_frame,
                 const std::string& reference_point_frame,
                 const rclcpp::Time& time,
                 const rclcpp::Duration& averaging_interval,
                 geometry_msgs::msg::Twist::SharedPtr twist)
{
    rclcpp::Time target_time, latest_time;
    tf2::TimePoint latest_time_point;
    tfBuffer._getLatestCommonTime(tfBuffer._lookupFrameNumber(observation_frame),
                                  tfBuffer._lookupFrameNumber(tracking_frame),
                                  latest_time_point,
                                  NULL); ///\TODO check time on reference point too
    latest_time = tf2_ros::toRclcpp(latest_time_point);
    if (rclcpp::Time() == time)
        target_time = latest_time;
    else
        target_time = time;
    rclcpp::Time end_time = std::min(target_time + averaging_interval * 0.5, latest_time);
    rclcpp::Time start_time = std::max(rclcpp::Time(10000) + averaging_interval, end_time) -
                              averaging_interval; // don't collide with zero
    rclcpp::Duration corrected_averaging_interval =
      end_time - start_time; // correct for the possiblity that start time was truncated above.
    tf2::Stamped<tf2::Transform> start, end;
    geometry_msgs::msg::TransformStamped geoStart, geoEnd;
    geoStart = tfBuffer.lookupTransform(observation_frame, tracking_frame, start_time);
    geoEnd = tfBuffer.lookupTransform(observation_frame, tracking_frame, end_time);
    tf2::fromMsg(geoStart, start);
    tf2::fromMsg(geoEnd, end);
    tf2::Matrix3x3 temp = start.getBasis().inverse() * end.getBasis();
    tf2::Quaternion quat_temp;
    temp.getRotation(quat_temp);
    tf2::Vector3 o = start.getBasis() * quat_temp.getAxis();
    tf2Scalar ang = quat_temp.getAngle();
    double delta_x = end.getOrigin().getX() - start.getOrigin().getX();
    double delta_y = end.getOrigin().getY() - start.getOrigin().getY();
    double delta_z = end.getOrigin().getZ() - start.getOrigin().getZ();
    auto corrected_averaging_seconds = 1e-9 * corrected_averaging_interval.nanoseconds();
    tf2::Vector3 twist_vel((delta_x) / corrected_averaging_seconds,
                           (delta_y) / corrected_averaging_seconds,
                           (delta_z) / corrected_averaging_seconds);
    tf2::Vector3 twist_rot = o * (ang / corrected_averaging_seconds);
    // This is a twist w/ reference frame in observation_frame  and reference point is in the
    // tracking_frame at the origin (at start_time)
    // correct for the position of the reference frame
    tf2::Stamped<tf2::Transform> inverse;
    geometry_msgs::msg::TransformStamped geoInverse;
    geoInverse = tfBuffer.lookupTransform(reference_frame, tracking_frame, target_time);
    tf2::fromMsg(geoInverse, inverse);
    tf2::Vector3 out_rot = inverse.getBasis() * twist_rot;
    tf2::Vector3 out_vel = inverse.getBasis() * twist_vel + inverse.getOrigin().cross(out_rot);
    twist->linear.x = out_vel.x();
    twist->linear.y = out_vel.y();
    twist->linear.z = out_vel.z();
    twist->angular.x = out_rot.x();
    twist->angular.y = out_rot.y();
    twist->angular.z = out_rot.z();
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    FollowTheGapNode ftg_node;
    ftg_node.start_with_speeds();
    return 0;
}