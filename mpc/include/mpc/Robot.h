//
// Created by kuang on 17-12-28.
//

#ifndef PROJECT_MOVEGOAL_H
#define PROJECT_MOVEGOAL_H

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include <cmath>
#include <iostream>

struct RobotState {
    double x_position;
    double y_position;
    double angular;
};

class Robot {
private:

    ros::NodeHandle n;
    ros::Subscriber odom_sub;
    ros::Publisher cmd_pub;
    geometry_msgs::Twist twist;
    RobotState current_position;

    // PID parameters
    // gains
    double Kp;
    double Ki;
    double Kd;

    // error
    double E_k;
    double e_k_1;

public:

    // stop
    bool reached;

    Robot(ros::NodeHandle &n);

    void init();

    void GetOdomCallback(const nav_msgs::Odometry::ConstPtr &odom_msg);

    void PrintRobotState();

    // steers the robot towards a angle with a constant velocity using PID
    void GoToAngle(double theta_d);

    void GoToGoal(double x_d, double y_d, double delta_t);

    void isReach(double x_d, double y_d, double tolerance);
};

#endif //PROJECT_MOVEGOAL_H
