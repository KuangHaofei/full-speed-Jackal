//
// Created by kuang on 17-11-30.
//
#include <tf/LinearMath/Transform.h>
#include <tf/transform_datatypes.h>
#include "../include/mpc/Robot.h"

Robot::Robot(ros::NodeHandle &n) {
    this->n = n;

    this->Kp = 2.0;
    this->Ki = 10.0;
    this->Kd = 0.01;

    this->E_k = 0.0;
    this->e_k_1 = 0.0;

    this->reached = false;
}

void Robot::init() {
    odom_sub = n.subscribe("/odometry/filtered", 100, &Robot::GetOdomCallback, this);
    cmd_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 100);
}

void Robot::GetOdomCallback(const nav_msgs::Odometry::ConstPtr &odom_msg) {
    // Getting current position
    this->current_position.x_position = odom_msg->pose.pose.position.x;
    this->current_position.y_position = odom_msg->pose.pose.position.y;

    tf::Quaternion q(odom_msg->pose.pose.orientation.x, odom_msg->pose.pose.orientation.y,
                     odom_msg->pose.pose.orientation.z, odom_msg->pose.pose.orientation.w);

    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    this->current_position.angular = yaw;
}

void Robot::PrintRobotState() {
    ROS_INFO("Current State: %f, %f, %f\n", current_position.x_position, current_position.y_position, current_position.angular);
}

void Robot::GoToAngle(double theta_d) {
    // initialize velocity
    double v = 1.5;
    double w = 0.0;

    // define angle error
    double e_k = theta_d - current_position.angular;
    e_k = atan2(sin(e_k), cos(e_k));

    w = this->Kp * e_k;

    this->twist.linear.x = v;
    this->twist.angular.z = w;

    cmd_pub.publish(twist);
}

void Robot::isReach(double x_d, double y_d, double tolerance) {
    double distance = sqrt( pow(current_position.x_position - x_d, 2) + pow(current_position.y_position - y_d, 2) );

    if (distance < tolerance) {
        this->reached = true;
    }
}

void Robot::GoToGoal(double x_d, double y_d, double delta_t) {
    double v = 1.0;
    double w = 0.0;
    double theta_d = 0.0;
    double e_k = 0.0;

    double e_p, e_i, e_d;

    // compute angle from robot to goal
    theta_d = atan2(y_d - current_position.y_position, x_d - current_position.x_position);

    // compute error
    e_k = theta_d - current_position.angular;
    e_k = atan2(sin(e_k), cos(e_k));

    // define PID
    e_p = e_k;
    e_i = (this->E_k + e_k) * delta_t;
    e_d = (e_k - e_k_1) * delta_t;

    w = this->Kp * e_p + this->Ki * e_i + this->Kd * e_d;

    this->E_k = e_i;
    this->e_k_1 = e_k;

    // publish velocity
    this->twist.linear.x = v;
    this->twist.angular.z = w;

    cmd_pub.publish(twist);
}