//
// Created by kuang on 17-12-28.
//

#ifndef PROJECT_MOVEGOAL_H
#define PROJECT_MOVEGOAL_H

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose2D.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/LaserScan.h"
#include "dub_car_ocp.h"
#include "tf/tf.h"
#include <tf/LinearMath/Transform.h>
#include <tf/transform_datatypes.h>
#include <cmath>
#include <iostream>

struct LaserData {
    double range;
    double theta_beam;
};

struct Vector2D {
    double x;
    double y;
};

class Robot {
private:

    // ROS variables
    ros::NodeHandle n;
    ros::Subscriber odom_sub;
    ros::Subscriber laser_sub;
    ros::Publisher cmd_pub;
    geometry_msgs::Twist twist;
    geometry_msgs::Pose2D current_position;
    vector<LaserData> laser_beam;
    unsigned long size;

    // PID parameters
    // gains
    double Kp;
    double Ki;
    double Kd;

    // error
    double E_k;
    double e_k_1;

    // member functions
    void GetOdomCallback(const nav_msgs::Odometry::ConstPtr &odom_msg);
    void GetLaserCallback(const sensor_msgs::LaserScan::ConstPtr &laser_msg);

    Vector2D GetAvoidVector(double x, double y, double z);
    tf::Transform get_transformation_matrix(double x, double y, double z, double theta);

public:

    // stop
    bool reached;

    Robot(ros::NodeHandle &n);

    void init();

    void PrintRobotState();

    // steers the robot towards a angle with a constant velocity using PID
    void GoToAngle(double theta_d);

    void GoToGoal(double x_d, double y_d, double delta_t);

    void AvoidObstacles(double delta_t);

    void isReach(double x_d, double y_d, double tolerance);

    void MPC_Model(double end_x, double end_y, double delta_t);
};

#endif //PROJECT_MOVEGOAL_H
