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
#include <string>

struct LaserData {
    double range;
    double theta_beam;
};

struct Vector2D {
    double x;
    double y;
};

struct ControlSignal{
    double v;
    double w;
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
    // for GotoGoal
    double KGp;
    double KGi;
    double KGd;

    // for Avoid Obstacle
    double KAp;
    double KAi;
    double KAd;

    // for AO_and_GtG
    double KAGp;
    double KAGi;
    double KAGd;


    // error
    // for GotoGoal
    double E_G_k;
    double e_g_k_1;

    // for Avoid Obstacle
    double E_A_k;
    double e_a_k_1;

    // for AO_and_GtG
    double E_AG_k;
    double e_ag_k_1;


    // member functions
    void GetOdomCallback(const nav_msgs::Odometry::ConstPtr &odom_msg);
    void GetLaserCallback(const sensor_msgs::LaserScan::ConstPtr &laser_msg);

    void PublishTist(ControlSignal control_signal);

    Vector2D GetAvoidVector(double x, double y, double z);
    tf::Transform get_transformation_matrix(double x, double y, double z, double theta);

    // Destination
    Vector2D destination;

    // delta time
    double delta_t;

    // Finite State Machine

    // current state
    // AO_and_GTG
    // Go_to_Goal
    // Avoid_Obstacle
    // Stop
    string current_state;

    // tolerance
    double  reach_tolerance;
    double at_obstacle_tolerance;
    double unsafe_tolerance;
    double dangerous_tolerance;

    // Guard function
    bool isReach();
    bool atObstacle();
    bool unSafe();
    bool obstacleCleared();
    bool dangerous();

    double smallest_distance();

    // check guard event
    // at_goal
    // at_obstacle
    // unsafe
    // obstacle_cleared
    bool check_event(string state);

    // switch state
    void switchState(string state);

public:

    Robot(ros::NodeHandle &n);

    void init();

    void PrintRobotState();

    void SetDestination(double x, double y);
    void Set_delta_t(double delta_t);

    // steers the robot towards a angle with a constant velocity using PID

    // State
    void GoToAngle(double theta_d);

    ControlSignal GoToGoal();

    ControlSignal AvoidObstacles();

    ControlSignal Stop();

    ControlSignal AOandGTG();

    // execute a navigation task
    void execute();

    void MPC_Model(double end_x, double end_y, double delta_t);
};

#endif //PROJECT_MOVEGOAL_H
