//
// Created by kuang on 17-11-30.
//
#include "../include/mpc/Robot.h"

Robot::Robot(ros::NodeHandle &n) : laser_beam(180) {
    this->n = n;

    this->Kp = 10.0;
    this->Ki = 0.1;
    this->Kd = 0.05;

    this->E_k = 0.0;
    this->e_k_1 = 0.0;

    this->reached = false;

    this->size = 0;
}

void Robot::init() {
    odom_sub = n.subscribe("/odometry/filtered", 100, &Robot::GetOdomCallback, this);
    laser_sub = n.subscribe("/velodyne_scan", 100, &Robot::GetLaserCallback, this);
    cmd_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 100);
}

void Robot::GetOdomCallback(const nav_msgs::Odometry::ConstPtr &odom_msg) {
    // Getting current position
    this->current_position.x = odom_msg->pose.pose.position.x;
    this->current_position.y = odom_msg->pose.pose.position.y;

    tf::Quaternion q(odom_msg->pose.pose.orientation.x, odom_msg->pose.pose.orientation.y,
                     odom_msg->pose.pose.orientation.z, odom_msg->pose.pose.orientation.w);

    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    this->current_position.theta = yaw;
}

void Robot::GetLaserCallback(const sensor_msgs::LaserScan::ConstPtr &laser_msg) {
    double value = 0.0;
    const double angle_min = -3.14159989357 / 2;
    const double angle_increment = 0.017453292519943295;
//    int count = 0;

    for (int i = 0; i < laser_msg->ranges.size(); ++i) {
//        if (i == 0 || i == 45 || i == 90 || i == 135 || i == 179){
//            if (laser_msg->ranges[i] > 2)
//            {
//                value = 2;
//            } else {
//                value = laser_msg->ranges[i];
//            }
//            this->laser_beam[count].range = value;
//            this->laser_beam[count].theta_beam = angle_min + angle_increment * i;
//            count++;
//        }
        if (laser_msg->ranges[i] > 3) {
            value = 3;
        } else {
            value = laser_msg->ranges[i];
        }
        this->laser_beam[i].range = value;
        this->laser_beam[i].theta_beam = angle_min + angle_increment * i;
    }

//    this->size = laser_msg->ranges.size();
    this->size = this->laser_beam.size();
}


tf::Transform Robot::get_transformation_matrix(double x, double y, double z, double theta) {
    tf::Transform T;

    T.setOrigin(tf::Vector3(x, y, z));
    tf::Quaternion q;
    q.setRPY(0.0, 0.0, theta);
    T.setRotation(q);

    return T;
}

Vector2D Robot::GetAvoidVector(double x, double y, double z) {
    tf::Vector3 robot_laser_point;
    tf::Vector3 global_laser_point;
    Vector2D sum_vector;
    sum_vector.x = 0.0;
    sum_vector.y = 0.0;
//    double laser_gain[180] = {1, 1, 3, 1, 1};

    // define transform matrix
    tf::Transform R_T_beam;
    tf::Transform W_T_R;
    double x_s = x;
    double y_s = y;
    double z_s = z;
    double theta_s;

    for (int i = 0; i < this->laser_beam.size(); ++i) {
        theta_s = this->laser_beam[i].theta_beam;

        R_T_beam = get_transformation_matrix(x_s, y_s, z_s, theta_s);

        tf::Vector3 laser_point(this->laser_beam[i].range, 0.0, 0.0);

        robot_laser_point = R_T_beam * laser_point;

        W_T_R = get_transformation_matrix(current_position.x, current_position.y, 0.0, current_position.theta);
        global_laser_point = W_T_R * robot_laser_point;

        if ( i == 89 ) {
            sum_vector.x += (global_laser_point.getX() - current_position.x) * 3; //* laser_gain[i];
            sum_vector.y += (global_laser_point.getY() - current_position.y) * 3;
        } else {
            sum_vector.x += (global_laser_point.getX() - current_position.x); //* laser_gain[i];
            sum_vector.y += (global_laser_point.getY() - current_position.y); //* laser_gain[i];
        }
    }

    return sum_vector;
}

void Robot::PrintRobotState() {
    ROS_INFO("Current State: %f, %f, %f\n",
             current_position.x, current_position.y, current_position.theta);
    ROS_INFO("Size: %d\n", this->size);
}

void Robot::GoToAngle(double theta_d) {
    // initialize velocity
    double v = 1.5;
    double w = 0.0;

    // define angle error
    double e_k = theta_d - current_position.theta;
    e_k = atan2(sin(e_k), cos(e_k));

    w = this->Kp * e_k;

    this->twist.linear.x = v;
    this->twist.angular.z = w;

    cmd_pub.publish(twist);
}

void Robot::isReach(double x_d, double y_d, double tolerance) {
    double distance = sqrt( pow(current_position.x - x_d, 2) + pow(current_position.y - y_d, 2) );

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
    theta_d = atan2(y_d - current_position.y, x_d - current_position.x);

    // compute error
    e_k = theta_d - current_position.theta;
    e_k = atan2(sin(e_k), cos(e_k));

    // define PID
    e_p = e_k;
    e_i = this->E_k + e_k * delta_t;
    e_d = (e_k - e_k_1) * delta_t;

    w = this->Kp * e_p + this->Ki * e_i + this->Kd * e_d;

    this->E_k = e_i;
    this->e_k_1 = e_k;

    // publish velocity
    this->twist.linear.x = v;
    this->twist.angular.z = w;

    cmd_pub.publish(twist);
}

void Robot::AvoidObstacles(double delta_t) {
    double v = 1.5;
    double w = 0.0;
    double theta_d = 0.0;
    double e_k = 0.0;

    double e_p, e_i, e_d;

    Vector2D obstacles_vector;
    double x_s = 0.120;
    double y_s = 0.000;
    double z_s = 0.267;

    // get obstacles vector
    obstacles_vector = GetAvoidVector(x_s, y_s, z_s);

    // compute angle from robot to goal
    theta_d = atan2(obstacles_vector.y, obstacles_vector.x);
    ROS_INFO("theta_d: %f", theta_d);
    ROS_INFO("current theta: %f", current_position.theta);

    // compute error
    e_k = theta_d - current_position.theta;
    e_k = atan2(sin(e_k), cos(e_k));

    // define PID
    e_p = e_k;
    e_i = this->E_k + e_k * delta_t;
    e_d = (e_k - e_k_1) * delta_t;

    w = this->Kp * e_p + this->Ki * e_i + this->Kd * e_d;

    this->E_k = e_i;
    this->e_k_1 = e_k;

    // publish velocity
    this->twist.linear.x = v;
    this->twist.angular.z = w;

    cmd_pub.publish(twist);
}

void Robot::MPC_Model(double end_x, double end_y, double delta_t) {
    Dub_Car_Ocp ocp_once(end_x, end_y, delta_t);

    double v = 0.0;
    double w = 0.0;

    vector<double> control_signal = ocp_once.executeOCP(current_position.x,
                                                        current_position.y,
                                                        current_position.theta);

    v = control_signal[0];
    w = control_signal[1];

    this->twist.linear.x = v;
    this->twist.angular.z = w;

    cmd_pub.publish(twist);
}