//
// Created by kuang on 17-11-30.
//
#include "../include/mpc/Robot.h"

Robot::Robot(ros::NodeHandle &n) : laser_beam(180) {
    this->n = n;

    this->KGp = 8.0;
    this->KGi = 0.1;
    this->KGd = 0.01;

    this->KAp = 8.0;
    this->KAi = 0.1;
    this->KAd = 0.05;

    this->KAGp = 8.0;
    this->KAGi = 0.1;
    this->KAGd = 0.05;


    this->E_A_k = 0.0;
    this->e_a_k_1 = 0.0;

    this->E_AG_k = 0.0;
    this->e_a_k_1 = 0.0;

    this->E_A_k = 0.0;
    this->e_a_k_1 = 0.0;

    this->destination.x = 0.0;
    this->destination.y = 0.0;

    this->delta_t = 0.0;

    this->current_state = "AO_and_GTG";

    this->reach_tolerance = 0.2;
    this->at_obstacle_tolerance = 1.5;
    this->unsafe_tolerance = 1;
    this->dangerous_tolerance = 0.7;
    this->closetogoal_tolerance = 2;

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

void Robot::PublishTist(ControlSignal control_signal) {
    control_signal.w = max(control_signal.w, -1.0);
    control_signal.w = min(control_signal.w, 1.0);

    control_signal.v = min(control_signal.v, 1.5);

    // Print velocity
    ROS_INFO("Velocity: [%f, %f]", control_signal.v, control_signal.w);

    // publish velocity
    this->twist.linear.x = control_signal.v;
    this->twist.angular.z = control_signal.w;

    this->cmd_pub.publish(this->twist);
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

        if (i > 84 && i < 95) {
            sum_vector.x += (global_laser_point.getX() - current_position.x) * 0.5; //* laser_gain[i];
            sum_vector.y += (global_laser_point.getY() - current_position.y) * 0.5;
        } else {
            sum_vector.x += (global_laser_point.getX() - current_position.x); //* laser_gain[i];
            sum_vector.y += (global_laser_point.getY() - current_position.y); //* laser_gain[i];
        }
    }

    return sum_vector;
}

void Robot::PrintRobotState() {
    ROS_INFO("Current State: Position [%f, %f, %f]",
             current_position.x, current_position.y, current_position.theta);
    ROS_INFO("Stata: %s", this->current_state.c_str());
//    ROS_INFO("Size: %d\n", this->size);
}

void Robot::SetDestination(double x, double y) {
    this->destination.x = x;
    this->destination.y = y;
}

void Robot::Set_delta_t(double delta_t) {
    this->delta_t = delta_t;
}

void Robot::GoToAngle(double theta_d) {
    // initialize velocity
    double v = 1.5;
    double w = 0.0;
    double Kp = 1;

    // define angle error
    double e_k = theta_d - current_position.theta;
    e_k = atan2(sin(e_k), cos(e_k));

    w = Kp * e_k;

    this->twist.linear.x = v;
    this->twist.angular.z = w;

    cmd_pub.publish(twist);
}

bool Robot::isReach() {
    double distance = sqrt(pow(current_position.x - this->destination.x, 2)
                           + pow(current_position.y - this->destination.y, 2));
    bool reach = false;

    if (distance < this->reach_tolerance) {
        ROS_INFO("At Goal!");
        reach = true;
    }

    return reach;
}

bool Robot::atObstacle() {
    double count = laser_beam.size();

    for (int i = 0; i < count; i++) {
        if (laser_beam[i].range < this->at_obstacle_tolerance) {
            ROS_INFO("At Obstacle!");
            return true;
        }
    }
    return false;
}

bool Robot::unSafe() {
    double count = laser_beam.size();

    for (int i = 0; i < count; i++) {
        if (laser_beam[i].range < this->unsafe_tolerance) {
            ROS_INFO("Unsafe!!!");
            return true;
        }
    }
    return false;
}

bool Robot::obstacleCleared() {
    double count = laser_beam.size();

    for (int i = 0; i < count; i++) {
        if (laser_beam[i].range < this->at_obstacle_tolerance) {
            return false;
        }
    }
    ROS_INFO("Obstacle Cleared! Safe!");
    return true;
}


bool Robot::dangerous() {
    double count = laser_beam.size();
    int num = 0;

    for (int i = 60; i < 120; i++) {
        if (laser_beam[i].range < this->dangerous_tolerance) {
            num++;
            if (num > 2){
                ROS_INFO("Dangerous! Stop!");
                return true;
            }
        }
    }
    return false;
}

bool Robot::closeToGoal() {
    double distance = sqrt(pow(current_position.x - this->destination.x, 2)
                           + pow(current_position.y - this->destination.y, 2));

    bool close = false;

    if (distance < this->closetogoal_tolerance) {
        close = true;
    }

    return close;
}


double Robot::smallest_distance() {
    double count = laser_beam.size();
    double smallest_num = INFINITY;

    for (int i = 60; i < 120; i++) {
        if (laser_beam[i].range < smallest_num) {
            smallest_num = laser_beam[i].range;
        }
    }
    return smallest_num;
}

bool Robot::check_event(string state) {
    if (state == "at_goal") {
        return isReach();
    } else if (state == "at_obstacle") {
        return atObstacle();
    } else if (state == "unsafe") {
        return unSafe();
    } else if (state == "obstacle_cleared") {
        return obstacleCleared();
    } else if (state == "dangerous") {
        return dangerous();
    } else if (state == "close_to_goal") {
        return closeToGoal();
    } else {
        ROS_INFO("Error Event! Please Check!");
        return false;
    }
}

void Robot::switchState(string state) {
    ROS_INFO("Change State to %s!", state.c_str());
    this->current_state = state;
}


ControlSignal Robot::GoToGoal() {
    ControlSignal control_signal;
    double v = 0.2;
    double w = 0.0;
    double theta_d = 0.0;
    double e_k = 0.0;

    double e_p, e_i, e_d;

    // compute angle from robot to goal
    theta_d = atan2(this->destination.y - current_position.y, this->destination.x - current_position.x);

    // compute error
    e_k = theta_d - current_position.theta;
    e_k = atan2(sin(e_k), cos(e_k));

    // define PID
    e_p = e_k;
    e_i = this->E_G_k + e_k * delta_t;
    e_d = (e_k - this->e_g_k_1) * delta_t;

    w = this->KGp * e_p + this->KGi * e_i + this->KGd * e_d;

    this->E_G_k = e_i;
    this->e_g_k_1 = e_k;

    // linear velocity restrict
    double gain_d = 1;
    double gain_w = 0.25;
    double distance = sqrt(pow(current_position.x - this->destination.x, 2)
                           + pow(current_position.y - this->destination.y, 2));
    v = v + gain_d * distance;
    v = min(v, 1.5);
    v = v - fabs(gain_w * w);
    v = max(v, 0.0);

    control_signal.v = v;
    control_signal.w = w;

    return control_signal;
}

ControlSignal Robot::AvoidObstacles() {
    ControlSignal control_signal;
    double v = smallest_distance() / 1.5;
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
    e_i = this->E_A_k + e_k * this->delta_t;
    e_d = (e_k - this->e_a_k_1) * this->delta_t;

    w = this->KAp * e_p + this->KAi * e_i + this->KAd * e_d;

    this->E_A_k = e_i;
    this->e_a_k_1 = e_k;

    // linear velocity restrict
    double gain_w = 0.05;
    v = max(v - fabs(gain_w * w), 0.0);

    if (dangerous()){
        v = 0.0;
        w = w * 10;
    }

    control_signal.v = v;
    control_signal.w = w;

    return control_signal;
}

ControlSignal Robot::AOandGTG() {
    ControlSignal control_signal;
    double v = 0.5;
    double w = 0.0;
    double theta_d = 0.0;
    double e_k = 0.0;

    double e_p, e_i, e_d;

    Vector2D u_ao;
    Vector2D u_gtg;
    Vector2D u_ao_gtg;
    double x_s = 0.120;
    double y_s = 0.000;
    double z_s = 0.267;

    // get obstacles vector
    u_ao = GetAvoidVector(x_s, y_s, z_s);

    // compute angle from robot to goal
    u_gtg.x = destination.x - current_position.x;
    u_gtg.y = destination.y - current_position.y;

    double alpha = 0.8;
    u_ao_gtg.x = alpha * u_gtg.x + (1 - alpha) * u_ao.x;
    u_ao_gtg.y = alpha * u_gtg.y + (1 - alpha) * u_ao.y;

    theta_d = atan2(u_ao_gtg.y, u_ao_gtg.x);
    ROS_INFO("theta_d: %f", theta_d);

    // compute error
    e_k = theta_d - current_position.theta;
    e_k = atan2(sin(e_k), cos(e_k));

    // define PID
    e_p = e_k;
    e_i = this->E_AG_k + e_k * this->delta_t;
    e_d = (e_k - this->e_ag_k_1) * this->delta_t;

    w = this->KAGp * e_p + this->KAGi * e_i + this->KAGd * e_d;

    this->E_AG_k = e_i;
    this->e_ag_k_1 = e_k;

    // linear velocity restrict
    double gain_d = 1.0;
    double gain_w = 0.5;
    double distance = sqrt(pow(current_position.x - this->destination.x, 2)
                           + pow(current_position.y - this->destination.y, 2));
    v = v + gain_d * distance;
    v = min(v, 1.5);
    v = v - fabs(gain_w * w);
    v = max(v, 0.0);

    if (dangerous()){
        v = 0.0;
        w = w * 10;
    }

    control_signal.v = v;
    control_signal.w = w;

    return control_signal;
}

ControlSignal Robot::Stop() {
    ControlSignal control_signal;

    double v = 0.0;
    double w = 0.0;

    control_signal.v = v;
    control_signal.w = w;

    return control_signal;
}

void Robot::execute() {
    ControlSignal control_signal;
    control_signal.v = 0.0;
    control_signal.w = 0.0;

    if (check_event("at_goal")) {
        switchState("Stop");
    } else {
        if (check_event("unsafe")) {
            switchState("Avoid_Obstacle");
        } else if ( check_event("close_to_goal") && !check_event("dangerous")) {
            switchState("Go_to_Goal");
        } else if (check_event("at_obstacle")) {
            switchState("AO_and_GTG");
        } else if (check_event("obstacle_cleared")) {
            switchState("Go_to_Goal");
        }
    }

    if (this->current_state == "Stop") {
        control_signal = Stop();
    } else if (this->current_state == "AO_and_GTG") {
        control_signal = AOandGTG();
    } else if (this->current_state == "Avoid_Obstacle") {
        control_signal = AvoidObstacles();
    } else if (this->current_state == "Go_to_Goal") {
        control_signal = GoToGoal();
    }

    // Publish velocity
    PublishTist(control_signal);
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