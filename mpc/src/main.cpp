//
// Created by kuang on 17-12-28.
//
#include "../include/mpc/Robot.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "Robot");
    ros::NodeHandle n;

    Robot robot(n);

    robot.init();

    double theta_d = M_PI / 4;

    double f = 10;
    ros::Rate loop_rate(f);
    double delta_t = 1 / f;

    double x_d = 5.0;
    double y_d = 5.0;
    double d_stop = 0.1;

    while (ros::ok()){
//        robot.GoToAngle(theta_d);
        robot.isReach(x_d, y_d, d_stop);

        if (robot.reached) {
            ROS_INFO("Arrived!\n");
            break;
        } else {
            robot.GoToGoal(x_d, y_d, delta_t);

            ROS_INFO("Target position: %f, %f\n", x_d, y_d);
            robot.PrintRobotState();

            ros::spinOnce();
            loop_rate.sleep();
        }
    }

    return 0;
}