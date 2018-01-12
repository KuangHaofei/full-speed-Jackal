//
// Created by kuang on 17-12-10.
//

#ifndef MPC_CAR_DUB_CAR_OCP_H
#define MPC_CAR_DUB_CAR_OCP_H

#include <nlopt.hpp>

#include <iostream>
#include <vector>
#include <cmath>
#include <stdarg.h>
#include <stdio.h>

using namespace std;

class Dub_Car_Ocp {
private:

    // Analytic solution of unicycle model
    // @param x_in:const vector<double> &   current robot state
    // @param delta_t:double  time interval
    // return:double Next robot position state(x, y, theta) + current velocity state(v, w)
    double ocp_car_d_x(const vector<double> &x_in, double delta_t);
    double ocp_car_d_y(const vector<double> &x_in, double delta_t);
    double ocp_car_d_theta(const vector<double> &x_in, double delta_t);

    // variables

    // initial position
    double start_x;
    double start_y;
    double start_theta;

    // expect destination
    double end_x;
    double end_y;

    // time
    double delta_t;

    int count;

    static Dub_Car_Ocp* pthis;

public:
    Dub_Car_Ocp(double end_x, double end_y,
                double delta_t);

    ~Dub_Car_Ocp();

    vector<double> executeOCP(double strat_x, double start_y, double start_theta);

    void setStartPosition(double start_x, double start_y, double start_theta);

    // methods

    // cost function
    static double costFunc(const vector<double> &x, vector<double> &grad, void *cost_func_data);

    // constraint
    static double xStartConstraint(const vector<double> &x, vector<double> &grad, void *data);
    static double yStartConstraint(const vector<double> &x, vector<double> &grad, void *data);
    static double thetaStartConstraint(const vector<double> &x, vector<double> &grad, void *data);

    static double xConstraint(const vector<double> &x, vector<double> &grad, void *data);

    static double yConstraint(const vector<double> &x, vector<double> &grad, void *data);

    static double thetaConstraint(const vector<double> &x, vector<double> &grad, void *data);

    static double obstacleConstrain(const vector<double> &x, vector<double> &grad, void *data);


    static double testConstraint(const vector<double> &x, vector<double> &grad, void *data);
};


#endif //MPC_CAR_DUB_CAR_OCP_H
