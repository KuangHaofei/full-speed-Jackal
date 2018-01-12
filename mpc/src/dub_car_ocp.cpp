//
// Created by kuang on 17-12-10.
//

#include "../include/mpc/dub_car_ocp.h"

Dub_Car_Ocp *Dub_Car_Ocp::pthis = NULL;

Dub_Car_Ocp::Dub_Car_Ocp(double end_x, double end_y,
                         double delta_t) {
    this->start_x = 0;
    this->start_y = 0;
    this->start_theta = 0;

    this->end_x = end_x;
    this->end_y = end_y;

    this->delta_t = delta_t;
    this->count = 0;

    pthis = this;
}

Dub_Car_Ocp::~Dub_Car_Ocp() {}

void Dub_Car_Ocp::setStartPosition(double start_x, double start_y, double start_theta) {
    this->start_x = start_x;
    this->start_y = start_y;
    this->start_theta = start_theta;
}

vector<double> Dub_Car_Ocp::executeOCP(double strat_x, double start_y, double start_theta) {
    this->start_x = strat_x;
    this->start_y = start_y;
    this->start_theta = start_theta;

    vector<double> control_signal(2);

    unsigned int N = 6;
    unsigned int input_num = 5 * N + 3;

    nlopt::opt opt(nlopt::LN_COBYLA, input_num);

    opt.set_min_objective(costFunc, NULL);

    vector<double> lb(input_num);
    for (int i = 0; i < lb.size(); ++i) {
        lb[i] = -100;
    }
    for (int i = 0; i < N; ++i) {
        lb[i * 5 + 3] = 0.0;
        lb[i * 5 + 4] = -M_PI / 4;
    }

    vector<double> ub(input_num);
    for (int i = 0; i < ub.size(); ++i) {
        ub[i] = 100;
    }
    for (int i = 0; i < N; ++i) {
        ub[i * 5 + 3] = 1.5;
        ub[i * 5 + 4] = M_PI / 4;
    }

    opt.set_lower_bounds(lb);
    opt.set_upper_bounds(ub);

    vector<int> index_equality(N);
    vector<int> index_inequality(N+1);


    int data_init = 1;

    for (int i = 0; i < N; ++i) {
        index_equality[i] = i;

        opt.add_equality_constraint(xConstraint, &index_equality[i], 1e-4);
        opt.add_equality_constraint(yConstraint, &index_equality[i], 1e-4);
        opt.add_equality_constraint(thetaConstraint, &index_equality[i], 1e-4);
    }

//    for (int i = 0; i < N + 1; ++i) {
//        index_inequality[i] = i;
//        opt.add_inequality_constraint(obstacleConstrain, &index_inequality[i], 1e-4);
//    }

    opt.add_equality_constraint(xStartConstraint, &data_init, 1e-4);
    opt.add_equality_constraint(yStartConstraint, &data_init, 1e-4);
    opt.add_equality_constraint(thetaStartConstraint, &data_init, 1e-4);

    opt.set_xtol_rel(1e-1);

    // initial guess
    vector<double> x(input_num);
    for (int j = 0; j < x.size(); ++j) {
        x[j] = 0.0;
    }
//    x[0] = strat_x; x[1] = start_y; x[2] = start_theta;
//
//    x[5] = 0.73086; x[10] = 1.35047; x[15] = 1.84845; x[20] = 2.3084;
//    x[25] = 2.78649; x[30] = 3.30629; x[35] = 3.73027; x[40] = 3.79562;
//
//
//    x[6] = 0.145377; x[11] = 0.559388; x[16] = 1.11964; x[21] = 1.71206;
//    x[26] = 2.28967; x[31] = 2.83013; x[36] = 3.44297; x[41] = 3.72781;
//
//    x[7] = 0.392699; x[12] = 0.785398; x[17] = 0.902959; x[22] = 0.918276;
//    x[27] = 0.840522; x[32] = 0.769227; x[37] = 1.16193; x[42] = 1.52877;
//
//    x[3] = x[8] = x[13] = x[18] = x[23] = x[28] = x[33] = 1.5; x[38] = 0.587753;
//
//    x[4] = x[9] = 0.785398; x[14] = 0.235121; x[19] = 0.0306354; x[24] = -0.155509; x[29] = -0.14259;
//    x[34] = 0.785398; x[39] = 0.733687;

    // result
    double minf = 0.0;
    nlopt::result result = opt.optimize(x, minf);

    if (result < 0) {
        cout << "nlopt failed!\n";
    } else {
        cout << "The type of result: " << result << endl;
        cout << "solver time: " << this->count <<endl;
        printf("found minimum %0.10g\n", minf);

        for (int i = 0; i < N + 1; ++i) {
            cout << x[5 * i] << ", " << x[1 + 5 * i] << ", " << x[2 + 5 * i] << ", " << x[3 + 5 * i] << ", " << x[4 + 5 * i] << endl;
        }
    }

    control_signal[0] = x[3];
    control_signal[1] = x[4];

    return control_signal;
}

double Dub_Car_Ocp::costFunc(const vector<double> &x, vector<double> &grad, void *cost_func_data) {
    int offset = 5;
    double result = 0.0;
    if (!grad.empty()) {
        for (int i = 0; i < x.size() / offset + 1; ++i) {
            grad[i * offset] = 2 * (x[i * offset] - 5.0);
            grad[1 + i * offset] = 2 * (x[1 + i * offset] - 5.0);
        }
    }

    for (int i = 0; i < x.size() / offset + 1; ++i) {
        result += pow(x[i * offset] - pthis->end_x, 2) + pow(x[1 + i * offset] - pthis->end_y, 2);
    }

    pthis->count++;
    return result;
}

double Dub_Car_Ocp::xStartConstraint(const vector<double> &x, vector<double> &grad, void *data) {
    int *con_data = reinterpret_cast<int *>(data);

    if (!grad.empty()) {
        grad[0] = *con_data;
    }

    return (x[0] - pthis->start_x);
}

double Dub_Car_Ocp::yStartConstraint(const vector<double> &x, vector<double> &grad, void *data) {
    int *con_data = reinterpret_cast<int *>(data);

    if (!grad.empty()) {
        grad[1] = *con_data;
    }

    return (x[1] - pthis->start_y);
}

double Dub_Car_Ocp::thetaStartConstraint(const vector<double> &x, vector<double> &grad, void *data) {
    int *con_data = reinterpret_cast<int *>(data);

    if (!grad.empty()) {
        grad[2] = *con_data;
    }

    return (x[2] - pthis->start_theta);
}


double Dub_Car_Ocp::xConstraint(const vector<double> &x, vector<double> &grad, void *data) {
    int *offset = reinterpret_cast<int *>(data);
    double new_x = 0.0;

//    if (!grad.empty()){
//        grad[*offset * 5] = x[3 + *offset * 5] * cos();
//    }

    vector<double> state(5);
    state[0] = x[*offset * 5];
    state[1] = x[1 + *offset * 5];
    state[2] = x[2 + *offset * 5];
    state[3] = x[3 + *offset * 5];
    state[4] = x[4 + *offset * 5];

//    vector<double> new_state = pthis->ocp_car(state, pthis->t_end, pthis->n_point);
    new_x = pthis->ocp_car_d_x(state, pthis->delta_t);

    return (x[(*offset + 1) * 5] - new_x);
}

double Dub_Car_Ocp::yConstraint(const vector<double> &x, vector<double> &grad, void *data) {
    int *offset = reinterpret_cast<int *>(data);
    double new_y = 0.0;

//    if (!grad.empty()){
//        grad[*offset * 5] = x[3 + *offset * 5] * cos();
//    }

    vector<double> state(5);
    state[0] = x[*offset * 5];
    state[1] = x[1 + *offset * 5];
    state[2] = x[2 + *offset * 5];
    state[3] = x[3 + *offset * 5];
    state[4] = x[4 + *offset * 5];

//    vector<double> new_state = pthis->ocp_car(state, pthis->t_end, pthis->n_point);
    new_y = pthis->ocp_car_d_y(state, pthis->delta_t);

    return (x[1 + (*offset + 1) * 5] - new_y);
}

double Dub_Car_Ocp::thetaConstraint(const vector<double> &x, vector<double> &grad, void *data) {
    int *offset = reinterpret_cast<int *>(data);
    double new_theta = 0.0;

//    if (!grad.empty()){
//        grad[*offset * 5] = x[3 + *offset * 5] * cos();
//    }

    vector<double> state(5);
    state[0] = x[*offset * 5];
    state[1] = x[1 + *offset * 5];
    state[2] = x[2 + *offset * 5];
    state[3] = x[3 + *offset * 5];
    state[4] = x[4 + *offset * 5];

//    vector<double> new_state = pthis->ocp_car(state, pthis->t_end, pthis->n_point);

    new_theta = pthis->ocp_car_d_theta(state, pthis->delta_t);

    return (x[2 + (*offset + 1) * 5] - new_theta);
}

double Dub_Car_Ocp::obstacleConstrain(const vector<double> &x, vector<double> &grad, void *data) {
    int *offset = reinterpret_cast<int *>(data);
    double result = 0.0;

    result = 2 - min( fabs(x[(*offset) * 5] - 3), fabs(x[(*offset) * 5 + 1] - 4));

    return result;
}

double Dub_Car_Ocp::testConstraint(const vector<double> &x, vector<double> &d, void *data) {


    return (x[0] - 100);
}

double Dub_Car_Ocp::ocp_car_d_x(const vector<double> &x_in, double delta_t) {
    double result = 0.0;
    double theta = x_in[2];
    double v = x_in[3];
    double w = x_in[4];

    if (w == 0){
        result = x_in[0] + v * cos(theta) * delta_t;
    } else {
        result = (v / w) * sin( w * delta_t + theta) - v * sin(theta) / w + x_in[0];
    }

    return result;
}

double Dub_Car_Ocp::ocp_car_d_y(const vector<double> &x_in, double delta_t) {
    double result = 0.0;
    double theta = x_in[2];
    double v = x_in[3];
    double w = x_in[4];

    if (w == 0){
        result = x_in[1] + v * sin(theta) * delta_t;
    } else {
        result = (v / w) * (cos(theta) - cos( w * delta_t + theta) ) + x_in[1];
    }

    return result;
}


double Dub_Car_Ocp::ocp_car_d_theta(const vector<double> &x_in, double delta_t) {
    double result = 0.0;
    double theta = x_in[2];
    double w = x_in[4];

    if (w == 0){
        result = theta;
    } else {
        result = w * delta_t + theta;
    }

    return result;
}