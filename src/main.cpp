/*
 * Name        : main.cpp
 * Author      : Simon Hu
 * Version     :
 * Copyright   : 2020, all rights reserved
 * Description : Main driver file for testing the Kalman filter
 */

#include <Eigen/Dense>
#include <iostream>
#include <vector>

#include "Kalman.h"

int main(int argc, char *argv[]) {
    // State parameters
    unsigned short int n_states = 3;
    unsigned short int n_measurements = 1;

    // Simulation parameters
    double dt = 1.0 / 30;

    // Process matrices
    Eigen::MatrixXd A(n_states, n_states);
    Eigen::MatrixXd C(n_measurements, n_states);
    Eigen::MatrixXd Q(n_states, n_states);
    Eigen::MatrixXd R(n_measurements, n_measurements);
    Eigen::MatrixXd P(n_states, n_states);

    // Initialize process matrices
    A << 1, dt, 0, 0, 1, dt, 0, 0, 1;
    C << 1, 0, 0;

    // Initialize covariance matrices
    Q << .05, .05, .0, .05, .05, .0, .0, .0, .0;
    R << 5;
    P << .1, .1, .1, .1, 10000, 10, .1, 10, 100;

    /*
    std::cout << "A: \n" << A << std::endl;
    std::cout << "C: \n" << C << std::endl;
    std::cout << "Q: \n" << Q << std::endl;
    std::cout << "R: \n" << R << std::endl;
    std::cout << "P: \n" << P << std::endl;

    std::cout << "A.shape: (" << A.rows() << ", " << A.cols() << ")"
              << std::endl;
    std::cout << "C.shape: (" << C.rows() << ", " << C.cols() << ")"
              << std::endl;
    std::cout << "Q.shape: (" << Q.rows() << ", " << Q.cols() << ")"
              << std::endl;
    std::cout << "R.shape: (" << R.rows() << ", " << R.cols() << ")"
              << std::endl;
    std::cout << "P.shape: (" << P.rows() << ", " << P.cols() << ")"
              << std::endl;
    */

    // Instantiate the Kalman filter
    BasicKalmanFilter kf(A, C, Q, R, P, dt);

    // Curate a list of noisy position measurements
    std::vector<double> noisy_measurements = {
        1.04202710058,  1.10726790452,  1.2913511148,    1.48485250951,
        1.72825901034,  1.74216489744,  2.11672039768,   2.14529225112,
        2.16029641405,  2.21269371128,  2.57709350237,   2.6682215744,
        2.51641839428,  2.76034056782,  2.88131780617,   2.88373786518,
        2.9448468727,   2.82866600131,  3.0006601946,    3.12920591669,
        2.858361783,    2.83808170354,  2.68975330958,   2.66533185589,
        2.81613499531,  2.81003612051,  2.88321849354,   2.69789264832,
        2.4342229249,   2.23464791825,  2.30278776224,   2.02069770395,
        1.94393985809,  1.82498398739,  1.52526230354,   1.86967808173,
        1.18073207847,  1.10729605087,  0.916168349913,  0.678547664519,
        0.562381751596, 0.355468474885, -0.155607486619, -0.287198661013,
        -0.602973173813};

    // Guess the initial state
    Eigen::VectorXd x0(n_states);
    x0 << noisy_measurements[0], 0, -9.81;
    kf.init(x0, dt);

    // Feed the measurements into the filter and output the estimated states
    double t = 0;
    Eigen::VectorXd y(n_measurements);
    std::cout << "t = " << t << ", "
              << "x_hat[0]: " << kf.get_state().transpose() << std::endl;
    for (unsigned int i = 0; i < noisy_measurements.size(); i++) {
        t += dt;
        y << noisy_measurements[i];
        kf.update(y);
        std::cout << "t = " << t << ", "
                  << "y[" << i << "] = " << y.transpose() << ", x_hat[" << i
                  << "] = " << kf.get_state().transpose() << std::endl;
    }

    return 0;
}