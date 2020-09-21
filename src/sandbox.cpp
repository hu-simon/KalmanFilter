/*
 * Name        : sandbox.cpp
 * Author      : Simon Hu
 * Version     :
 * Copyright   : 2020, all rights reserved
 * Description : Sandbox file for debugging the Kalman filter
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
    A << 1.0, dt, 0.0, 0.0, 1.0, dt, 0.0, 0.0, 1.0;
    C << 1.0, 0.0, 0.0;

    // Initialize covariance matrices
    Q << 0.05, 0.05, 0.0, 0.05, 0.05, 0.0, 0.0, 0.0, 0.0;
    R << 5.0;
    P << 0.1, 0.1, 0.1, 0.1, 10000.0, 10.0, 0.1, 10.0, 100.0;

    // Print the shape of the matrices
    /*
    std::cout << "A: \n" << A << std::endl;
    std::cout << "C: \n" << C << std::endl;
    std::cout << "Q: \n" << Q << std::endl;
    std::cout << "R: \n" << R << std::endl;
    std::cout << "P: \n" << P << std::endl;
    */
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

    // Instantiate the Kalman filter
    KalmanFilter kf(A, C, Q, R, P, dt);

    // Instantiate an initial position
    Eigen::VectorXd x0(n_states);
    x0 << 1.04202710058, 0.0, -9.81;
    kf.init(x0, dt);

    // Print the current state and the sizes of the vectors
    std::cout << "x0.shape: (" << x0.rows() << ", " << x0.cols() << ")"
              << std::endl;

    // Test some of the shapes of the matrix multiplication
    std::cout << "A * x0.shape: (" << (A * x0).rows() << ", " << (A * x0).cols()
              << ")" << std::endl;
}