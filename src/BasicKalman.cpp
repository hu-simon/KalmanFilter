/*
 * Name        : BasicKalman.cpp
 * Author      : Simon Hu
 * Version     :
 * Copyright   : 2020, all rights reserved
 * Description : C++ file containing the logic for the basic Kalman filter
 */

#include <iostream>
#include <stdexcept>

#include "Kalman.h"

// Constructors
BasicKalmanFilter::BasicKalmanFilter() {}
BasicKalmanFilter::BasicKalmanFilter(const Eigen::MatrixXd &A,
                                     const Eigen::MatrixXd &C,
                                     const Eigen::MatrixXd &Q,
                                     const Eigen::MatrixXd &R,
                                     const Eigen::MatrixXd &P, double dt)
    : A(A), C(C), Q(Q), R(R), P0(P), m(C.rows()), n(A.rows()), dt(dt),
      is_initialized(false), I(n, n), x_hat(n), x_hat_new(n) {
    I.setIdentity(n, n);
}

// Destructors
BasicKalmanFilter::~BasicKalmanFilter() {}

// Process methods
void BasicKalmanFilter::init() {
    x_hat.setZero();
    P = P0;
    t0 = 0.0;
    t = t0;
    is_initialized = true;
}

void BasicKalmanFilter::init(const Eigen::VectorXd &x0, double t0) {
    x_hat = x0;
    P = P0;
    this->t0 = t0;
    t = t0;
    is_initialized = true;
}

void BasicKalmanFilter::update(const Eigen::VectorXd &y) {
    if (!is_initialized) {
        throw std::runtime_error("Kalman filter is not initialized!");
    }
    x_hat_new = A * x_hat;
    P = A * P * A.transpose() + Q;
    K = P * C.transpose() * (C * P * C.transpose() + R).inverse();
    x_hat_new += K * (y - C * x_hat_new);
    P = (I - K * C) * P;
    x_hat = x_hat_new;

    t += dt;
}

void BasicKalmanFilter::update(const Eigen::VectorXd &y, double dt) {
    this->dt = dt;
    this->update(y);
}