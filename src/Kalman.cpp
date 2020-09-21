// THIS FILE IS DEPRECATED IN FAVOR OF MULTIPLE CPP FILES

/*
 * Name        : Kalman.cpp
 * Author      : Simon Hu
 * Version     :
 * Copyright   : 2020, all rights reserved
 * Description : C++ file containing the logic for the Kalman filter
 */

#include <iostream>
#include <stdexcept>

#include "Kalman.h"

// Constructors
KalmanFilter::KalmanFilter() {}
KalmanFilter::KalmanFilter(const Eigen::MatrixXd &A, const Eigen::MatrixXd &B,
                           const Eigen::MatrixXd &C, const Eigen::MatrixXd &Q,
                           const Eigen::MatrixXd &R, const Eigen::MatrixXd &P,
                           double dt)
    : A(A), B(B), C(C), Q(Q), R(R), P0(P), m(C.rows()), n(A.rows()), dt(dt),
      is_initialized(false), I(n, n), x_hat(n), x_hat_new(n), u(n) {
    I.setIdentity(n, n);
}

// Destructors
KalmanFilter::~KalmanFilter() {}

// Process methods
void KalmanFilter::init() {
    x_hat.setZero();
    u.setZero();
    P = P0;
    t0 = 0.0;
    t = t0;
    is_initialized = true;
}

void KalmanFilter::init(const Eigen::VectorXd &x0, double t0) {
    x_hat = x0;
    u.setZero();
    P = P0;
    this->t0 = t0;
    t = t0;
    is_initialized = true;
}

void KalmanFilter::init(const Eigen::VectorXd &x0, const Eigen::VectorXd &u,
                        double t0) {
    x_hat = x0;
    this->u = u;
    P = P0;
    this->t0 = t0;
    t = t0;
    is_initialized = true;
}

void KalmanFilter::update(const Eigen::VectorXd &u, const Eigen::VectorXd &y) {
    if (!is_initialized) {
        throw std::runtime_error("Kalman filter is not initialized!");
    }
    x_hat_new = A * x_hat + B * u;
    P = A * P * A.transpose() + Q;
    K = P * C.transpose() * (C * P * C.transpose() + R).inverse();
    x_hat_new += K * (y - C * x_hat_new);
    P = (I - K * C) * P;
    x_hat = x_hat_new;

    t += dt;
}
void KalmanFilter::update(const Eigen::VectorXd &u, const Eigen::VectorXd &y,
                          double dt) {
    this->dt = dt;
    this->update(u, y);
}