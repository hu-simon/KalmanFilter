/*
 * Name        : ExtendedKalman.cpp
 * Author      : Simon Hu
 * Version     :
 * Copyright   : 2020, all rights reserved
 * Description : C++ file containing the logic for the extended Kalman filter
 *
 * This entire class needs some more thinking before we can move on with the
 * design. The reason for this is because we need some way to compute the motion
 * model.
 */

#include <iostream>
#include <stdexcept>

#include "Kalman.h"

// Constructors
ExtendedKalmanFilter::ExtendedKalmanFilter() {}
ExtendedKalmanFilter::ExtendedKalmanFilter(
    const Eigen::MatrixXd &A, const Eigen::MatrixXd &F,
    const Eigen::MatrixXd &B, const Eigen::MatrixXd &C,
    const Eigen::MatrixXd &H, const Eigen::MatrixXd &Q,
    const Eigen::MatrixXd &R, const Eigen::MatrixXd &P,
    const Eigen::MatrixXd &W, const Eigen::MatrixXd &V, double dt)
    : A(A), F(F), B(B), C(C), H(H), Q(Q), R(R), P0(P), W(W), V(V), m(C.rows()),
      n(A.rows()), dt(dt), is_initialized(false), I(n, n), x_hat(n),
      x_hat_new(n), u(n) {
    I.setIdentity(n, n);
}

// Destructors
ExtendedKalmanFilter::~ExtendedKalmanFilter() {}

// Process methods
void ExtendedKalmanFilter::init() {
    x_hat.setZero();
    u.setZero();
    P = P0;
    t0 = 0.0;
    t = t0;
    is_initialized = true;
}

void ExtendedKalmanFilter::init(const Eigen::VectorXd &x0, double dt) {
    x_hat = x0;
    u.setZero();
    P = P0;
    this->t0 = t0;
    t = t0;
    is_initialized = true;
}

void ExtendedKalmanFilter::predict() {
    if (!is_initialized) {
        throw std::runtime_error("Kalman filter is not initialized!");
    }

    x_hat_new = ;
}