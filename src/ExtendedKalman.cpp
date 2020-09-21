/*
 * Name        : ExtendedKalman.cpp
 * Author      : Simon Hu
 * Version     :
 * Copyright   : 2020, all rights reserved
 * Description : C++ file containing the logic for the extended Kalman filter
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
    const Eigen::MatrixXd &R, const Eigen::MatrixXd &P, double dt)
    : A(A), F(F), B(B), C(C), H(H), Q(Q), R(R), P0(P), m(C.rows()), n(A.rows()),
      dt(dt), is_initialized(false), I(n, n), x_hat(n), x_hat_new(n), u(n) {
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
void ExtendedKalmanFilter::init(const Eigen::VectorXd &x0,
                                const Eigen::VectorXd &u, double t0) {
    x_hat = x0;
    this->u = u;
    P = P0;
    this->t0 = t0;
    t = t0;
    is_initialized = true;
}

void ExtendedKalmanFilter::update(const Eigen::VectorXd &u,
                                  const Eigen::VectorXd &y) {
    /*
     * The implementation is going to be like the previous ones, except there is
     * going to be some sort of derivative term.
     */
}

void ExtendedKalmanFilter::update(const Eigen::VectorXd &A,
                                  const Eigen::VectorXd &F,
                                  const Eigen::MatrixXd &B,
                                  const Eigen::MatrixXd &C,
                                  const Eigen::MatrixXd &H,
                                  const Eigen::VectorXd &u,
                                  const Eigen::VectorXd &y, double dt) {
    this->A = A;
    this->F = F;
    this->B = B;
    this->C = C;
    this->H = H;
    this->dt = dt;
    this->update(u, y);
}