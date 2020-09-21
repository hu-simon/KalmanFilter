/*
 * Name        : Kalman.h
 * Author      : Simon Hu
 * Version     :
 * Copyright   : 2020, all rights reserved
 * Description : C++ header file containing prototypes for the Kalman filter
 */

#ifndef KALMAN_H_
#define KALMAN_H_

#include <Eigen/Dense>

class BasicKalmanFilter {
  private:
    // Matrices
    Eigen::MatrixXd A;
    Eigen::MatrixXd C;
    Eigen::MatrixXd Q;
    Eigen::MatrixXd R;
    Eigen::MatrixXd P;
    Eigen::MatrixXd K;
    Eigen::MatrixXd P0;
    Eigen::MatrixXd I;

    // Estimated states
    Eigen::VectorXd x_hat;
    Eigen::VectorXd x_hat_new;

    // Filter parameters
    unsigned short int m;
    unsigned short int n;
    double t0;
    double t;
    double dt;
    bool is_initialized;

  public:
    // Constructor
    BasicKalmanFilter();
    BasicKalmanFilter(const Eigen::MatrixXd &A, const Eigen::MatrixXd &C,
                      const Eigen::MatrixXd &Q, const Eigen::MatrixXd &R,
                      const Eigen::MatrixXd &P, double dt);

    // Destructors
    virtual ~BasicKalmanFilter();

    // Process methods
    void init();
    void init(const Eigen::VectorXd &x0, double t0);
    void update(const Eigen::VectorXd &y);
    void update(Eigen::VectorXd &y, double dt);

    // Accessors
    Eigen::VectorXd get_state() { return x_hat; };
    double get_time() { return t; };
};

class KalmanFilter {
  private:
    // Matrices
    Eigen::MatrixXd A;
    Eigen::MatrixXd B;
    Eigen::MatrixXd C;
    Eigen::MatrixXd Q;
    Eigen::MatrixXd R;
    Eigen::MatrixXd P;
    Eigen::MatrixXd K;
    Eigen::MatrixXd P0;
    Eigen::MatrixXd I;

    // Estimated states
    Eigen::VectorXd x_hat;
    Eigen::VectorXd x_hat_new;

    // Controls
    Eigen::VectorXd u;

    // Filter parameters
    unsigned short int m;
    unsigned short int n;
    double t0;
    double t;
    double dt;
    bool is_initialized;

  public:
    // Constructors
    KalmanFilter();
    KalmanFilter(const Eigen::MatrixXd &A, const Eigen::MatrixXd &B,
                 const Eigen::MatrixXd &C, const Eigen::MatrixXd &Q,
                 const Eigen::MatrixXd &R, const Eigen::MatrixXd &P, double dt);

    // Destructors
    virtual ~KalmanFilter();

    // Process methods
    void init();
    void init(const Eigen::VectorXd &x0, double t0);
    void init(const Eigen::VectorXd &x0, const Eigen::VectorXd &u, double t0);
    void update(const Eigen::VectorXd &u, const Eigen::VectorXd &y);
    void update(const Eigen::VectorXd &u, const Eigen::VectorXd &y, double dt);

    // Accessors
    Eigen::VectorXd get_state() { return x_hat; };
    double get_time() { return t; };
};

/* TO IMPLEMENT */
class ExtendedKalmanFilter {
  private:
    // Matrices
    Eigen::MatrixXd A;
    Eigen::MatrixXd F; // Jacobian of motion model
    Eigen::MatrixXd B;
    Eigen::MatrixXd C;
    Eigen::MatrixXd H; // Jacobian of observation model
    Eigen::MatrixXd Q;
    Eigen::MatrixXd R;
    Eigen::MatrixXd P;
    Eigen::MatrixXd K;
    Eigen::MatrixXd P0;
    Eigen::MatrixXd I;

    // Estimated states
    Eigen::VectorXd x_hat;
    Eigen::VectorXd x_hat_new;

    // Controls
    Eigen::VectorXd u;

    // Filter parameters
    unsigned short int m;
    unsigned short int n;
    double t0;
    double t;
    double dt;
    bool is_initialized;

  public:
    // Constructors
    ExtendedKalmanFilter();
    ExtendedKalmanFilter(const Eigen::MatrixXd &A, const Eigen::MatrixXd &F,
                         const Eigen::MatrixXd &B, const Eigen::MatrixXd &C,
                         const Eigen::MatrixXd &H, const Eigen::MatrixXd &Q,
                         const Eigen::MatrixXd &R, const Eigen::MatrixXd &P,
                         double dt);

    // Destructors
    ~ExtendedKalmanFilter();

    // Process methods
    void init();
    void init(const Eigen::VectorXd &x0, double t0);
    void init(const Eigen::VectorXd &x0, const Eigen::VectorXd &u, double t0);
    void update(const Eigen::VectorXd &u, const Eigen::VectorXd &y);
    void update(const Eigen::VectorXd &A, const Eigen::VectorXd &F,
                const Eigen::MatrixXd &B, const Eigen::MatrixXd &C,
                const Eigen::MatrixXd &H, const Eigen::VectorXd &u,
                const Eigen::VectorXd &y, double dt);

    // Accessors
    Eigen::VectorXd get_state() { return x_hat; };
    double get_time() { return t; };
};

#endif