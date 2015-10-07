#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

//#include <ros/ros.h>
#include <Eigen/Dense>

//assumes linear system and constant matrices
template<int NumStates, int NumMeasurements, int NumControls>
class LinearKalmanFilter
{
public:
  typedef Eigen::Matrix<double, NumStates, 1> StateVector;
  typedef Eigen::Matrix<double, NumMeasurements, 1> MeasurementVector;
  typedef Eigen::Matrix<double, NumControls, 1> ControlVector;

  typedef Eigen::Matrix<double, NumStates, NumStates> StateCovarianceMatrix;
  typedef Eigen::Matrix<double, NumMeasurements, NumMeasurements> MeasurementCovarianceMatrix;
  typedef Eigen::Matrix<double, NumStates, NumStates> StateTransitionMatrix;
  typedef Eigen::Matrix<double, NumStates, NumControls> InputMatrix;
  typedef Eigen::Matrix<double, NumMeasurements, NumStates> OutputMatrix;
  typedef Eigen::Matrix<double, NumStates, NumMeasurements> KalmanMatrix;
  typedef Eigen::Matrix<double, NumStates, NumStates> IdentityMatrix;

  LinearKalmanFilter(StateTransitionMatrix A_matrix, InputMatrix B_matrix, OutputMatrix C_matrix, StateCovarianceMatrix R_matrix, MeasurementCovarianceMatrix Q_matrix)
  {
    mu = StateVector::Zero();
    sigma = StateCovarianceMatrix::Identity();// * std::numeric_limits<double>::max();
    A = A_matrix;
    B = B_matrix;
    C = C_matrix;
    R = R_matrix;
    Q = Q_matrix;
    I = IdentityMatrix::Identity();
  }

  //Probabilistic Robotics, pg. 42, Table 3.1
  StateVector process(MeasurementVector z, ControlVector u)
  {
    mu_bar = A * mu + B * u;
    sigma_bar = A * sigma * A.transpose() + R;
    K = sigma_bar * C.transpose() * (C * sigma_bar * C.transpose() + Q).inverse();
    mu = mu_bar + K * (z - C * mu_bar);
    sigma = (I - K * C) * sigma_bar;

//    std::cerr << "================================" << std::endl;
//    std::cerr << "mu_bar: " << mu_bar.transpose() << std::endl;
//    std::cerr << "sigma_bar: " << sigma_bar.transpose() << std::endl;
//    std::cerr << "K: " << K.transpose() << std::endl;
//    std::cerr << "mu: " << mu.transpose() << std::endl;
//    std::cerr << "sigma: " << sigma.transpose() << std::endl;

    return mu;
  }

  double get_sigma_value( int index_x , int index_y){ return sigma(index_x, index_y); }
  StateVector get_mu(){ return mu; }
  StateVector update_mu(int index , double value)
  {
    mu(index) = value;
    return mu;
  }

private:
  StateTransitionMatrix A;
  InputMatrix B;
  OutputMatrix C;
  StateCovarianceMatrix R; //process error
  MeasurementCovarianceMatrix Q; //measurement error
  IdentityMatrix I;

  StateVector mu;
  StateVector mu_bar;
  StateCovarianceMatrix sigma;
  StateCovarianceMatrix sigma_bar;
  KalmanMatrix K;
};

#endif //KALMAN_FILTER_H
