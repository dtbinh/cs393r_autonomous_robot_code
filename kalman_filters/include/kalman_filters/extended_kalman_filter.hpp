#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

#include <Eigen/Dense>

//assumes linear system and constant matrices
template<int NumStates, int NumMeasurements, int NumControls>
class ExtendedKalmanFilter
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
  typedef Eigen::Matrix<double, NumStates, NumStates> StateJacobianMatrix;
  typedef Eigen::Matrix<double, NumMeasurements, NumStates> MeasurementJacobianMatrix;

  typedef boost::function<StateVector(StateVector&, ControlVector&)> StateTransitionFunction;
  typedef boost::function<MeasurementVector(StateVector&)> MeasurementFunction;
  typedef boost::function<StateJacobianMatrix(StateVector&, ControlVector&)> StateJacobianFunction;
  typedef boost::function<MeasurementJacobianMatrix(StateVector&)> MeasurementJacobianFunction;

  ExtendedKalmanFilter(StateTransitionFunction g_function, MeasurementFunction h_function, StateJacobianFunction G_function, MeasurementJacobianFunction H_function, StateCovarianceMatrix R_matrix, MeasurementCovarianceMatrix Q_matrix)
  {
    g = g_function;
    h = h_function;
    G = G_function;
    H = H_function;
    R = R_matrix;
    Q = Q_matrix;

    I = IdentityMatrix::Identity();
    mu = StateVector::Zero();
    sigma = StateCovarianceMatrix::Identity();
  }

  //Probabilistic Robotics, pg. 59, Table 3.3
  StateVector process(MeasurementVector z, ControlVector u)
  {
    Jg = G(mu, u);
    Jh = H(mu);
    mu_bar = g(mu, u);
    sigma_bar = Jg * sigma * Jg.transpose() + R;
    K = sigma_bar * Jh.transpose() * (Jh * sigma_bar * Jh.transpose() + Q).inverse();
    mu = mu_bar + K * (z - h(mu_bar));
    sigma = (I - K * Jh) * sigma_bar;

    return mu;
  }
  
  double get_sigma_value( int index ){ return sigma(index); }
  StateVector get_mu(){ return mu; }
  StateVector update_mu(int index , double value)
  {
    mu(index) = value;
    return mu;
  }

private:
  StateTransitionFunction g;
  MeasurementFunction h;
  StateJacobianFunction G;
  MeasurementJacobianFunction H;
  StateCovarianceMatrix R; //process error
  MeasurementCovarianceMatrix Q; //measurement error
  IdentityMatrix I;

  StateVector mu;
  StateVector mu_bar;
  StateCovarianceMatrix sigma;
  StateCovarianceMatrix sigma_bar;
  StateJacobianMatrix Jg;
  MeasurementJacobianMatrix Jh;
  KalmanMatrix K;
};

#endif //KALMAN_FILTER_H
