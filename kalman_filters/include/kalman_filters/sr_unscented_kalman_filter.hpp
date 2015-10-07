#ifndef SR_UNSCENTED_KALMAN_FILTER_H
#define SR_UNSCENTED_KALMAN_FILTER_H

#include <ros/ros.h>
#include <Eigen/Dense>

//assumes linear system and constant matrices
template<int NumStates, int NumMeasurements, int NumControls>
class SRUnscentedKalmanFilter
{
public:
  typedef Eigen::Matrix<double, NumStates, 1> StateVector;
  typedef Eigen::Matrix<double, NumMeasurements, 1> MeasurementVector;
  typedef Eigen::Matrix<double, NumControls, 1> ControlVector;

  typedef Eigen::Matrix<double, NumStates, NumStates> StateCovarianceMatrix;
  typedef Eigen::Matrix<double, NumMeasurements, NumMeasurements> MeasurementCovarianceMatrix;
  typedef Eigen::Matrix<double, NumStates, NumMeasurements> KalmanMatrix;
  typedef boost::array<double, (1 + 2 * NumStates)> WeightMatrix;
  typedef boost::array<StateVector, (1 + 2 * NumStates)> SigmaPointMatrix;
  typedef boost::array<MeasurementVector, (1 + 2 * NumStates)> MeasurementPointMatrix;

  typedef boost::function<StateVector(StateVector&, ControlVector&)> StateTransitionFunction;
  typedef boost::function<MeasurementVector(StateVector&)> MeasurementFunction;

  SRUnscentedKalmanFilter(StateTransitionFunction g_function, MeasurementFunction h_function, StateCovarianceMatrix R_matrix, MeasurementCovarianceMatrix Q_matrix, double w0)
  {
    g = g_function;
    h = h_function;
    R = R_matrix;
    Q = Q_matrix;

    mu = StateVector::Zero();
    sigma = StateCovarianceMatrix::Identity();

    n = (double) NumStates;
    w[0] = w0;
    for(unsigned int i = 1; i < (2 * NumStates); i++)
    {
      w[i] = (1 - w[0]) / (2.0 * n);
    }
  }

  StateVector process(MeasurementVector z, ControlVector u)
  {
    //todo: init S

    //17
    X[0] = x_hat;
    for(unsigned int i = 1; i < (NumStates); i++)
    {
      X[i] = X[0] + (sqrt(n / (1 - w[0])) * S).col(i);
    }

    //18
    for(unsigned int i = 0; i < (2 * NumStates); i++)
    {
      X_star[i] = g(X[i], u);
    }

    //19
    x_hat_bar = StateVector::Zero();
    for(unsigned int i = 0; i < (2 * NumStates); i++)
    {
      x_hat_bar += w[i] * X_star[i];
    }

    //20
    S_bar = qr{};

    //21
    S_bar = cholupdate;

    //22
    X2[0] = x_hat_bar;
    for(unsigned int i = 1; i < (NumStates); i++)
    {
      X2[i] = X2[0] + (sqrt(n / (1 - w[0])) * S_bar).col(i);
    }

    //23
    for(unsigned int i = 0; i < (2 * NumStates); i++)
    {
      Y_star[i] = h(X2[i]);
    }

    //24
    y_hat_bar = MeasurementVector::Zero();
    for(unsigned int i = 0; i < (2 * NumStates); i++)
    {
      y_hat_bar += w[i] * Y_star[i];
    }

    //25
    MeasurementCovarianceMatrix sqrtQ;
    Sy_tilde = qr(sqrt(w[1])*(), sqrtQ);

    //26
    Sy_tilde = cholupdate(Sy_tilde, Y0-y_hat_bar, w[0]);

    //27
    Pxy = ::Zero();
    for(unsigned int i = 0; i < (2 * NumStates); i++)
    {
      Pxy += w[i] * (X2[i] - x_hat_bar) * (Y_star[i] - y_hat_bar).transpose();
    }

    //28
    K = (Pxy / Sy_tilde.transpose()) / Sy_tilde;
    x_hat = x_hat_bar + K * (z - y_hat_bar);

    //29
    U = K * Sy_tilde;

    //30
    S = cholupdate(S_bar, U, -1);

    return x_hat;
  }

private:
  double n;
  WeightMatrix w;

  StateTransitionFunction g;
  MeasurementFunction h;

  StateCovarianceMatrix R;
  StateCovarianceMatrix S;
  StateCovarianceMatrix S_bar;
  MeasurementCovarianceMatrix Sy_tilde;
  MeasurementCovarianceMatrix Q; //measurement error

  SigmaPointMatrix X;
  SigmaPointMatrix X_star;
  SigmaPointMatrix X2;
  MeasurementPointMatrix Y_star;

  KalmanMatrix K;
  KalmanMatrix U;

  StateVector x_hat;
  StateVector x_hat_bar;
  MeasurementVector y_hat_bar;
  StateCovarianceMatrix sigma;
};

#endif //SR_UNSCENTED_KALMAN_FILTER_H
