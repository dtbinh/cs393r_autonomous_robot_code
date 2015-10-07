#ifndef UNSCENTED_KALMAN_FILTER_H
#define UNSCENTED_KALMAN_FILTER_H

#include <ros/ros.h>
#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>

//stolen from core/math/MatrixOperations.h
template<typename T, int R>
static Eigen::Matrix<T, R, R> CholeskyDecomposition(const Eigen::Matrix<T, R, R>& A)
{
  Eigen::Matrix<T, R, R> L = Eigen::Matrix<T, R, R>::Zero();
  for(int i = 0; i < R; i++)
  {
    for(int j = 0; j < i + 1; j++)
    {
      T s = 0;
      for(int k = 0; k < j; k++)
      {
        s += L(i, k) * L(j, k);
      }
      // Here we check to ensure that a positive value is passed to the sqrt function.
      // This algorithm requires positive-definite matrices as inputs, but sometimes
      // particular values can fall below zero due to issues with floating point
      // arithmetic. Thus the algorithm provides a small correction where necessary.
      if(i == j)
      {
        auto diff = A(i, i) - s;
        if(diff < 0)
        {
          /*printf("Decomposition Error: i:%i,j:%i,A(%i,%i):%2.4f,s:%2.4f:\n",i,j,i,j,A(i,j),s);*/
          /*cout << A << "\n";*/
          L(i, j) = 1;
        }
        else
          L(i, j) = sqrt(diff);
      }
      else
        L(i, j) = 1.0 / L(j, j) * (A(i, j) - s);
    }
  }
  return L;
}

//assumes linear system and constant matrices
template<int NumStates, int NumMeasurements, int NumControls>
class UnscentedKalmanFilter
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

  typedef boost::function<StateVector(StateVector&, ControlVector&)> StateTransitionFunction;
  typedef boost::function<MeasurementVector(StateVector&)> MeasurementFunction;

  UnscentedKalmanFilter(StateTransitionFunction g_function, MeasurementFunction h_function, StateCovarianceMatrix R_matrix, MeasurementCovarianceMatrix Q_matrix, double alpha = 1.0e-3, double kappa = 0.0, double beta = 2.0)
  {
    g = g_function;
    h = h_function;
    R = R_matrix;
    Q = Q_matrix;

    mu = StateVector::Zero();
    sigma = StateCovarianceMatrix::Identity();

    n = (double) NumStates;
    lambda = alpha * alpha * (n + kappa) - n;
    nlambda = (n + lambda);
    gamma = sqrt(nlambda);

    wm[0] = lambda / nlambda;
    wc[0] = lambda / nlambda + (1.0 - alpha * alpha + beta);
    for(unsigned int i = 1; i < (2 * NumStates); i++)
    {
      wm[i] = 1.0 / (2.0 * nlambda);
      wc[i] = wm[i];
    }
  }

  inline StateCovarianceMatrix covSqrt(StateCovarianceMatrix mat)
  {
    Eigen::EigenSolver<StateCovarianceMatrix> es(mat, true);
    auto V = es.eigenvectors();
    auto sqrtD = es.eigenvalues().cwiseSqrt().asDiagonal();
    auto sqrt = V * sqrtD * V.inverse();

    StateCovarianceMatrix wtf;
    for(unsigned int eigen_is_stupid = 0; eigen_is_stupid < NumStates * NumStates; eigen_is_stupid++)
    {
      wtf(eigen_is_stupid) = sqrt(eigen_is_stupid).real();
    }

    return wtf;
  }

  inline void populateSigmaPoints(StateVector mu_mat, StateCovarianceMatrix sigma_mat, double gamma_val, SigmaPointMatrix& X_mat)
  {
    sigma_mat *= (n + lambda);
    StateCovarianceMatrix sigma_root = CholeskyDecomposition(sigma_mat);
    X_mat[0] = mu_mat;
    for(unsigned int i = 1; i < NumStates; i++)
    {
      X_mat[i] = mu_mat + sigma_root.col(i);
      X_mat[i + NumStates] = mu_mat - sigma_root.col(i);
    }
  }

  //Probabilistic Robotics, pg. 70, Table 3.4
  StateVector process(MeasurementVector z, ControlVector u)
  {
    populateSigmaPoints(mu, sigma, gamma, X);

    mu_bar = StateVector::Zero();
    for(unsigned int i = 0; i < (2 * NumStates); i++)
    {
      X_bar_star[i] = g(X[i], u);
      mu_bar += wm[i] * X_bar_star[i];
    }

    sigma_bar = R;
    for(unsigned int i = 0; i < (2 * NumStates); i++)
    {
      StateVector no_mean = X_bar_star[i] - mu_bar;
      sigma_bar += wc[i] * no_mean * no_mean.transpose();
    }

    populateSigmaPoints(mu_bar, sigma_bar, gamma, X_bar);

    z_hat = MeasurementVector::Zero();
    for(unsigned int i = 0; i < (2 * NumStates); i++)
    {
      Z_bar[i] = h(X_bar[i]);
      z_hat += wm[i] * Z_bar[i];
    }

    S = Q;
    sigma_bar_xz = StateCovarianceMatrix::Zero();
    for(unsigned int i = 0; i < (2 * NumStates); i++)
    {
      StateVector no_mean = Z_bar[i] - z_hat;
      S += wc[i] * no_mean * no_mean.transpose();
      sigma_bar_xz += wc[i] * (X_bar[i] - mu_bar) * no_mean.transpose();
    }

    K = sigma_bar_xz * S.inverse();
    mu = mu_bar + K * (z - z_hat);
    sigma = sigma_bar - K * S * K.transpose();

    std::cerr << "================================" << std::endl;
    std::cerr << "n: " << n << std::endl;
    std::cerr << "lambda: " << lambda << std::endl;
    std::cerr << "nlambda: " << nlambda << std::endl;
    std::cerr << "gamma: " << gamma << std::endl;
    std::cerr << "z_hat:\n " << z_hat << std::endl;
    std::cerr << "mu:\n " << mu << std::endl;
    std::cerr << "mu_bar:\n " << mu_bar << std::endl;
    std::cerr << "sigma:\n " << sigma << std::endl;
    std::cerr << "sigma_bar:\n " << sigma_bar << std::endl;
    std::cerr << "sigma_bar_xz:\n " << sigma_bar_xz << std::endl;
    std::cerr << "S:\n " << S << std::endl;
    std::cerr << "S.inverse():\n " << S.inverse() << std::endl;
    std::cerr << "K:\n " << K << std::endl;
    for(unsigned int i = 0; i < (2 * NumStates); i++)
    {
      std::cerr << "wm(" << i << "): " << wm[i] << std::endl;
      std::cerr << "wc(" << i << "): " << wc[i] << std::endl;
      std::cerr << "X(" << i << "): " << X[i].transpose() << std::endl;
      std::cerr << "X_bar_star(" << i << "): " << X_bar_star[i].transpose() << std::endl;
      std::cerr << "X_bar(" << i << "): " << X_bar[i].transpose() << std::endl;
      std::cerr << "Z_bar(" << i << "): " << Z_bar[i].transpose() << std::endl;
    }

    return mu;
  }

private:
  StateTransitionFunction g;
  MeasurementFunction h;

  StateCovarianceMatrix R;
  MeasurementCovarianceMatrix Q; //measurement error
  MeasurementCovarianceMatrix S; //measurement error

  double n;
  double lambda;
  double nlambda;
  double gamma;
  WeightMatrix wm;
  WeightMatrix wc;

  SigmaPointMatrix X;
  SigmaPointMatrix X_bar_star;
  SigmaPointMatrix X_bar;
  SigmaPointMatrix Z_bar;

  MeasurementVector z_hat;
  StateVector mu;
  StateVector mu_bar;
  StateCovarianceMatrix sigma;
  StateCovarianceMatrix sigma_bar;
  StateCovarianceMatrix sigma_bar_xz;
  KalmanMatrix K;
};

#endif //UNSCENTED_KALMAN_FILTER_H
