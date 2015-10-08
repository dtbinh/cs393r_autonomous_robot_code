#ifndef UNSCENTED_KALMAN_FILTER_H
#define UNSCENTED_KALMAN_FILTER_H

#include <ros/ros.h>
#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>

//stolen from core/math/MatrixOperations.h
template<typename T, int R>
static Eigen::Matrix<T, R, R> chol(const Eigen::Matrix<T, R, R>& A)
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
  typedef Eigen::Matrix<double, (1 + 2 * NumStates), 1> WeightVector;
  typedef Eigen::Matrix<double, (1 + 2 * NumStates), (1 + 2 * NumStates)> WeightMatrix;
//  typedef boost::array<double, (1 + 2 * NumStates)> WeightVector;
//  typedef boost::array<StateVector, (1 + 2 * NumStates)> SigmaPointMatrix;
  typedef Eigen::Matrix<double, NumStates, (1 + 2 * NumStates)> SigmaPointMatrix;

  typedef boost::function<StateVector(StateVector, ControlVector)> StateTransitionFunction;
  typedef boost::function<MeasurementVector(StateVector, ControlVector)> MeasurementFunction;

  UnscentedKalmanFilter(StateTransitionFunction g_function, MeasurementFunction h_function, StateCovarianceMatrix R_matrix, MeasurementCovarianceMatrix Q_matrix, double alpha = 1.0e-3, double kappa = 0.0, double beta = 2.0)
  {
    g = g_function;
    h = h_function;
    R = R_matrix;
    Q = Q_matrix;

    x = StateVector::Zero();
    P = StateCovarianceMatrix::Identity();

    n = (double) NumStates;
    lambda = alpha * alpha * (n + kappa) - n;
    c = (n + lambda);

   wm(0) = lambda / c;
   wc(0) = lambda / c + (1.0 - alpha * alpha + beta);
   for(unsigned int i = 1; i < (2 * NumStates); i++)
   {
     wm(i) = 1.0 / (2.0 * c);
     wc(i) = wm(i);
   }

    Wm = wm.asDiagonal();
    Wc = wc.asDiagonal();

    c = sqrt(c);
  }

  template<typename MapFunction, typename InArrayType, typename InCovType, typename ControlType, typename OutType, typename OutArrayType, typename OutCovType>
  inline void ut(MapFunction f, InArrayType X, ControlType u, InCovType R, OutType& y, OutArrayType& Y, OutArrayType& Y1, OutCovType& P)
  {
    y = OutType::Zero();
    for(unsigned int k = 0; k < 2 * NumStates; k++)
    {
      Y.col(k) = f(X.col(k), u);
      y += wm[k] * Y.col(k);
    }
    for(unsigned int k = 0; k < 2 * NumStates; k++)
    {
      Y1.col(k) = Y.col(k) - y;
    }

    P = Y1 * Wc * Y1.transpose() + R;
  }

  inline void populateSigmaPoints(StateVector x, StateCovarianceMatrix P, double c, SigmaPointMatrix& X)
  {
    Eigen::LLT<StateCovarianceMatrix> llt(P);
    StateCovarianceMatrix A = llt.matrixL();
    A *= sqrt(n / (1.0 - (double)wm(0)));
//    StateCovarianceMatrix A = c * chol(P).transpose();
    std::cerr << "sigma points: P=\n" << P << "\n A=\n " << A << std::endl;
    X.col(0) = x;
    for(unsigned int i = 1; i < NumStates; i++)
    {
      X.col(i) = x + A.col(i);
      X.col(i + NumStates) = x - A.col(i);
    }
  }

  StateVector process(MeasurementVector z, ControlVector u)
  {
    populateSigmaPoints(x, P, c, X);
    ut(g, X, u, R, x1, X1, X2, P1);
    ut(h, X1, u, Q, z1, Z1, Z2, P2);
    P12 = X2 * Wc * Z2.transpose() + Q;
    K = P12 * P2.inverse();
    x = x1 + K * (z - z1);
    P = P1 - K * P12.transpose();

    std::cerr << "================================" << std::endl;
    std::cerr << "n: " << n << std::endl;
    std::cerr << "c: " << c << std::endl;
    std::cerr << "Wm:\n " << Wm << std::endl;
    std::cerr << "Wc:\n " << Wc << std::endl;
    std::cerr << "x:\n " << x << std::endl;
    std::cerr << "X:\n " << X << std::endl;
    std::cerr << "P:\n " << P << std::endl;
    std::cerr << "K:\n " << K << std::endl;

    std::cerr << "x1:\n " << x1 << std::endl;
    std::cerr << "X1:\n " << X1 << std::endl;
    std::cerr << "X2:\n " << X2 << std::endl;
    std::cerr << "P1:\n " << P1 << std::endl;
    std::cerr << "z1:\n " << z1 << std::endl;
    std::cerr << "Z1:\n " << Z1 << std::endl;
    std::cerr << "Z2:\n " << Z2 << std::endl;
    std::cerr << "P2:\n " << P2 << std::endl;

    return x;
  }

private:
  StateTransitionFunction g;
  MeasurementFunction h;

  StateCovarianceMatrix R;
  MeasurementCovarianceMatrix Q; //measurement error
  MeasurementCovarianceMatrix S; //measurement error

  double n;
  double lambda;
  double c;
  WeightVector wm;
  WeightVector wc;
  WeightMatrix Wm;
  WeightMatrix Wc;

  SigmaPointMatrix X;
  SigmaPointMatrix X1;
  SigmaPointMatrix X2;
  SigmaPointMatrix Z;
  SigmaPointMatrix Z1;
  SigmaPointMatrix Z2;

  SigmaPointMatrix X_bar;
  SigmaPointMatrix Z_bar;

  MeasurementVector z_hat;
  StateVector x;
  StateVector x1;
  StateVector z1;
  StateCovarianceMatrix P;
  StateCovarianceMatrix P1;
  StateCovarianceMatrix P2;
  StateCovarianceMatrix P12;
  StateCovarianceMatrix sigma_bar_xz;
  KalmanMatrix K;
};

#endif //UNSCENTED_KALMAN_FILTER_H
