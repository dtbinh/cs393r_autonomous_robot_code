#include <ros/ros.h>
#include <kalman_filters/extended_kalman_filter.hpp>

#include <std_msgs/Float64.h>
#include <random>

typedef ExtendedKalmanFilter<2, 2, 1> KF;

double dt;
Eigen::Matrix<double, 2, 2> A;
Eigen::Matrix<double, 2, 1> B;
Eigen::Matrix<double, 2, 2> C;

double grav = 9.81;
double len = 1.0;
double damping = 0.2;

enum TestType
{
  STATIONARY,
  LINEAR,
  PENDULUM
};
TestType test_type;

KF::StateVector g(KF::StateVector x, KF::ControlVector u)
{
  if(test_type == STATIONARY || test_type == LINEAR)
  {
    return A * x + B * u;
  }
  else if(test_type == PENDULUM)
  {
    double x_dot = x(1);
    double x_dot_dot = -grav / len * sin((double) x(0)) - damping*x(1);
    return x + dt * KF::StateVector(x_dot, x_dot_dot);
  }
  return KF::StateVector();
}

KF::MeasurementVector h(KF::StateVector x)
{
  if(test_type == STATIONARY || test_type == LINEAR)
  {
    return C * x;
  }
  else if(test_type == PENDULUM)
  {
    return C * x;
  }
  return KF::MeasurementVector();
}

KF::StateJacobianMatrix G(KF::StateVector x, KF::ControlVector u)
{
  if(test_type == STATIONARY || test_type == LINEAR)
  {
    return A;
  }
  else if(test_type == PENDULUM)
  {
    KF::StateJacobianMatrix Jg;
    Jg << 1, dt, 0, (dt*(-grav / len * cos((double) x(0)) - damping*x(1)));
    return Jg;
  }
  return KF::StateJacobianMatrix();
}

KF::MeasurementJacobianMatrix H(KF::StateVector x)
{
  if(test_type == STATIONARY || test_type == LINEAR)
  {
    return C;
  }
  else if(test_type == PENDULUM)
  {
    return C;
  }
  return KF::MeasurementJacobianMatrix();
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ekf_test");
  ros::NodeHandle nh("~");

  dt = 0.01;
  test_type = PENDULUM;

  if(test_type == STATIONARY)
  {
    A << 1, 0, 0, 1;
    B << 0, 0;
  }
  else if(test_type == LINEAR)
  {
    A << 1, dt, 0, 1;
    B << 0, dt;
  }
  C << 1, 0, 0, 1;

  ros::Publisher pos_truth_pub = nh.advertise<std_msgs::Float64>("/pos_truth", 1, true);
  ros::Publisher vel_truth_pub = nh.advertise<std_msgs::Float64>("/vel_truth", 1, true);
  ros::Publisher pos_noise_pub = nh.advertise<std_msgs::Float64>("/pos_noise", 1, true);
  ros::Publisher vel_noise_pub = nh.advertise<std_msgs::Float64>("/vel_noise", 1, true);
  ros::Publisher pos_estimate_pub = nh.advertise<std_msgs::Float64>("/pos_estimate", 1, true);
  ros::Publisher vel_estimate_pub = nh.advertise<std_msgs::Float64>("/vel_estimate", 1, true);
  ros::Publisher control_pub = nh.advertise<std_msgs::Float64>("/control", 1, true);

  KF::StateCovarianceMatrix R_matrix = KF::StateCovarianceMatrix::Zero();
  KF::MeasurementCovarianceMatrix Q_matrix = KF::MeasurementCovarianceMatrix::Identity();

  KF::StateVector state, estimated_state;
  KF::MeasurementVector measurement;
  KF::ControlVector control;

  double control_magnitude = 10.0;
  double var = 1.0;

  if(test_type == STATIONARY)
  {
    state << 5.0, 0.0;
  }
  else if(test_type == LINEAR)
  {
    state << 0.0, -control_magnitude / 2.0;
  }
  else
  {
    var = M_PI/8.0;
    state << M_PI/4.0, 0.0;
  }
  control << 0.0;

  std::default_random_engine generator;
  std::normal_distribution<double> measurement_distribution(0.0, var);
  Q_matrix *= var;

  KF filter(boost::bind(&g, _1, _2), boost::bind(&h, _1), boost::bind(&G, _1, _2), boost::bind(&H, _1), R_matrix, Q_matrix);

  double t = 0;
  while(ros::ok())
  {
    KF::MeasurementVector noise;
    noise(0) = measurement_distribution(generator);
    noise(1) = measurement_distribution(generator);
    measurement = state + noise;
    estimated_state = filter.process(measurement, control);

    if(std::isnan(estimated_state(0)))
    {
      std::cerr << "nan fail!" << std::endl;
      return 0;
    }

    std_msgs::Float64 msg;
    msg.data = state(0);
    pos_truth_pub.publish(msg);
    msg.data = state(1);
    vel_truth_pub.publish(msg);

    msg.data = measurement(0);
    pos_noise_pub.publish(msg);
    msg.data = measurement(1);
    vel_noise_pub.publish(msg);

    msg.data = estimated_state(0);
    pos_estimate_pub.publish(msg);
    msg.data = estimated_state(1);
    vel_estimate_pub.publish(msg);

    msg.data = control(0);
    control_pub.publish(msg);

    t += dt;
    if(((((int) std::floor(t)) % 2) == 0))
      control << control_magnitude;
    else
      control << -control_magnitude;

    state = g(state, control);

    ros::Duration(dt).sleep();
  }

  return 0;
}
