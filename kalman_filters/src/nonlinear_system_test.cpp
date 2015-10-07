#include <ros/ros.h>
#include <kalman_filters/unscented_kalman_filter.hpp>

#include <std_msgs/Float64.h>
#include <random>

typedef UnscentedKalmanFilter<2, 2, 1> KF;

Eigen::Matrix<double, 2, 2> A;
Eigen::Matrix<double, 2, 1> B;
Eigen::Matrix<double, 2, 2> C;

KF::StateVector g(KF::StateVector x, KF::ControlVector u)
{
  return A * x + B * u;
}

KF::MeasurementVector h(KF::StateVector x)
{
  return C * x;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "nonlinear_system_test");
  ros::NodeHandle nh("~");

  bool stationary = true;

  double dt = 0.01;
  if(stationary)
  {
    A << 1, 0, 0, 1;
    B << 0, 0;
    C << 1, 0, 0, 1;
  }
  else
  {
    A << 1, dt, 0, 1;
    B << 0, dt;
    C << 1, 0, 0, 1;
  }

  ros::Publisher truth_pub = nh.advertise<std_msgs::Float64>("/pos_truth", 1, true);
  ros::Publisher truth_pub2 = nh.advertise<std_msgs::Float64>("/vel_truth", 1, true);
  ros::Publisher noise_pub = nh.advertise<std_msgs::Float64>("/pos_noise", 1, true);
  ros::Publisher estimate_pub = nh.advertise<std_msgs::Float64>("/pos_estimate", 1, true);
  ros::Publisher estimate_pub2 = nh.advertise<std_msgs::Float64>("/vel_estimate", 1, true);
  ros::Publisher control_pub = nh.advertise<std_msgs::Float64>("/control", 1, true);

  KF::StateCovarianceMatrix R_matrix = KF::StateCovarianceMatrix::Zero();
  KF::MeasurementCovarianceMatrix Q_matrix = KF::MeasurementCovarianceMatrix::Identity();

  KF::StateVector state, estimated_state;
  KF::MeasurementVector measurement;
  KF::ControlVector control;

  double control_magnitude = 10.0;

  if(stationary)
  {
    state << 5.0, 0.0;
  }
  else
  {
    state << 0.0, -control_magnitude / 2.0;
  }
  control << 0.0;

  double var = 1.0;
  std::default_random_engine generator;
  std::normal_distribution<double> measurement_distribution(0.0, var);
  Q_matrix *= var;

  KF filter(boost::bind(&g, _1, _2), boost::bind(&h, _1), R_matrix, Q_matrix);

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
    truth_pub.publish(msg);
    msg.data = state(1);
    truth_pub2.publish(msg);
    msg.data = measurement(0);
    noise_pub.publish(msg);
    msg.data = estimated_state(0);
    estimate_pub.publish(msg);
    msg.data = estimated_state(1);
    estimate_pub2.publish(msg);

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
