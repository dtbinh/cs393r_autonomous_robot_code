#include <ros/ros.h>
#include <kalman_filters/linear_kalman_filter.hpp>

#include <std_msgs/Float64.h>
#include <random>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "linear_system_test");
  ros::NodeHandle nh("~");

  ros::Publisher truth_pub = nh.advertise<std_msgs::Float64>("/pos_truth", 1, true);
  ros::Publisher truth_pub2 = nh.advertise<std_msgs::Float64>("/vel_truth", 1, true);
  ros::Publisher noise_pub = nh.advertise<std_msgs::Float64>("/pos_noise", 1, true);
  ros::Publisher estimate_pub = nh.advertise<std_msgs::Float64>("/pos_estimate", 1, true);
  ros::Publisher estimate_pub2 = nh.advertise<std_msgs::Float64>("/vel_estimate", 1, true);
  ros::Publisher control_pub = nh.advertise<std_msgs::Float64>("/control", 1, true);

  typedef LinearKalmanFilter<2, 2, 1> KF;
  KF::StateTransitionMatrix A_matrix = KF::StateTransitionMatrix::Identity();
  KF::InputMatrix B_matrix = KF::InputMatrix::Identity();
  KF::OutputMatrix C_matrix = KF::OutputMatrix::Identity();
  KF::StateCovarianceMatrix R_matrix = KF::StateCovarianceMatrix::Zero();
  KF::MeasurementCovarianceMatrix Q_matrix = KF::MeasurementCovarianceMatrix::Identity();

  KF::StateVector state, estimated_state;
  KF::MeasurementVector measurement;
  KF::ControlVector control;

  double control_magnitude = 10.0;
  double dt = 0.01;
  A_matrix << 1, dt, 0, 1;
  B_matrix << 0, dt;
  C_matrix << 1, 0, 0, 1;
  state << 0.0, -control_magnitude/2.0;
  control << 0.0;

  double var = 1.0;
  std::default_random_engine generator;
  std::normal_distribution<double> measurement_distribution(0.0, var);
  Q_matrix *= var;

  KF filter(A_matrix, B_matrix, C_matrix, R_matrix, Q_matrix);

  double t = 0;
  while(ros::ok())
  {
    KF::MeasurementVector noise;
    noise(0) = measurement_distribution(generator);
    noise(1) = measurement_distribution(generator);
    measurement = state + noise;
    estimated_state = filter.process(measurement, control);

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

    state = A_matrix * state + B_matrix * control;

    ros::Duration(dt).sleep();
  }

  return 0;
}
