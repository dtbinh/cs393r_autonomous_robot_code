#include <ros/ros.h>
#include <kalman_filters/linear_kalman_filter.hpp>

#include <std_msgs/Float64.h>
#include <random>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "stationary_test");
  ros::NodeHandle nh("~");

  ros::Publisher truth_pub = nh.advertise<std_msgs::Float64>("/truth", 1, true);
  ros::Publisher noise_pub = nh.advertise<std_msgs::Float64>("/noise", 1, true);
  ros::Publisher estimate_pub = nh.advertise<std_msgs::Float64>("/estimate", 1, true);

  typedef LinearKalmanFilter<1, 1, 1> KF;
  KF::StateTransitionMatrix A_matrix = KF::StateTransitionMatrix::Identity();
  KF::InputMatrix B_matrix = KF::InputMatrix::Zero();
  KF::OutputMatrix C_matrix = KF::OutputMatrix::Identity();
  KF::StateCovarianceMatrix R_matrix = KF::StateCovarianceMatrix::Zero();
  KF::MeasurementCovarianceMatrix Q_matrix = KF::MeasurementCovarianceMatrix::Identity();

  KF::StateVector state, estimated_state;
  KF::MeasurementVector measurement;
  KF::ControlVector control;

  KF filter(A_matrix, B_matrix, C_matrix, R_matrix, Q_matrix);

  state(0) = (5.0);
  control(0) = 0;

  std::default_random_engine generator;
  std::normal_distribution<double> measurement_distribution(0.0, 1.0);

  while(ros::ok())
  {
    KF::MeasurementVector noise;
    noise(0) = measurement_distribution(generator);
    measurement = state + noise;
    estimated_state = filter.process(measurement, control);

    std_msgs::Float64 msg;
    msg.data = state(0);
    truth_pub.publish(msg);
    msg.data = measurement(0);
    noise_pub.publish(msg);
    msg.data = estimated_state(0);
    estimate_pub.publish(msg);

    ros::Duration(0.1).sleep();
  }

  return 0;
}
