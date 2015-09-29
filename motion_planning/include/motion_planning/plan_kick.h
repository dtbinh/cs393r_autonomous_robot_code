#ifndef PLAN_KICK_H
#define PLAN_KICK_H

#include <ros/ros.h>
#include <ros/package.h>
#include <visualization_msgs/MarkerArray.h>
#include <matec_dynamics_tree/dynamics_graph.h>
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <matec_msgs/Odometry.h>
#include <matec_utils/string_parameter_parsers.h>
#include <iostream>
#include <fstream>

namespace motion_planning
{
  class PlanKick
  {
  public:
    PlanKick(const ros::NodeHandle& nh);
    ~PlanKick();
    void spin();

  private:
    ros::NodeHandle m_nh;
    double m_frame_rate;

    double m_lift_height;
    double m_kick_time;
    double m_kick_dist;
    std::string m_kick_filename;

    ros::Publisher m_com_pub;
    ros::Publisher m_js_pub;
    tf::TransformBroadcaster m_odom_broadcaster;

    std::vector<std::vector<double> > m_joint_plan;

    std::vector<std::string> m_joint_names;
    std::vector<unsigned int> m_joint_ids;
    std::vector<double> m_joint_mins;
    std::vector<double> m_joint_maxes;
    std::vector<double> m_joint_vels;
    sensor_msgs::JointState m_js;
    matec_utils::Matrix4 m_rootTworld;
    matec_msgs::Odometry m_odom;

    urdf::Model m_urdf_model;
    dynamics_tree::DynamicsGraph m_graph;
    dynamics_tree::DynamicsTree m_tree;

    void planMove(double foot_x, double foot_y, double foot_z, double foot_pitch, double com_x, double com_y, double dt);
    void comJacobian(std::vector<unsigned int> joint_indices, std::string goal_frame, matec_utils::Matrix& jacobian);
    void comRecursive(boost::shared_ptr<dynamics_tree::DynamicsTreeNode> node);
    void planPose(std::vector<double> initial, std::vector<double>& final, double foot_x, double foot_y, double foot_z, double foot_pitch, double com_x, double com_y, double dt, int maxiter = 1000);
    void exportKick(std::string filename);
  };
}

#endif //PLAN_KICK_H
