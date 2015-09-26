#ifndef PLAN_KICK_H
#define PLAN_KICK_H

#include <ros/ros.h>
#include <ros/package.h>
#include <visualization_msgs/MarkerArray.h>
#include <rad/rad.hpp>
#include <live_params/live_params.hpp>
#include <live_params/string_parameter_parsers.hpp>
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>

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
    double m_loop_rate;

    ros::Publisher m_com_pub;
    ros::Publisher m_js_pub;
    tf::TransformBroadcaster m_odom_broadcaster;

    std::vector<std::string> m_joint_names;
    sensor_msgs::JointState m_js;
    rad::Matrix4 m_rootTworld;

    urdf::Model m_urdf_model;
    rad::DynamicsModel m_robot;
  };
}

#endif //PLAN_KICK_H
