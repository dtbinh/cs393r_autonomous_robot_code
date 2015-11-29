#ifndef PLAN_KICK_H
#define PLAN_KICK_H

#include <ros/ros.h>
#include <ros/package.h>
#include <visualization_msgs/MarkerArray.h>
#include <interactive_markers/interactive_marker_server.h>
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <live_params/string_parameter_parsers.hpp>
#include <component_tree/component_tree.hpp>
#include <boost/thread.hpp>
#include <kack.hpp>
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
    std::string m_world_frame;

    bool m_handle_mode;

//    std::string m_kick_filename;
//    std::string m_sparse_kick_filename;

    ros::Publisher m_com_pub;
    ros::Publisher m_js_pub;
    tf::TransformBroadcaster m_odom_broadcaster;

    boost::shared_ptr<KACK::Kack> m_kack;

    interactive_markers::InteractiveMarkerServer m_server;
    visualization_msgs::InteractiveMarker m_foot_marker;
    visualization_msgs::InteractiveMarker m_com_marker;
    visualization_msgs::InteractiveMarkerControl m_foot_control;
    visualization_msgs::InteractiveMarkerControl m_com_control;
    geometry_msgs::Pose m_foot_target;
    geometry_msgs::Pose m_com_target;

    sensor_msgs::JointState m_js;
    dynamics_tree::Matrix4 m_rootTworld;

    void planMove(double foot_x, double foot_y, double foot_z, double foot_pitch, double com_x, double com_y, double dt);
    void comJacobian(std::vector<unsigned int> joint_indices, std::string goal_frame, dynamics_tree::Matrix& jacobian);
    void comRecursive(boost::shared_ptr<dynamics_tree::DynamicsTreeNode> node);
    void planPose(std::vector<double> initial, std::vector<double>& final, double foot_x, double foot_y, double foot_z, double foot_pitch, double com_x, double com_y, double dt, int maxiter = 1000);
    void exportKick(std::string filename);
    void exportSparseKick(std::string filename);

    void doControl();

    void makeFootMarker(geometry_msgs::Pose initial_pose);
    void makeCoMMarker(geometry_msgs::Pose initial_pose);

    void processFootFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
    void processCoMFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
  };
}

#endif //PLAN_KICK_H
