#include "motion_planning/plan_kick.h"

namespace motion_planning
{
  PlanKick::PlanKick(const ros::NodeHandle& nh) :
      m_nh(nh)
  {
    m_nh.param("loop_rate", m_loop_rate, 10000.0);
    
    std::string joint_string = "HeadYaw, HeadPitch, LHipYawPitch, LHipRoll, LHipPitch, LKneePitch, LAnklePitch, LAnkleRoll, RHipYawPitch, RHipRoll, RHipPitch, RKneePitch, RAnklePitch, RAnkleRoll, LShoulderPitch, LShoulderRoll, LElbowYaw, LElbowRoll, LWristYaw, LHand, RShoulderPitch, RShoulderRoll, RElbowYaw, RElbowRoll, RWristYaw, RHand, RFinger13, RFinger12, LFinger21, LFinger13, LFinger11, RFinger22, LFinger22, RFinger21, LFinger12, RFinger23, RFinger11, LFinger23, LThumb1, RThumb1, RThumb2, LThumb2";
    m_joint_names = rpp::parameterStringToStringVector(joint_string);
    std::string urdf_path = ros::package::getPath("motion_planning") + "/nao.urdf";
    m_urdf_model.initFile(urdf_path);
    m_robot.loadURDF(m_urdf_model, "nao", m_joint_names);
    m_robot.printTree();

    m_js.name = m_joint_names;
    m_js.position = std::vector<double>(m_joint_names.size(), 0.0);
    m_js.velocity = std::vector<double>(m_joint_names.size(), 0.0);
    m_js.effort = std::vector<double>(m_joint_names.size(), 0.0);
    m_rootTworld = rad::Matrix4::Identity();

    m_com_pub = m_nh.advertise<geometry_msgs::PointStamped>("/com", 1, true);
    m_js_pub = m_nh.advertise<sensor_msgs::JointState>("/joint_states", 1, true);
  }

  PlanKick::~PlanKick()
  {
  }

  void PlanKick::spin()
  {
    ROS_INFO("%s started.", ros::this_node::getName().c_str());
    ros::Rate loop_rate(m_loop_rate);
    while(ros::ok())
    {
      geometry_msgs::PointStamped com;
      m_robot.kinematics("nao",  m_js.position, m_rootTworld);
      m_robot.centerOfMass("nao", "nao", "l_ankle", com);
      m_com_pub.publish(com);

      m_js.header.stamp = ros::Time::now();
      m_js_pub.publish(m_js);

      tf::StampedTransform trans = rad::matrixToStampedTransform(m_rootTworld, "world", m_robot.getModelLookup("nao").root->name);
      trans.stamp_ = m_js.header.stamp;
      m_odom_broadcaster.sendTransform(trans);

      ros::spinOnce();
      loop_rate.sleep();
    }
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "plan_kick");
  ros::NodeHandle nh("~");

  motion_planning::PlanKick node(nh);
  node.spin();

  return 0;
}
