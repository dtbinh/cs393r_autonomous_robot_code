#include "motion_planning/plan_kick.h"

namespace motion_planning
{
  PlanKick::PlanKick(const ros::NodeHandle& nh) :
      m_nh(nh)
  {
    m_nh.param("loop_rate", m_frame_rate, 30.0);
    m_nh.param("lift_height", m_lift_height, 0.04);
    m_nh.param("kick_time", m_kick_time, 0.5);
    m_nh.param("kick_dist", m_kick_dist, 0.08);
    m_nh.param("kick_filename", m_kick_filename, ros::package::getPath("motion_planning") + std::string("/kick_trajectory.py"));
    m_nh.param("sparse_kick_filename", m_sparse_kick_filename, ros::package::getPath("motion_planning") + std::string("/../data/kicks/default.yaml"));

    m_kack.reset(new KACK::Kack(ros::package::getPath("motion_planning") + "/nao.urdf"));
    std::string initial_pos_string = "      0,      -0.4,            0,        0,    -0.436,      0.873,      -0.436,          0,            0,        0,    -0.436,      0.873,      -0.436,          0,            1.4,          0.3,         0,          -0.05,            1.4,          -0.3,         0,          0.05,         0,     0,         0,     0";

    m_js.name = m_kack->getJointNames();
    m_js.position = rpp::parameterStringToFPVector(initial_pos_string);
    m_js.velocity = std::vector<double>(m_js.name.size(), 0.0);
    m_js.effort = std::vector<double>(m_js.name.size(), 0.0);
    m_rootTworld = dynamics_tree::Matrix4::Identity();

    m_com_pub = m_nh.advertise<geometry_msgs::PointStamped>("/com", 1, true);
    m_js_pub = m_nh.advertise<sensor_msgs::JointState>("/joint_states", 1, true);

    //test plan
    double foot_separation = 0.1;
//    double kick_dist = 0.14;
//    double landing_splay = 0.01;

    double ppt = 1e-3;
    double pot = 1e-3;
    double cpt = 1e-1;
    double czt = 1.0;

    KACK::Pose start(0.0, -foot_separation, 0.0, 0.0, 0.0, 0.0);
    KACK::Pose lift(0.0, -foot_separation, 0.3, 0.0, 0.0, 0.0);

    KACK::Point com_center(0.0, -foot_separation / 2.0, 0.4);
    KACK::Point com_support(0.0, 0.0, 0.4);

    std::vector<KACK::CartesianKeyframe> keyframes;
    keyframes.push_back(KACK::CartesianKeyframe(3.0, start, com_support, ppt, pot, cpt, czt));
    keyframes.push_back(KACK::CartesianKeyframe(0.6, lift, com_support, ppt, pot, cpt, czt));
    keyframes.push_back(KACK::CartesianKeyframe(0.6, start, com_support, ppt, pot, cpt, czt));
    keyframes.push_back(KACK::CartesianKeyframe(0.6, start, com_center, ppt, pot, cpt, czt));
    m_kack->plan(m_js.position, keyframes);

//    planMove(0.0, -foot_separation, 0.0, 0.0, 0.01, 0.005, 3.0);
//    planMove(kick_dist, -foot_separation, 0.03, 0.0, 0.03, -0.01, 0.6);
//    planMove(kick_dist, -foot_separation - landing_splay, 0.0, 0.0, 0.045, -0.055, 0.6);
//    planMove(kick_dist, -foot_separation - landing_splay, 0.0, 0.0, 0.045, -0.055, 10.0);
//    planMove(kick_dist, -foot_separation - landing_splay, 0.0, 0.0, 0.04, -0.01, 5.0);
//    planMove(0.025, -foot_separation, 0.02, 0.0, 0.01, 0.0, 2.0);
//    planMove(0.025, -foot_separation, 0.0, 0.0, 0.01, 0.0, 2.0);

// std::cerr << "Planning return" << std::endl;
// planMove(0.0, -foot_separation, m_lift_height, 0.025, 0.0, 0.5);
// std::cerr << "Planning set" << std::endl;
// planMove(0.0, -foot_separation, 0.0, 0.025, 0.0, 0.5);
// std::cerr << "Planning balance" << std::endl;
// planMove(0.0, -foot_separation, 0.0, 0.025, -0.05, 1.25);

//    exportKick(m_kick_filename);
//    exportSparseKick(m_sparse_kick_filename);
  }

  PlanKick::~PlanKick()
  {
  }

//  void PlanKick::exportKick(std::string filename)
//  {
//    std::ofstream data_file;
//    data_file.open(filename.c_str());
//
//    //write joints
//    data_file << "names = [";
//    for(unsigned int i = 0; i < (m_joint_names.size() - 4); i++)
//    {
//      if(i != 0)
//      {
//        data_file << ", ";
//      }
//      data_file << "'" << m_joint_names.at(i) << "'";
//    }
//    data_file << "]" << std::endl;
//
//    data_file << "trajectory = [";
//    for(unsigned int i = 0; i < m_joint_plan.size(); i++)
//    {
//      if(i != 0)
//      {
//        data_file << ", ";
//      }
//      data_file << "[";
//      for(unsigned int j = 0; j < (m_joint_names.size() - 4); j++)
//      {
//        if(j != 0)
//        {
//          data_file << ", ";
//        }
//        data_file << m_joint_plan[i][j];
//
//        //sanity check
//        if(m_joint_plan[i][j] > m_joint_maxes[j] || m_joint_plan[i][j] < m_joint_mins[j])
//        {
//          std::cerr << "BAD JOINT CLAMP! plan idx: " << i << " joint idx: " << j << " val: " << m_joint_plan[i][j] << " min: " << m_joint_mins[j] << " max: " << m_joint_maxes[j] << std::endl;
//        }
//      }
//      data_file << "]";
//    }
//    data_file << "]" << std::endl;
//
//    data_file.close();
//    std::cerr << "Wrote trajectory to " << filename << std::endl;
//  }
//
//  void PlanKick::exportSparseKick(std::string filename)
//  {
//    std::ofstream data_file;
//    data_file.open(filename.c_str());
//
//    std::vector<double> inverted_joints = {1.0, -1.0, 1.0, -1.0, 1.0, 1.0, 1.0, -1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, -1.0, 1.0, 1.0, 1.0, -1.0, -1.0, -1.0, -1.0};
//
//    //write joints
//    data_file << "---" << std::endl;
//    data_file << "keyframes:" << std::endl;
//    for(unsigned int i = 0; i < m_sparse_joint_plan.size(); i++)
//    {
//      data_file << "  - name: move " << i << std::endl;
//      data_file << "    frames: " << m_sparse_joint_plan[i].first << std::endl;
//      data_file << "    joints: " << std::endl;
//      for(unsigned int j = 0; j < (m_sparse_joint_plan[i].second.size() - 4); j++)
//      {
//        data_file << "      " << m_joint_names.at(j) << ": " << m_sparse_joint_plan[i].second[j] * (180.0 / M_PI) * inverted_joints[j] << std::endl;
//      }
//    }
//    data_file << "..." << std::endl;
//
//    data_file.close();
//    std::cerr << "Wrote trajectory to " << filename << std::endl;
//  }

  void PlanKick::spin()
  {
    ROS_INFO("%s started.", ros::this_node::getName().c_str());
    ros::Rate loop_rate(m_frame_rate);
    unsigned int plan_idx = 0;
    std::vector<std::vector<double> > joint_plan = m_kack->getJointPlan();
    std::vector<KACK::Point> com_plan = m_kack->getCoMPlan();
    while(ros::ok())
    {
      m_js.position = joint_plan[plan_idx % joint_plan.size()];
      KACK::Point com_point; // = com_plan[plan_idx % com_plan.size()];
      plan_idx++;

      geometry_msgs::PointStamped com;
      com.header.frame_id = "l_ankle";
      com.header.stamp = ros::Time::now();
      com.point.x = com_point.x;
      com.point.y = com_point.y;
      m_com_pub.publish(com);

      m_js.header.stamp = ros::Time::now();
      m_js_pub.publish(m_js);

      tf::StampedTransform trans = tf::StampedTransform(tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0, 0, 0)), m_js.header.stamp, "world", "torso");
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
