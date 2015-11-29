#include "motion_planning/plan_kick.h"

namespace motion_planning
{
  visualization_msgs::Marker makeBox(visualization_msgs::InteractiveMarker &msg)
  {
    visualization_msgs::Marker marker;

    marker.type = visualization_msgs::Marker::CUBE;
    marker.scale.x = msg.scale * 0.45;
    marker.scale.y = msg.scale * 0.45;
    marker.scale.z = msg.scale * 0.45;
    marker.color.r = 0.5;
    marker.color.g = 0.5;
    marker.color.b = 0.5;
    marker.color.a = 1.0;

    return marker;
  }

  visualization_msgs::InteractiveMarkerControl& makeBoxControl(visualization_msgs::InteractiveMarker &msg)
  {
    visualization_msgs::InteractiveMarkerControl control;
    control.always_visible = true;
    control.markers.push_back(makeBox(msg));
    msg.controls.push_back(control);

    return msg.controls.back();
  }

  inline geometry_msgs::Pose matrixToPose(dynamics_tree::Matrix4 mat)
  {
    dynamics_tree::Matrix3 E = mat.topLeftCorner(3, 3);
    Eigen::Quaterniond quat(E);
    dynamics_tree::Vector3 r = mat.topRightCorner(3, 1);

    geometry_msgs::Pose pose;
    pose.position.x = r(0);
    pose.position.y = r(1);
    pose.position.z = r(2);
    pose.orientation.w = quat.w();
    pose.orientation.x = quat.x();
    pose.orientation.y = quat.y();
    pose.orientation.z = quat.z();

    return pose;
  }

  PlanKick::PlanKick(const ros::NodeHandle& nh) :
      m_nh(nh), m_server("/kick_setpoint_server")
  {
    m_nh.param("loop_rate", m_frame_rate, 100.0);
    m_nh.param("handle_mode", m_handle_mode, true);
    m_nh.param("world_frame", m_world_frame, std::string("world"));
//    m_nh.param("kick_filename", m_kick_filename, ros::package::getPath("motion_planning") + std::string("/kick_trajectory.py"));
//    m_nh.param("sparse_kick_filename", m_sparse_kick_filename, ros::package::getPath("motion_planning") + std::string("/../data/kicks/default.yaml"));

    m_kack.reset(new KACK::Kack(ros::package::getPath("motion_planning") + "/nao.urdf"));
    std::string initial_pos_string = "      0,      -0.4,            0,        0,    -0.436,      0.873,      -0.436,          0,            0,        0,    -0.436,      0.873,      -0.436,          0,            1.4,          0.3,         0,          -0.05,            1.4,          -0.3,         0,          0.05,         0,     0,         0,     0";

    m_js.name = m_kack->getJointNames();
    m_js.position = rpp::parameterStringToFPVector(initial_pos_string);
    m_js.velocity = std::vector<double>(m_js.name.size(), 0.0);
    m_js.effort = std::vector<double>(m_js.name.size(), 0.0);
    m_rootTworld = dynamics_tree::Matrix4::Identity();

    m_com_pub = m_nh.advertise<geometry_msgs::PointStamped>("/com", 1, true);
    m_js_pub = m_nh.advertise<sensor_msgs::JointState>("/joint_states", 1, true);

    if(m_handle_mode)
    {
      m_kack->tree(true).kinematics(m_js.position, m_rootTworld);

      dynamics_tree::Matrix4 worldTsole;
      m_kack->tree(true).lookupTransform("world", "r_sole", worldTsole);
      m_foot_target = matrixToPose(worldTsole);
      makeFootMarker(m_foot_target);

      dynamics_tree::Vector4 com;
      m_kack->tree(true).centerOfMass("world", com);
      m_com_target.orientation.w = 1.0;
      m_com_target.position.x = com(0);
      m_com_target.position.y = com(1);
      m_com_target.position.z = com(2);
      makeCoMMarker(m_com_target);
    }
    else
    {
      //test plan
      double foot_separation = 0.1;
      double ppt = 1e-3;
      double pot = 1e-3;
      double cpt = 1e-3;
      double czt = 1.0;

      KACK::Pose start(0.0, -foot_separation, 0.0, 0.0, 0.0, 0.0);
      KACK::Pose lift(0.0, -foot_separation, 0.05, 0.0, 0.0, 0.0);

      KACK::Point com_center(0.0, -foot_separation / 2.0, 0.22);
      KACK::Point com_support(0.0, 0.0, 0.4);

      std::vector<KACK::CartesianKeyframe> keyframes;
      keyframes.push_back(KACK::CartesianKeyframe(3.0, start, com_support, ppt, pot, cpt, czt));
      keyframes.push_back(KACK::CartesianKeyframe(3.0, lift, com_support, ppt, pot, cpt, czt));
      keyframes.push_back(KACK::CartesianKeyframe(3.0, start, com_support, ppt, pot, cpt, czt));
      keyframes.push_back(KACK::CartesianKeyframe(3.0, start, com_center, ppt, pot, cpt, czt));
      m_kack->plan(m_js.position, keyframes, true, false);

//    exportKick(m_kick_filename);
//    exportSparseKick(m_sparse_kick_filename);
    }
  }

  PlanKick::~PlanKick()
  {
  }

  void PlanKick::makeFootMarker(geometry_msgs::Pose initial_pose)
  {
    std::cerr << "Initial foot pose: " << initial_pose << std::endl;

    m_foot_marker.header.frame_id = "l_sole";
    m_foot_marker.description = m_foot_marker.name;
    m_foot_marker.pose = initial_pose;

    m_foot_control.always_visible = true;
    m_foot_control.orientation_mode = visualization_msgs::InteractiveMarkerControl::INHERIT;

    m_foot_control.orientation.w = 1;
    m_foot_control.orientation.x = 1;
    m_foot_control.orientation.y = 0;
    m_foot_control.orientation.z = 0;
    m_foot_control.name = "rotate_x";
    m_foot_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    m_foot_marker.controls.push_back(m_foot_control);
    m_foot_control.name = "move_x";
    m_foot_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    m_foot_marker.controls.push_back(m_foot_control);

    m_foot_control.orientation.w = 1;
    m_foot_control.orientation.x = 0;
    m_foot_control.orientation.y = 1;
    m_foot_control.orientation.z = 0;
    m_foot_control.name = "rotate_z";
    m_foot_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    m_foot_marker.controls.push_back(m_foot_control);
    m_foot_control.name = "move_z";
    m_foot_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    m_foot_marker.controls.push_back(m_foot_control);

    m_foot_control.orientation.w = 1;
    m_foot_control.orientation.x = 0;
    m_foot_control.orientation.y = 0;
    m_foot_control.orientation.z = 1;
    m_foot_control.name = "rotate_y";
    m_foot_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    m_foot_marker.controls.push_back(m_foot_control);
    m_foot_control.name = "move_y";
    m_foot_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    m_foot_marker.controls.push_back(m_foot_control);

    m_foot_marker.scale = 0.1;

    m_server.insert(m_foot_marker, boost::bind(&PlanKick::processFootFeedback, this, _1));
    m_server.applyChanges();
  }

  void PlanKick::makeCoMMarker(geometry_msgs::Pose initial_pose)
  {
    std::cerr << "Initial com pose: " << initial_pose << std::endl;

    m_com_marker.header.frame_id = "l_sole";
    m_com_marker.name = "com setpoint";
    m_com_marker.description = m_com_marker.name;
    m_com_marker.pose = initial_pose;

    m_com_control.always_visible = true;
    m_com_control.orientation_mode = visualization_msgs::InteractiveMarkerControl::INHERIT;

    m_com_control.orientation.w = 1;
    m_com_control.orientation.x = 1;
    m_com_control.orientation.y = 0;
    m_com_control.orientation.z = 0;
    m_com_control.name = "move_x";
    m_com_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    m_com_marker.controls.push_back(m_com_control);

    m_com_control.orientation.w = 1;
    m_com_control.orientation.x = 0;
    m_com_control.orientation.y = 1;
    m_com_control.orientation.z = 0;
    m_com_control.name = "move_z";
    m_com_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    m_com_marker.controls.push_back(m_com_control);

    m_com_control.orientation.w = 1;
    m_com_control.orientation.x = 0;
    m_com_control.orientation.y = 0;
    m_com_control.orientation.z = 1;
    m_com_control.name = "move_y";
    m_com_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    m_com_marker.controls.push_back(m_com_control);

    m_com_marker.scale = 0.1;

    m_server.insert(m_com_marker, boost::bind(&PlanKick::processCoMFeedback, this, _1));
    m_server.applyChanges();
  }

  void PlanKick::processFootFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
  {
    m_foot_target = feedback->pose;
  }

  void PlanKick::processCoMFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
  {
    m_com_target = feedback->pose;
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

  void PlanKick::doControl()
  {
    double R, P, Y;
    tf::Quaternion tf_quat;
    tf::quaternionMsgToTF(m_foot_target.orientation, tf_quat);
    tf::Matrix3x3(tf_quat).getRPY(R, P, Y);

    KACK::Pose kack_foot_pose(m_foot_target.position.x, m_foot_target.position.y, m_foot_target.position.z, R, P, Y);
    KACK::Point kack_com_point(m_com_target.position.x, m_com_target.position.y, m_com_target.position.z);
    KACK::CartesianKeyframe k(3.0, kack_foot_pose, kack_com_point, 1e-3, 1e-3, 1e-3, 1e3);

    m_kack->moveFoot(m_frame_rate, true, k, m_js.position, KACK::FootSensor(), KACK::FootSensor(), m_js.position, 0, 0, 0, 0, 0, 0, false);
  }

  void PlanKick::spin()
  {
    ROS_INFO("%s started.", ros::this_node::getName().c_str());
    ros::Rate loop_rate(m_frame_rate);
    unsigned int plan_idx = 0;
    std::vector<std::vector<double> > joint_plan = m_kack->getJointPlan();
    std::vector<KACK::Point> com_plan = m_kack->getCoMPlan();
    while(ros::ok())
    {
      if(m_handle_mode)
      {
        doControl();
      }
      else
      {
        m_js.position = joint_plan[plan_idx % joint_plan.size()];
        KACK::Point com_point; // = com_plan[plan_idx % com_plan.size()];
        plan_idx++;
      }

      m_kack->tree(true).kinematics(m_js.position, m_rootTworld);
      dynamics_tree::Vector4 com_vector;
      m_kack->tree(true).centerOfMass("l_sole", com_vector);
      geometry_msgs::PointStamped com;
      com.header.frame_id = "l_sole";
      com.header.stamp = ros::Time::now();
      com.point.x = com_vector(0);
      com.point.y = com_vector(1);
      com.point.z = com_vector(2);
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
