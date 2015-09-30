#include "motion_planning/plan_kick.h"

namespace motion_planning
{
  PlanKick::PlanKick(const ros::NodeHandle& nh) :
      m_nh(nh)
  {
    m_nh.param("loop_rate", m_frame_rate, 30.0);
    m_nh.param("lift_height", m_lift_height, 0.04);
    m_nh.param("kick_time", m_kick_time, 0.5);
    m_nh.param("kick_dist", m_kick_dist, 0.1);
    m_nh.param("kick_filename", m_kick_filename, std::string("kick_trajectory.py"));
    m_nh.param("sparse_kick_filename", m_sparse_kick_filename, std::string("kick_trajectory.yaml"));
                                      
    std::string joint_string =       "HeadYaw, HeadPitch, LHipYawPitch, LHipRoll, LHipPitch, LKneePitch, LAnklePitch, LAnkleRoll, RHipYawPitch, RHipRoll, RHipPitch, RKneePitch, RAnklePitch, RAnkleRoll, LShoulderPitch, LShoulderRoll, LElbowYaw, LElbowRoll, RShoulderPitch, RShoulderRoll, RElbowYaw, RElbowRoll, LWristYaw, LHand, RWristYaw, RHand";
    std::string initial_pos_string = "      0,      -0.4,            0,        0,    -0.436,      0.873,      -0.436,          0,            0,        0,    -0.436,      0.873,      -0.436,          0,            1.4,          0.3,         0,          -0.05,            1.4,          -0.3,         0,          0.05,         0,     0,         0,     0";
    m_joint_names = matec_utils::parameterStringToStringVector(joint_string);
    std::string urdf_path = ros::package::getPath("motion_planning") + "/nao.urdf";
    m_urdf_model.initFile(urdf_path);

    for(unsigned int i = 0; i < (m_joint_names.size()-4); i++)
    {
//      if(m_joint_names.at(i) == "LElbowYaw" || m_joint_names.at(i) == "LElbowRoll" || m_joint_names.at(i) == "RElbowYaw" || m_joint_names.at(i) == "RElbowRoll")
//      {
//        continue;
//      }

      std::cerr << "joint " << i << " is " << m_joint_names.at(i) << std::endl;
      m_joint_ids.push_back(i);

      if(m_urdf_model.getJoint(m_joint_names.at(i))->limits)
      {
        m_joint_mins.push_back(m_urdf_model.getJoint(m_joint_names.at(i))->limits->lower);
        m_joint_maxes.push_back(m_urdf_model.getJoint(m_joint_names.at(i))->limits->upper);
        m_joint_vels.push_back(m_urdf_model.getJoint(m_joint_names.at(i))->limits->velocity);
      }
    }

    m_graph.loadFromURDF(m_joint_names, m_urdf_model);
    m_graph.print("l_ankle");
    m_graph.spawnDynamicsTree("l_ankle", false, m_tree);

    m_js.name = m_joint_names;
    m_js.position = matec_utils::parameterStringToFPVector(initial_pos_string);
    m_js.velocity = std::vector<double>(m_joint_names.size(), 0.0);
    m_js.effort = std::vector<double>(m_joint_names.size(), 0.0);
    m_rootTworld = matec_utils::Matrix4::Identity();

    m_odom.pose.orientation.w = 1.0;

    m_com_pub = m_nh.advertise<geometry_msgs::PointStamped>("/com", 1, true);
    m_js_pub = m_nh.advertise<sensor_msgs::JointState>("/joint_states", 1, true);

    m_joint_plan.push_back(m_js.position);
    m_sparse_joint_plan.push_back(std::pair<unsigned int, std::vector<double> >(100, m_joint_plan.at(m_joint_plan.size() - 1)));
    double foot_separation = 0.1;
    std::cerr << "Planning shift" << std::endl;
    planMove(0.0, -foot_separation, 0.0, 0.0, 0.01, 0.0, 3.0);
    std::cerr << "Planning lift" << std::endl;
    planMove(0.0, -foot_separation, m_lift_height, -0.1, 0.01, 0.0, 0.3);
    std::cerr << "Planning kick" << std::endl;
    planMove(m_kick_dist, -foot_separation, m_lift_height, -0.1, 0.01, 0.0, 0.5);

    std::cerr << "Planning lunge" << std::endl;
    planMove(m_kick_dist, -foot_separation, -0.0, 0.0, 0.04, -0.02, 0.5);
    std::cerr << "Planning delay" << std::endl;
    planMove(m_kick_dist, -foot_separation, -0.0, 0.0, 0.08, -0.05, 3.0);

    // std::cerr << "Planning return" << std::endl;
    // planMove(0.0, -foot_separation, m_lift_height, 0.025, 0.0, 0.5);
    // std::cerr << "Planning set" << std::endl;
    // planMove(0.0, -foot_separation, 0.0, 0.025, 0.0, 0.5);
    // std::cerr << "Planning balance" << std::endl;
    // planMove(0.0, -foot_separation, 0.0, 0.025, -0.05, 1.25);

    exportKick(m_kick_filename);
    exportSparseKick(m_sparse_kick_filename);
  }

  PlanKick::~PlanKick()
  {
  }

  inline void matrixToRPY(matec_utils::Matrix3 rot, double&roll, double&pitch, double&yaw)
  {
    double epsilon = 1E-12;
    pitch = atan2((double) -rot(2, 0), sqrt((double) (rot(0, 0) * rot(0, 0) + rot(1, 0) * rot(1, 0))));
    if(fabs(pitch) > (M_PI / 2.0 - epsilon))
    {
      yaw = atan2((double) -rot(0, 1), (double) rot(1, 1));
      roll = 0.0;
    }
    else
    {
      roll = atan2((double) rot(2, 1), (double) rot(2, 2));
      yaw = atan2((double) rot(1, 0), (double) rot(0, 0));
    }
  }

  //assumes that f1 and f2 are expressed w.r.t. the same frame (common)
  //returns a twist in the common frame that takes you from f1 to f2 in dt seconds
  inline matec_utils::Vector6 frameTwist(matec_utils::Matrix4 commonTf1, matec_utils::Matrix4 commonTf2, double dt, double& dx, double& dy, double& dz, double& dR, double& dP, double& dY)
  {
    matec_utils::Matrix4 delta = commonTf1.inverse() * commonTf2;
    Eigen::AngleAxis<double> angle_axis((matec_utils::Matrix3) delta.topLeftCorner(3, 3));

    matec_utils::Vector6 twist; //expressed here in f1 coordinates

    matrixToRPY((matec_utils::Matrix3) delta.topLeftCorner(3, 3), dR, dP, dY);
    dx = delta(0, 3);
    dy = delta(1, 3);
    dz = delta(2, 3);
    twist(0) = dR / dt;
    twist(1) = dP / dt;
    twist(2) = dY / dt;
    twist(3) = dx / dt;
    twist(4) = dy / dt;
    twist(5) = dz / dt;

    //transform back to common
    twist.topRows(3) = commonTf1.topLeftCorner(3, 3) * twist.topRows(3);
    twist.bottomRows(3) = commonTf1.topLeftCorner(3, 3) * twist.bottomRows(3);

    return twist;
  }

  void PlanKick::comRecursive(boost::shared_ptr<dynamics_tree::DynamicsTreeNode> node)
  {
    //forward pass
    node->supported_mass = 0.0;
    node->supported_com = matec_utils::Vector4::Zero();

    //recursion
    for(auto c : node->children)
    {
      comRecursive(c);
    }

    //backwards pass
    node->supported_mass += node->mass;
    node->supported_com += node->mass * node->iTicom.topRightCorner(3, 1);
    if(node->parent)
    {
      node->parent->supported_mass += node->supported_mass;
      node->parent->supported_com += node->supported_com;
    }
    node->supported_com /= node->supported_mass;
  }

  void PlanKick::comJacobian(std::vector<unsigned int> joint_indices, std::string goal_frame, matec_utils::Matrix& jacobian)
  {
    if(m_tree.getNodes().find(goal_frame) == m_tree.getNodes().end())
    {
      std::cerr << "Couldn't find goal frame " << goal_frame << " in the tree!" << std::endl;
      return;
    }

    boost::shared_ptr<dynamics_tree::DynamicsTreeNode> goal_node = m_tree.getNodes()[goal_frame];
    jacobian.resize(6, joint_indices.size());
    for(unsigned int i = 0; i < joint_indices.size(); i++)
    {
      boost::this_thread::interruption_point();
      boost::shared_ptr<dynamics_tree::DynamicsTreeNode> joint_node = m_tree.getNodeByIndex(joint_indices[i]);
      if(!joint_node)
      {
        std::cerr << "Couldn't look up joint at index " << joint_indices[i] << "!" << std::endl;
        return;
      }

      matec_utils::Matrix4 goalTpartial_com = matec_utils::Matrix4::Identity();
      goalTpartial_com.topRightCorner(3, 1) = joint_node->supported_com;
      goalTpartial_com = goal_node->iTO * goalTpartial_com.inverse();

      matec_utils::Matrix4 goalTjoint = goal_node->iTO * joint_node->iTO.inverse();
      goalTjoint.topRightCorner(3, 1) -= goalTpartial_com.topRightCorner(3, 1); //translate goal to tool frame
      matec_utils::Matrix6 goalXjoint = matec_utils::motionTransformFromAffine(goalTjoint);
      jacobian.block<6, 1>(0, i) = goalXjoint * joint_node->axis * joint_node->supported_mass;
    }

    jacobian /= m_tree.getTotalMass();
  }

  void PlanKick::planPose(std::vector<double> last_positions, std::vector<double>& next_positions, double foot_x, double foot_y, double foot_z, double foot_pitch, double com_x, double com_y, double dt, int maxiter)
  {
    next_positions = last_positions;

    matec_utils::Matrix4 leftTtarget = matec_utils::pureRotation(0.0, foot_pitch, 0.0);
    leftTtarget.topRightCorner(3, 1) << foot_x, foot_y, foot_z;

    for(int i = 0; i < maxiter; i++)
    {
      m_tree.kinematics(next_positions, m_odom);

      matec_utils::Matrix4 leftTright;
      m_tree.lookupTransform("l_ankle", "r_ankle", leftTright);

      matec_utils::Matrix J, JT, JJT;
      m_tree.jacobian(m_joint_ids, "l_ankle", "r_ankle", J);
      JT = J.transpose();
      JJT = J * JT;

      double twist_scale = 0.1;
      double dx, dy, dz, dR, dP, dY;
      matec_utils::Vector6 foot_twist = frameTwist(leftTright, leftTtarget, dt, dx, dy, dz, dR, dP, dY);
      foot_twist *= twist_scale;

      //transpose
      //dQ = J^T * e * (<e, J*J^T*e> / <J*J^T*e, J*J^T*e>)
//      matec_utils::Vector JJTe = JJT * foot_twist;
//      boost::this_thread::interruption_point();
//      matec_utils::Vector trans_velocities = JT * foot_twist * (foot_twist.transpose() * JJTe) / (JJTe.transpose() * JJTe);
//      boost::this_thread::interruption_point();

      //pinv
      matec_utils::Vector pinv_velocities = JT * JJT.inverse() * foot_twist;

      //===========COM==================
      geometry_msgs::PointStamped com;
      m_tree.centerOfMass("l_ankle", com);
      double com_dx = com_x - com.point.x;
      double com_dy = com_y - com.point.y;

      matec_utils::Matrix Jcom, JcomT, JcomJcomT;
      comRecursive(m_tree.getRootNode());
      comJacobian(m_joint_ids, "l_ankle", Jcom);
      JcomT = Jcom.transpose();
      JcomJcomT = Jcom * JcomT;

      matec_utils::Vector6 com_twist = matec_utils::Vector6::Zero();
      com_twist(3) = com_dx / dt;
      com_twist(4) = com_dy / dt;
      com_twist(5) = 0.0;
      com_twist *= twist_scale;

      //transpose
//      matec_utils::Vector JJTe_com = JcomJcomT * com_twist;
//      boost::this_thread::interruption_point();
//      matec_utils::Vector com_trans_velocities = JcomT * com_twist * (com_twist.transpose() * JJTe_com) / (JJTe_com.transpose() * JJTe_com);
//      boost::this_thread::interruption_point();

      //pinv
      matec_utils::Vector com_pinv_velocities = JcomT * JcomJcomT.inverse() * com_twist;

      //check if done
      double eps = 1.0e-3;
      if(fabs(com_dx) < eps && fabs(com_dy) < eps && fabs(dx) < eps && fabs(dy) < eps && fabs(dz) < eps && fabs(dR) < eps && fabs(dP) < eps && fabs(dY) < eps)
      {
        std::cerr << ".";
        return;
      }

      //simulate forward (euler)
      for(unsigned int j = 0; j < m_joint_ids.size() && ros::ok(); j++)
      {
        unsigned int joint_id = m_joint_ids[j];
        unsigned int num_velocities = 0;
        double combined_joint_velocity = 0.0;
//        if(trans_velocities[j] != 0.0)
//        {
//          combined_joint_velocity += trans_velocities[j];
//          num_velocities++;
//        }
        if(pinv_velocities[j] != 0.0)
        {
          combined_joint_velocity += pinv_velocities[j];
          num_velocities++;
        }
//        if(com_trans_velocities[j] != 0.0)
//        {
//          combined_joint_velocity += com_trans_velocities[j];
//          num_velocities++;
//        }
        if(com_pinv_velocities[j] != 0.0)
        {
          combined_joint_velocity += com_pinv_velocities[j];
          num_velocities++;
        }

        double local_min = std::max((double) m_joint_mins[j], (double) (last_positions[joint_id] - m_joint_vels[j] * dt));
        double local_max = std::min((double) m_joint_maxes[j], (double) (last_positions[joint_id] + m_joint_vels[j] * dt));

        next_positions.at(joint_id) += dt * combined_joint_velocity / (double) num_velocities;
        next_positions.at(joint_id) = matec_utils::clamp(next_positions.at(joint_id), local_min, local_max);
      }
    }

    std::cerr << "Failed to solve!" << std::endl;
  }

  void PlanKick::planMove(double xf, double yf, double zf, double pf, double cxf, double cyf, double dt)
  {
    m_tree.kinematics(m_joint_plan.at(m_joint_plan.size() - 1), m_odom);

    matec_utils::Matrix4 leftTright;
    m_tree.lookupTransform("l_ankle", "r_ankle", leftTright);

    double R, P, Y;
    matrixToRPY((matec_utils::Matrix3) leftTright.topLeftCorner(3, 3), R, P, Y);

    geometry_msgs::PointStamped com;
    m_tree.centerOfMass("l_ankle", com);

    unsigned int num_frames = std::ceil(dt * m_frame_rate);
    for(unsigned int i = 1; i <= num_frames; i++)
    {
      double x = leftTright(0, 3) + i * (xf - leftTright(0, 3)) / (double) num_frames;
      double y = leftTright(1, 3) + i * (yf - leftTright(1, 3)) / (double) num_frames;
      double z = leftTright(2, 3) + i * (zf - leftTright(2, 3)) / (double) num_frames;
      double p = P + i * (pf - P) / (double) num_frames;
      double cx = com.point.x + i * (cxf - com.point.x) / (double) num_frames;
      double cy = com.point.y + i * (cyf - com.point.y) / (double) num_frames;

      std::vector<double> next_positions = m_joint_plan.at(m_joint_plan.size() - 1);
      planPose(m_joint_plan.at(m_joint_plan.size() - 1), next_positions, x, y, z, p, cx, cy, 1.0 / m_frame_rate, 100000);
      m_joint_plan.push_back(next_positions);
    }
    std::cerr << std::endl;

    m_sparse_joint_plan.push_back(std::pair<unsigned int, std::vector<double> >(num_frames,m_joint_plan.at(m_joint_plan.size() - 1)));
  }

  void PlanKick::exportKick(std::string filename)
  {
    std::ofstream data_file;
    data_file.open(filename.c_str());

    //write joints
    data_file << "names = [";
    for(unsigned int i = 0; i < (m_joint_names.size() - 4); i++)
    {
      if(i != 0)
      {
        data_file << ", ";
      }
      data_file << "'" << m_joint_names.at(i) << "'";
    }
    data_file << "]" << std::endl;

    data_file << "trajectory = [";
    for(unsigned int i = 0; i < m_joint_plan.size(); i++)
    {
      if(i != 0)
      {
        data_file << ", ";
      }
      data_file << "[";
      for(unsigned int j = 0; j < (m_joint_names.size() - 4); j++)
      {
        if(j != 0)
        {
          data_file << ", ";
        }
        data_file << m_joint_plan[i][j];

        //sanity check
        if(m_joint_plan[i][j] > m_joint_maxes[j] || m_joint_plan[i][j] < m_joint_mins[j])
        {
          std::cerr << "BAD JOINT CLAMP! plan idx: " << i << " joint idx: " << j << " val: " << m_joint_plan[i][j] << " min: " << m_joint_mins[j] << " max: " << m_joint_maxes[j] << std::endl;
        }
      }
      data_file << "]";
    }
    data_file << "]" << std::endl;

    data_file.close();
    std::cerr << "Wrote trajectory to " << filename << std::endl;
  }

  void PlanKick::exportSparseKick(std::string filename)
  {
    std::ofstream data_file;
    data_file.open(filename.c_str());

    std::vector<double> inverted_joints = {1.0, -1.0, 1.0, -1.0, 1.0, 1.0, 1.0, -1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, -1.0, 1.0, 1.0, 1.0, -1.0, -1.0, -1.0, -1.0};
      
    //write joints
    data_file << "---" << std::endl;
    data_file << "keyframes:" << std::endl;
    for(unsigned int i = 0; i < m_sparse_joint_plan.size(); i++)
    {
      data_file << "  - name: move " << i << std::endl;
      data_file << "    frames: " << m_sparse_joint_plan[i].first << std::endl;
      data_file << "    joints: " << std::endl;
      for(unsigned int j = 0; j < (m_sparse_joint_plan[i].second.size() - 4); j++)
      {
        data_file << "      " << m_joint_names.at(j) << ": " << m_sparse_joint_plan[i].second[j]*(180.0/M_PI)*inverted_joints[j] << std::endl;
      }
    }
    data_file << "..." << std::endl;

    data_file.close();
    std::cerr << "Wrote trajectory to " << filename << std::endl;
  }

  void PlanKick::spin()
  {
    ROS_INFO("%s started.", ros::this_node::getName().c_str());
    ros::Rate loop_rate(m_frame_rate);
    unsigned int plan_idx = 0;
    while(ros::ok())
    {
      m_js.position = m_joint_plan[plan_idx % m_joint_plan.size()];
      plan_idx++;

      geometry_msgs::PointStamped com;
      m_tree.kinematics(m_js.position, m_odom);
      m_tree.centerOfMass("l_ankle", com);
      m_com_pub.publish(com);

      m_js.header.stamp = ros::Time::now();
      m_js_pub.publish(m_js);

      tf::StampedTransform trans = matec_utils::matrixToStampedTransform(m_rootTworld, "world", m_urdf_model.getRoot()->name);
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
