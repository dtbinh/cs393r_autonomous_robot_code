#ifndef KACK_HPP
#define KACK_HPP

#include "matec/dynamics_graph.h"

namespace KACK
{
  class Point
  {
  public:
    double x;
    double y;
    double z;

    Point()
    {
      x = 0;
      y = 0;
      z = 0;
    }

    Point(double x_, double y_, double z_)
    {
      x = x_;
      y = y_;
      z = z_;
    }

    void update(double a, double b, double c)
    {
      x = a;
      y = b;
      z = c;
    }
  };

  class Pose
  {
  public:
    //int Kick_foot_state; //0:double stand; 1:Right; 2: Left;
    double x;
    double y;
    double z;
    double R; //=PI to +PI
    double P; //=PI to +PI
    double Y; //=PI to +PI

    Pose()
    {
      x = 0;
      y = 0;
      z = 0;
      R = 0;
      P = 0;
      Y = 0;
    }

    Pose(double x_, double y_, double z_, double R_, double P_, double Y_)
    {
      x = x_;
      y = y_;
      z = z_;
      R = R_;
      P = P_;
      Y = Y_;
    }

    void update(double a, double b, double c, double roll, double pitch, double yaw)
    {
      //Kick_foot_state = kick_foot_s;
      x = a;
      y = b;
      z = c;
      R = roll;
      P = pitch;
      Y = yaw;
    }
  };

  class CartesianKeyframe
  {
  public:
    double duration; //how long the keyframe should last, in seconds
    Pose min_pose; // min/max pose allow the specification of tolerances on each axis. Set them to the same pose for zero tolerance
    Pose max_pose; //poses are specified in the coordinate frame of the supporting foot
    Point min_com;
    Point max_com;

    CartesianKeyframe()
    {
    }

    CartesianKeyframe(double d, Pose pose_min, Pose pose_max, Point com_min, Point com_max)
    {
      duration = d;
      min_pose = pose_min;
      max_pose = pose_max;
      min_com = com_min;
      max_com = com_max;
    }

    CartesianKeyframe(double d, Pose pose, Point com, double pose_pos_tol, double pose_ori_tol, double com_xy_pos_tol, double com_z_pos_tol)
    {
      duration = d;

      min_pose = pose;
      min_pose.x -= pose_pos_tol / 2.0;
      min_pose.y -= pose_pos_tol / 2.0;
      min_pose.z -= pose_pos_tol / 2.0;
      min_pose.R -= pose_ori_tol / 2.0;
      min_pose.P -= pose_ori_tol / 2.0;
      min_pose.Y -= pose_ori_tol / 2.0;

      max_pose = pose;
      max_pose.x += pose_pos_tol / 2.0;
      max_pose.y += pose_pos_tol / 2.0;
      max_pose.z += pose_pos_tol / 2.0;
      max_pose.R += pose_ori_tol / 2.0;
      max_pose.P += pose_ori_tol / 2.0;
      max_pose.Y += pose_ori_tol / 2.0;

      min_com = com;
      min_com.x -= com_xy_pos_tol / 2.0;
      min_com.y -= com_xy_pos_tol / 2.0;
      min_com.z -= com_z_pos_tol / 2.0;

      max_com = com;
      max_com.x += com_xy_pos_tol / 2.0;
      max_com.y += com_xy_pos_tol / 2.0;
      max_com.z += com_z_pos_tol / 2.0;
    }

    void update(double d, Pose min_p, Pose max_p, Point min_c, Point max_c)
    {
      duration = d;
      min_pose = min_p;
      max_pose = max_p;
      min_com = min_c;
      max_com = max_c;
    }

    void getPoseCentroid(Pose& pose)
    {
      pose.x = (max_pose.x + min_pose.x) / 2.0;
      pose.y = (max_pose.y + min_pose.y) / 2.0;
      pose.z = (max_pose.z + min_pose.z) / 2.0;
      pose.R = (max_pose.R + min_pose.R) / 2.0;
      pose.P = (max_pose.P + min_pose.P) / 2.0;
      pose.Y = (max_pose.Y + min_pose.Y) / 2.0;
    }

    void getCoMCentroid(Point& com)
    {
      com.x = (max_com.x + min_com.x) / 2.0;
      com.y = (max_com.y + min_com.y) / 2.0;
      com.z = (max_com.z + min_com.z) / 2.0;
    }

    void getPoseTolerances(Pose& tol)
    {
      tol.x = fabs(max_pose.x - min_pose.x) / 2.0;
      tol.y = fabs(max_pose.y - min_pose.y) / 2.0;
      tol.z = fabs(max_pose.z - min_pose.z) / 2.0;
      tol.R = fabs(max_pose.R - min_pose.R) / 2.0;
      tol.P = fabs(max_pose.P - min_pose.P) / 2.0;
      tol.Y = fabs(max_pose.Y - min_pose.Y) / 2.0;
    }

    void getCoMTolerances(Point& tol)
    {
      tol.x = fabs(max_com.x - min_com.x) / 2.0;
      tol.y = fabs(max_com.y - min_com.y) / 2.0;
      tol.z = fabs(max_com.z - min_com.z) / 2.0;
    }
  };

  class FootSensor
  {
  public:
    double fl;
    double fr;
    double rl;
    double rr;

    void update(double a, double b, double c, double d)
    {
      fl = a;
      fr = b;
      rl = c;
      rr = d;
    }
  };

  class Kack
  {
  public:
    Kack(std::string model_filename, double planning_rate = 30.0)
    {
      m_planning_rate = planning_rate;
      m_xml_model.fromFile(model_filename);
      std::string joint_string = "HeadYaw, HeadPitch, LHipYawPitch, LHipRoll, LHipPitch, LKneePitch, LAnklePitch, LAnkleRoll, RHipYawPitch, RHipRoll, RHipPitch, RKneePitch, RAnklePitch, RAnkleRoll, LShoulderPitch, LShoulderRoll, LElbowYaw, LElbowRoll, RShoulderPitch, RShoulderRoll, RElbowYaw, RElbowRoll, LWristYaw, LHand, RWristYaw, RHand";
      m_joint_names = rpp::parameterStringToStringVector(joint_string);
      m_rootTworld = dynamics_tree::Matrix4::Identity();

//      std::cerr << "Had " << m_joint_names.size() << " joint names" << std::endl;

      m_graph.loadFromURDF(m_joint_names, m_xml_model);
//      m_graph.print("l_ankle");
      m_graph.spawnDynamicsTree("l_ankle", false, m_left_tree);
      m_graph.spawnDynamicsTree("r_ankle", false, m_right_tree);

      for(unsigned int i = 0; i < (m_joint_names.size() - 4); i++)
      {
        if(m_joint_names.at(i) == "LElbowYaw" || m_joint_names.at(i) == "LElbowRoll" || m_joint_names.at(i) == "RElbowYaw" || m_joint_names.at(i) == "RElbowRoll")
        {
          continue;
        }

//        std::cerr << "joint " << i << " is " << m_joint_names.at(i) << std::endl;
        m_joint_ids.push_back(i);

        pugi::xml_node joint_xml = m_xml_model.component("joint", m_joint_names.at(i))->xml;

        if(joint_xml && joint_xml.child("limit"))
        {
          m_joint_mins.push_back(joint_xml.child("limit").attribute("lower").as_double());
          m_joint_maxes.push_back(joint_xml.child("limit").attribute("upper").as_double());
          m_joint_vels.push_back(joint_xml.child("limit").attribute("velocity").as_double());
        }
//        else
//        {
//          std::cerr << "joint " << m_joint_names.at(i) << " had no limits!" << std::endl;
//        }
      }
//      std::cerr << "Had " << m_joint_ids.size() << " joint ids" << std::endl;
//      std::cerr << "Had " << m_joint_maxes.size() << " joint limits" << std::endl;
    }

    ~Kack()
    {

    }

    void invertJoints(std::vector<double>& joints)
    {
      for(unsigned int i = 0; i < joints.size(); i++)
        joints[i] *= m_inverted_joints[i];
    }

    //first keyframe should be at the foot's pose in the nominal standing position
    bool plan(std::vector<double> starting_joint_positions, std::vector<CartesianKeyframe> keyframes, bool left_foot_supporting = true)
    {
      invertJoints(starting_joint_positions);

      m_joint_plan.push_back(starting_joint_positions);
      dynamics_tree::DynamicsTree& tree = left_foot_supporting? m_left_tree : m_right_tree;
      std::string supporting = left_foot_supporting? "l_ankle" : "r_ankle";
      std::string controlled = left_foot_supporting? "r_ankle" : "l_ankle";

      for(unsigned int k = 0; k < keyframes.size(); k++)
      {
        std::cerr << "Working on keyframe idx " << k + 1 << "/" << keyframes.size() << std::endl;
        tree.kinematics(m_joint_plan.at(m_joint_plan.size() - 1), m_rootTworld);
        dynamics_tree::Matrix4 leftTright;
        tree.lookupTransform(supporting, controlled, leftTright);
        double xi = leftTright(0, 3);
        double yi = leftTright(1, 3);
        double zi = leftTright(2, 3);

        double Ri, Pi, Yi;
        matrixToRPY((dynamics_tree::Matrix3) leftTright.topLeftCorner(3, 3), Ri, Pi, Yi);

        dynamics_tree::Vector4 com_vector;
        tree.centerOfMass(supporting, com_vector);

        printf("started at (%g,%g,%g)(%g,%g,%g), com(%g,%g,%g)\n", xi, yi, zi, Ri, Pi, Yi, com_vector(0), com_vector(1), com_vector(2));

        Pose pc, pt;
        Point cc, ct;
        keyframes[k].getPoseCentroid(pc);
        keyframes[k].getPoseTolerances(pt);
        keyframes[k].getCoMCentroid(cc);
        keyframes[k].getCoMTolerances(ct);

        unsigned int num_frames = std::ceil(keyframes[k].duration * m_planning_rate);
        for(unsigned int i = 1; i <= num_frames; i++)
        {
          double x = xi + i * (pc.x - xi) / (double) num_frames;
          double y = yi + i * (pc.y - yi) / (double) num_frames;
          double z = zi + i * (pc.z - zi) / (double) num_frames;
          double R = Ri + i * (pc.R - Ri) / (double) num_frames;
          double P = Pi + i * (pc.P - Pi) / (double) num_frames;
          double Y = Yi + i * (pc.Y - Yi) / (double) num_frames;

          double cx = com_vector(0) + i * (cc.x - com_vector(0)) / (double) num_frames;
          double cy = com_vector(1) + i * (cc.y - com_vector(1)) / (double) num_frames;
          double cz = com_vector(2) + i * (cc.z - com_vector(2)) / (double) num_frames;

          CartesianKeyframe interpolated = CartesianKeyframe((keyframes[k].duration / num_frames), Pose(x - pt.x, y - pt.y, z - pt.z, R - pt.R, P - pt.P, Y - pt.Y), Pose(x + pt.x, y + pt.y, z + pt.z, R + pt.R, P + pt.P, Y + pt.Y), Point(cx - ct.x, cy - ct.y, cz - ct.z), Point(cx + ct.x, cy + ct.y, cz + ct.z));

          std::vector<double> next_positions = m_joint_plan.at(m_joint_plan.size() - 1);
          planPose(tree, supporting, controlled, m_joint_plan.at(m_joint_plan.size() - 1), next_positions, interpolated, 10000);
          m_joint_plan.push_back(next_positions);
        }
        std::cerr << std::endl;
      }
      m_start_time = 0.0;
      return true;
    }

    //current_time in seconds, command is output joint positions
    void execute(double current_time, std::vector<double> current_joint_positions, FootSensor left_foot, FootSensor right_foot, std::vector<double>& command)
    {
      if(m_start_time == 0.0)
      {
        m_start_time = current_time;
      }
      invertJoints(current_joint_positions);
      
      double dt = current_time - m_start_time;
      unsigned long current_slice = dt * m_planning_rate;
      if(current_slice < m_joint_plan.size())
        command = m_joint_plan.at(current_slice);
      else
        command = m_joint_plan.back();
      invertJoints(command);
    }

    std::vector<std::string> getJointNames()
    {
      return m_joint_names;
    }

    std::vector<std::vector<double> > getJointPlan()
    {
      return m_joint_plan;
    }

    std::vector<KACK::Point> getCoMPlan()
    {
      return m_com_plan;
    }

  private:
    std::vector<std::string> m_joint_names;
    rpp::ComponentTree m_xml_model;
    dynamics_tree::DynamicsGraph m_graph;
    dynamics_tree::DynamicsTree m_left_tree;
    dynamics_tree::DynamicsTree m_right_tree;
    std::vector<double> m_inverted_joints = {1.0, -1.0, 1.0, -1.0, 1.0, 1.0, 1.0, -1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, -1.0, 1.0, 1.0, 1.0, -1.0, -1.0, -1.0, -1.0, 0.0, 0.0, 0.0, 0.0};

    std::vector<unsigned int> m_joint_ids;
    std::vector<double> m_joint_mins;
    std::vector<double> m_joint_maxes;
    std::vector<double> m_joint_vels;

    dynamics_tree::Matrix4 m_rootTworld;

    double m_planning_rate;
    std::vector<std::vector<double> > m_joint_plan;
    std::vector<Point> m_com_plan;

    double m_start_time;

    inline void processState(dynamics_tree::DynamicsTree& tree, std::vector<double>& positions, dynamics_tree::Matrix4& rootTworld)
    {
      tree.getRootNode()->iTO = rootTworld;
      tree.getRootNode()->iXO = dynamics_tree::motionTransformFromAffine(tree.getRootNode()->iTO);
      processStateRecursive(tree.getRootNode(), positions);
    }

    void processStateRecursive(boost::shared_ptr<dynamics_tree::DynamicsTreeNode> node, std::vector<double>& positions)
    {
      //forward pass
      node->supported_mass = 0.0;
      node->supported_com = dynamics_tree::Vector4::Zero();
      if(node->parent)
      {
        double q = 0.0;
        if(node->joint_type != dynamics_tree::FIXED_JOINT)
        {
          q = positions.at(node->joint_idx);
        }
        if(q != node->q) //don't repeat past work
        {
          node->q = q;
          dynamics_tree::Matrix3 iEip = Eigen::AngleAxisd(node->q, node->axis.topRows(3)).toRotationMatrix().inverse() * node->E_fixed;

          node->iXip.topLeftCorner(3, 3) = iEip;
          // node->iXip.topRightCorner(3, 3) = Matrix3::Zero();
          node->iXip.bottomLeftCorner(3, 3) = -iEip * dynamics_tree::cross(node->r_fixed);
          node->iXip.bottomRightCorner(3, 3) = iEip;

          node->iTip.topLeftCorner(3, 3) = iEip;
          node->iTip.topRightCorner(3, 1) = -iEip * node->r_fixed;
          // node->iTip.bottomLeftCorner(1, 3) = 0.0;
          // node->iTip.bottomRightCorner(1, 1) = 1.0;
        }
        node->iXO = node->iXip * node->parent->iXO;
        node->iTO = node->iTip * node->parent->iTO;
      }

      //recursion
      for(auto c : node->children)
      {
        processStateRecursive(c, positions);
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

    void comJacobian(dynamics_tree::DynamicsTree& tree, std::vector<unsigned int> joint_indices, std::string goal_frame, dynamics_tree::Matrix& jacobian)
    {
      if(tree.getNodes().find(goal_frame) == tree.getNodes().end())
      {
        std::cerr << "Couldn't find goal frame " << goal_frame << " in the tree!" << std::endl;
        return;
      }

      boost::shared_ptr<dynamics_tree::DynamicsTreeNode> goal_node = tree.getNodes()[goal_frame];
      jacobian.resize(6, joint_indices.size());
      for(unsigned int i = 0; i < joint_indices.size(); i++)
      {
        //boost::this_thread::interruption_point();
        boost::shared_ptr<dynamics_tree::DynamicsTreeNode> joint_node = tree.getNodeByIndex(joint_indices[i]);
        if(!joint_node)
        {
          std::cerr << "Couldn't look up joint at index " << joint_indices[i] << "!" << std::endl;
          return;
        }

        dynamics_tree::Matrix4 goalTpartial_com = dynamics_tree::Matrix4::Identity();
        goalTpartial_com.topRightCorner(3, 1) = joint_node->supported_com;
        goalTpartial_com = goal_node->iTO * goalTpartial_com.inverse();

        dynamics_tree::Matrix4 goalTjoint = goal_node->iTO * joint_node->iTO.inverse();
        goalTjoint.topRightCorner(3, 1) -= goalTpartial_com.topRightCorner(3, 1); //translate goal to tool frame
        dynamics_tree::Matrix6 goalXjoint = dynamics_tree::motionTransformFromAffine(goalTjoint);
        jacobian.block<6, 1>(0, i) = goalXjoint * joint_node->axis * joint_node->supported_mass;
      }

      jacobian /= tree.getTotalMass();
    }

    void planPose(dynamics_tree::DynamicsTree& tree, std::string support_frame, std::string control_frame, std::vector<double> last_positions, std::vector<double>& next_positions, CartesianKeyframe k, int maxiter)
    {
      next_positions = last_positions;

      Pose pc, pt;
      Point cc, ct;
      k.getPoseCentroid(pc);
      k.getPoseTolerances(pt);
      k.getCoMCentroid(cc);
      k.getCoMTolerances(ct);

      dynamics_tree::Matrix4 supportTtarget = dynamics_tree::pureRotation(pc.R, pc.P, pc.Y);
      supportTtarget.topRightCorner(3, 1) << pc.x, pc.y, pc.z;

      for(int i = 0; i < maxiter; i++)
      {
        processState(tree, next_positions, m_rootTworld);

        dynamics_tree::Matrix4 supportTcontrol;
        tree.lookupTransform(support_frame, control_frame, supportTcontrol);

        dynamics_tree::Matrix J, JT, JJT;
        tree.jacobian(m_joint_ids, support_frame, control_frame, J);
        JT = J.transpose();
        JJT = J * JT;

        double twist_scale = 0.1;
        double dx, dy, dz, dR, dP, dY;
        dynamics_tree::Vector6 foot_twist = frameTwist(supportTcontrol, supportTtarget, k.duration, dx, dy, dz, dR, dP, dY);
        foot_twist *= twist_scale;

        //transpose
        //dQ = J^T * e * (<e, J*J^T*e> / <J*J^T*e, J*J^T*e>)
        //      dynamics_tree::Vector JJTe = JJT * foot_twist;
        //      //boost::this_thread::interruption_point();
        //      dynamics_tree::Vector trans_velocities = JT * foot_twist * (foot_twist.transpose() * JJTe) / (JJTe.transpose() * JJTe);
        //      //boost::this_thread::interruption_point();

        //pinv
        dynamics_tree::Vector pinv_velocities = JT * JJT.inverse() * foot_twist;

        //===========COM==================
        dynamics_tree::Vector4 com_vector;
        tree.centerOfMass(support_frame, com_vector);
        double com_dx = cc.x - com_vector(0);
        double com_dy = cc.y - com_vector(1);
        double com_dz = 0; //cc.z - com_vector(2);

        dynamics_tree::Matrix Jcom, JcomT, JcomJcomT;
        comJacobian(tree, m_joint_ids, support_frame, Jcom);
        JcomT = Jcom.transpose();
        JcomJcomT = Jcom * JcomT;

        dynamics_tree::Vector6 com_twist = dynamics_tree::Vector6::Zero();
        com_twist(3) = com_dx / k.duration;
        com_twist(4) = com_dy / k.duration;
        com_twist(5) = com_dz / k.duration;
        com_twist *= twist_scale;

        //transpose
        //      dynamics_tree::Vector JJTe_com = JcomJcomT * com_twist;
        //      //boost::this_thread::interruption_point();
        //      dynamics_tree::Vector com_trans_velocities = JcomT * com_twist * (com_twist.transpose() * JJTe_com) / (JJTe_com.transpose() * JJTe_com);
        //      //boost::this_thread::interruption_point();

        //pinv
        dynamics_tree::Vector com_pinv_velocities = JcomT * JcomJcomT.inverse() * com_twist;

        //check if done
        if(fabs(com_dx) < ct.x && fabs(com_dy) < ct.y && fabs(com_dz) < ct.z && fabs(dx) < pt.x && fabs(dy) < pt.y && fabs(dz) < pt.z && fabs(dR) < pt.R && fabs(dP) < pt.P && fabs(dY) < pt.Y)
        {
          std::cerr << ".";
          return;
        }

        //NAO SPECIFIC: hip yaw velocities (idx 2 and 8) must be equal and inverse of one another
        double pinv_hip_yaw_avg = (pinv_velocities[2] - pinv_velocities[8]) / 2.0;
        pinv_velocities[2] = pinv_hip_yaw_avg;
        pinv_velocities[8] = -pinv_hip_yaw_avg;
        double com_pinv_hip_yaw_avg = (com_pinv_velocities[2] - com_pinv_velocities[8]) / 2.0;
        com_pinv_velocities[2] = com_pinv_hip_yaw_avg;
        com_pinv_velocities[8] = -com_pinv_hip_yaw_avg;
        //!NAO SPECIFIC

        //simulate forward (euler)
        for(unsigned int j = 0; j < m_joint_ids.size(); j++)
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

          double local_min = std::max((double) m_joint_mins[j], (double) (last_positions[joint_id] - m_joint_vels[j] * k.duration));
          double local_max = std::min((double) m_joint_maxes[j], (double) (last_positions[joint_id] + m_joint_vels[j] * k.duration));

          next_positions.at(joint_id) += k.duration * combined_joint_velocity / (double) num_velocities;
          next_positions.at(joint_id) = dynamics_tree::clamp(next_positions.at(joint_id), local_min, local_max);
        }
      }

      std::cerr << "Failed to solve!" << std::endl;
    }

    inline void matrixToRPY(dynamics_tree::Matrix3 rot, double&roll, double&pitch, double&yaw)
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
    inline dynamics_tree::Vector6 frameTwist(dynamics_tree::Matrix4 commonTf1, dynamics_tree::Matrix4 commonTf2, double dt, double& dx, double& dy, double& dz, double& dR, double& dP, double& dY)
    {
      dynamics_tree::Matrix4 delta = commonTf1.inverse() * commonTf2;
      Eigen::AngleAxis<double> angle_axis((dynamics_tree::Matrix3) delta.topLeftCorner(3, 3));

      dynamics_tree::Vector6 twist; //expressed here in f1 coordinates

      matrixToRPY((dynamics_tree::Matrix3) delta.topLeftCorner(3, 3), dR, dP, dY);
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
  };
}
#endif //KACK_HPP

