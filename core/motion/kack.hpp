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
    Kack(std::string model_filename, double planning_rate = 100.0) :
        m_supporting_tree(m_left_tree)
    {
      m_have_plan = false;
      m_planning_rate = planning_rate;
      m_xml_model.fromFile(model_filename);
      m_rootTworld = dynamics_tree::Matrix4::Identity();

//      std::cerr << "Had " << m_joint_names.size() << " joint names" << std::endl;

      m_graph.loadFromURDF(m_joint_names, m_xml_model);
//      m_graph.print("l_sole");
      m_graph.spawnDynamicsTree("l_sole", false, m_left_tree);
      m_graph.spawnDynamicsTree("r_sole", false, m_right_tree);

      for(unsigned int i = 0; i < (m_joint_names.size() - 4); i++)
      {
        if(m_joint_names.at(i) == "HeadYaw" || m_joint_names.at(i) == "HeadPitch" || m_joint_names.at(i) == "LElbowYaw" || m_joint_names.at(i) == "LElbowRoll" || m_joint_names.at(i) == "RElbowYaw" || m_joint_names.at(i) == "RElbowRoll")
        {
          continue;
        }

        if(m_joint_names.at(i) != "LHipYawPitch" && m_joint_names.at(i) != "LHipRoll" && m_joint_names.at(i) != "LHipPitch" && m_joint_names.at(i) != "LKneePitch" && m_joint_names.at(i) != "LAnklePitch" && m_joint_names.at(i) != "LAnkleRoll" && m_joint_names.at(i) != "RHipYawPitch" && m_joint_names.at(i) != "RHipRoll" && m_joint_names.at(i) != "RHipPitch" && m_joint_names.at(i) != "RKneePitch" && m_joint_names.at(i) != "RAnklePitch" && m_joint_names.at(i) != "RAnkleRoll")
          m_balance_joint_ids.push_back(i);

//        std::cerr << "joint " << i << " is " << m_joint_names.at(i) << std::endl;
        m_joint_ids.push_back(i);

        pugi::xml_node joint_xml = m_xml_model.component("joint", m_joint_names.at(i))->xml;

        if(joint_xml && joint_xml.child("limit"))
        {
          double velocity_limit_scale = 0.25;
          m_joint_mins.push_back(joint_xml.child("limit").attribute("lower").as_double());
          m_joint_maxes.push_back(joint_xml.child("limit").attribute("upper").as_double());
          m_joint_vels.push_back(velocity_limit_scale*joint_xml.child("limit").attribute("velocity").as_double());
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
    bool plan(std::vector<double> starting_joint_positions, std::vector<CartesianKeyframe> keyframes, bool left_foot_supporting = true, bool invert_starting_joints = true)
    {
      m_cartesian_plan.clear();
      m_joint_plan.clear();
      m_com_plan.clear();

      m_left_foot_supporting = left_foot_supporting;
      m_supporting_tree = left_foot_supporting? m_left_tree : m_right_tree;
      m_supporting_frame = left_foot_supporting? "l_sole" : "r_sole";
      m_controlled_frame = left_foot_supporting? "r_sole" : "l_sole";

      if(invert_starting_joints)
        invertJoints(starting_joint_positions);

      m_joint_plan.push_back(starting_joint_positions);

      for(unsigned int k = 0; k < keyframes.size(); k++)
      {
        std::cerr << "Working on keyframe idx " << k + 1 << "/" << keyframes.size() << std::endl;
        m_supporting_tree.kinematics(m_joint_plan.at(m_joint_plan.size() - 1), m_rootTworld);
        dynamics_tree::Matrix4 leftTright;
        m_supporting_tree.lookupTransform(m_supporting_frame, m_controlled_frame, leftTright);
        double xi = leftTright(0, 3);
        double yi = leftTright(1, 3);
        double zi = leftTright(2, 3);

        double Ri, Pi, Yi;
        matrixToRPY((dynamics_tree::Matrix3) leftTright.topLeftCorner(3, 3), Ri, Pi, Yi);

        dynamics_tree::Vector4 com_vector;
        m_supporting_tree.centerOfMass(m_supporting_frame, com_vector);

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
          planPose(m_joint_plan.at(m_joint_plan.size() - 1), next_positions, interpolated, 10000);
          m_joint_plan.push_back(next_positions);
        }
        std::cerr << std::endl;
      }
      m_start_time = 0.0;
      m_have_plan = true;
      return true;
    }

    Point calculateCoP(FootSensor f)
    {
      double X = 0.1;
      double Y = 0.05;
      double d = 2.0 * (f.fr + f.rr + f.fl + f.rl);
      Point cop;
      cop.x = X * (f.fr + f.rr - f.fl - f.rl) / d;
      cop.y = Y * (f.fr + f.fl - f.rr - f.rl) / d;
      return cop;
    }

    double calculateForce(FootSensor f)
    {
      return (f.fr + f.rr + f.fl + f.rl);
    }

    //current_time in seconds, command is output joint positions
    void execute(double current_time, std::vector<double> current_joint_positions, FootSensor left_foot, FootSensor right_foot, std::vector<double>& command, double cop_alpha = 0.95, double kp_cop_x = 1e-3, double kp_cop_y = 1e-3, double kmax_cop_x = 1e-2, double kmax_cop_y = 1e-2, double cop_x_desired = 0.0, double cop_y_desired = 0.0, double kicking_foot_force_threshold = 0.1)
    {
      if(!m_have_plan)
      {
        std::cerr << "Execute called before planning completed!" << std::endl;
        return;
      }
      if(m_start_time == 0.0)
      {
        m_start_time = current_time;
      }
      
      //get current slice
      double time_since_start = current_time - m_start_time;
      unsigned long current_slice = time_since_start * m_planning_rate;
      for(unsigned int i = 0; i < command.size(); i++) //explicit copy just in case
      {
        command[i] = (current_slice < m_joint_plan.size())? m_joint_plan.at(current_slice)[i] : m_joint_plan.back()[i];
      }

      //compute CoM adjustment
      Point cop = m_left_foot_supporting? calculateCoP(left_foot) : calculateCoP(right_foot);
      m_cop.x = cop_alpha * m_cop.x + (1.0 - cop_alpha) * cop.x;
      m_cop.y = cop_alpha * m_cop.y + (1.0 - cop_alpha) * cop.y;
//      double kicking_foot_force = m_left_foot_supporting? calculateForce(right_foot) : calculateForce(left_foot);
      std::cerr << "CoP is at (" << m_cop.x << ", " << m_cop.y << ")" << std::endl;
      // if(fabs(kicking_foot_force) < kicking_foot_force_threshold)
      // {
      dynamics_tree::Matrix Jcom, JcomT, JcomJcomT;
      // invertJoints(current_joint_positions);
      processState(command, m_rootTworld);
      comJacobian(m_balance_joint_ids, m_supporting_frame, Jcom);
      JcomT = Jcom.transpose();
      JcomJcomT = Jcom * JcomT;

      dynamics_tree::Vector6 com_twist = dynamics_tree::Vector6::Zero();
      // com_twist(3) = kmax_cop_y * tanh(kp_cop_y * (cop_y_desired - m_cop.y));
      // com_twist(4) = -kmax_cop_x * tanh(kp_cop_x * (cop_x_desired - m_cop.x));
      com_twist(3) = (kp_cop_y * (cop_y_desired - m_cop.y));
      com_twist(4) = -(kp_cop_x * (cop_x_desired - m_cop.x));
      com_twist(5) = 0.0;
      std::cerr << "CoM twist is (" << com_twist(3) << ", " << com_twist(4) << ")" << std::endl;
      com_twist *= m_planning_rate;
      dynamics_tree::Vector com_pinv_velocities = JcomT * JcomJcomT.inverse() * com_twist;

      //NAO SPECIFIC: hip yaw velocities (idx 2 and 8) must be equal and inverse of one another
      // double com_pinv_hip_yaw_avg = (com_pinv_velocities[2] - com_pinv_velocities[8]) / 2.0;
      // com_pinv_velocities[2] = com_pinv_hip_yaw_avg;
      // com_pinv_velocities[8] = -com_pinv_hip_yaw_avg;
      //!NAO SPECIFIC

      for(unsigned int i = 0; i < m_balance_joint_ids.size(); i++)
      {
        double vel = com_pinv_velocities(i); //dynamics_tree::clamp(com_pinv_velocities(i), -1.0, 1.0);
        command[m_balance_joint_ids[i]] += vel / m_planning_rate;
        std::cerr << "vel is " << vel << ", cmd is " << command[m_balance_joint_ids[i]] << std::endl;
      }
      // }
      // else
      // {
      //   std::cerr << "Kicking foot is still on the ground! Not balancing!" << std::endl;
      // }

      //switch command to UT's signs
      invertJoints(command);
    }

    //returns true when foot satisfies k
    bool moveFoot(double rate, bool left_foot_supporting, CartesianKeyframe k, std::vector<double> current_joint_positions, FootSensor left_foot, FootSensor right_foot, std::vector<double>& command, double cop_alpha = 0.95, double kp_cop_x = 1e-3, double kp_cop_y = 1e-3, double kmax_cop_x = 1e-2, double kmax_cop_y = 1e-2, double kicking_foot_force_threshold = 0.1, bool invert_joints = true)
    {
      if(m_last_commanded.size() == 0)
      {
        m_last_commanded = current_joint_positions;
        if(invert_joints)
          invertJoints(m_last_commanded);
      }
      m_planning_rate = rate;
      m_left_foot_supporting = left_foot_supporting;
      m_supporting_tree = left_foot_supporting? m_left_tree : m_right_tree;
      m_supporting_frame = left_foot_supporting? "l_sole" : "r_sole";
      m_controlled_frame = left_foot_supporting? "r_sole" : "l_sole";

      Pose pc, pt;
      Point cc, ct;
      k.getPoseCentroid(pc);
      k.getPoseTolerances(pt);
      k.getCoMCentroid(cc);
      k.getCoMTolerances(ct);

      for(unsigned int i = 0; i < 1; i++)
      {
        processState(m_last_commanded, m_rootTworld);

        dynamics_tree::Matrix4 supportTtarget = dynamics_tree::pureRotation(pc.R, pc.P, pc.Y);
        supportTtarget.topRightCorner(3, 1) << pc.x, pc.y, pc.z;

        dynamics_tree::Matrix4 supportTcontrol, supportTtorso;
        m_supporting_tree.lookupTransform(m_supporting_frame, m_controlled_frame, supportTcontrol);
        m_supporting_tree.lookupTransform(m_supporting_frame, "torso", supportTtorso);

        double dx, dy, dz, dR, dP, dY;
        dynamics_tree::Vector6 foot_twist = frameTwist(supportTcontrol, supportTtarget, 1.0 / m_planning_rate, dx, dy, dz, dR, dP, dY);

        dynamics_tree::Vector4 com_vector;
        m_supporting_tree.centerOfMass(m_supporting_frame, com_vector);
        double com_dx = cc.x - com_vector(0);
        double com_dy = cc.y - com_vector(1);
        double com_dz = cc.z - com_vector(2);

        //check if done
        if(fabs(com_dx) < ct.x && fabs(com_dy) < ct.y && fabs(com_dz) < ct.z && fabs(dx) < pt.x && fabs(dy) < pt.y && fabs(dz) < pt.z && fabs(dR) < pt.R && fabs(dP) < pt.P && fabs(dY) < pt.Y)
        {
          return true;
        }

        //compute torso adjustment
        double torso_weight = 0.5;
        double torso_roll, torso_pitch, torso_yaw;
        matrixToRPY(dynamics_tree::Matrix3(supportTtorso.topLeftCorner(3, 3)), torso_roll, torso_pitch, torso_yaw);
        dynamics_tree::Vector6 torso_twist = dynamics_tree::Vector6::Zero();
        torso_twist(0) = 0;//torso_roll;
        torso_twist(1) = -torso_pitch;
        torso_twist *= torso_weight*m_planning_rate;
        std::cerr << "torso pitch: " << torso_pitch << ", torso roll: "<< torso_roll << std::endl;

        //compute CoM adjustment
        Point cop = m_left_foot_supporting? calculateCoP(left_foot) : calculateCoP(right_foot);
        m_cop.x = cop_alpha * m_cop.x + (1.0 - cop_alpha) * cop.x;
        m_cop.y = cop_alpha * m_cop.y + (1.0 - cop_alpha) * cop.y;

        dynamics_tree::Vector6 com_twist = dynamics_tree::Vector6::Zero();
        // com_twist(3) = kmax_cop_y * tanh(kp_cop_y * (cop_y_desired - m_cop.y));
        // com_twist(4) = -kmax_cop_x * tanh(kp_cop_x * (cop_x_desired - m_cop.x));
//      com_twist(3) = (kp_cop_y * (cc.y - m_cop.y));
//      com_twist(4) = -(kp_cop_x * (cc.x - m_cop.x));
        com_twist(3) = com_dx; // + (kp_cop_y * (cc.y - m_cop.y));
        com_twist(4) = com_dy; // - (kp_cop_x * (cc.x - m_cop.x));
        com_twist(5) = com_dz;
        com_twist *= m_planning_rate;

        move(m_last_commanded, m_last_commanded, foot_twist, torso_twist, com_twist, 1.0 / m_planning_rate, false);
      }

      command = m_last_commanded;
      if(invert_joints)
        invertJoints(command); //switch command to UT's signs

      return false;
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

    dynamics_tree::DynamicsTree& tree(bool left)
    {
      return left? m_left_tree : m_right_tree;
    }

  private:
    std::vector<std::string> m_joint_names = {"HeadYaw", "HeadPitch", "LHipYawPitch", "LHipRoll", "LHipPitch", "LKneePitch", "LAnklePitch", "LAnkleRoll", "RHipYawPitch", "RHipRoll", "RHipPitch", "RKneePitch", "RAnklePitch", "RAnkleRoll", "LShoulderPitch", "LShoulderRoll", "LElbowYaw", "LElbowRoll", "RShoulderPitch", "RShoulderRoll", "RElbowYaw", "RElbowRoll", "LWristYaw", "LHand", "RWristYaw", "RHand"};
    std::vector<double> m_nominal_positions = {0, -0.4, 0, 0, -0.436, 0.873, -0.436, 0, 0, 0, -0.436, 0.873, -0.436, 0, 1.4, 0.3, 0, -0.05, 1.4, -0.3, 0, 0.05, 0, 0, 0, 0};
    std::vector<double> m_inverted_joints = {1.0, -1.0, 1.0, -1.0, 1.0, 1.0, 1.0, -1.0, -1.0, 1.0, 1.0, 1.0, 1.0, 1.0, -1.0, 1.0, 1.0, 1.0, -1.0, -1.0, -1.0, -1.0, 0.0, 0.0, 0.0, 0.0};

    rpp::ComponentTree m_xml_model;
    dynamics_tree::DynamicsGraph m_graph;
    dynamics_tree::DynamicsTree m_left_tree;
    dynamics_tree::DynamicsTree m_right_tree;

    bool m_have_plan;
    bool m_left_foot_supporting;
    dynamics_tree::DynamicsTree& m_supporting_tree;
    std::string m_supporting_frame;
    std::string m_controlled_frame;

    std::vector<unsigned int> m_balance_joint_ids;
    std::vector<unsigned int> m_joint_ids;
    std::vector<double> m_joint_mins;
    std::vector<double> m_joint_maxes;
    std::vector<double> m_joint_vels;

    Point m_cop;

    dynamics_tree::Matrix4 m_rootTworld;

    double m_planning_rate;
    std::vector<CartesianKeyframe> m_cartesian_plan;
    std::vector<std::vector<double> > m_joint_plan;
    std::vector<Point> m_com_plan;
    std::vector<double> m_last_commanded;

    double m_start_time;

    inline void processState(std::vector<double>& positions, dynamics_tree::Matrix4& rootTworld)
    {
      m_supporting_tree.getRootNode()->iTO = rootTworld;
      m_supporting_tree.getRootNode()->iXO = dynamics_tree::motionTransformFromAffine(m_supporting_tree.getRootNode()->iTO);
      processStateRecursive(m_supporting_tree.getRootNode(), positions);
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
      node->supported_com += node->mass * node->iTicom.topRightCorner(4, 1);
      if(node->parent)
      {
        node->parent->supported_mass += node->supported_mass;
        node->parent->supported_com += node->supported_com; //node->iTip.inverse() * node->supported_com;
      }
      node->supported_com /= node->supported_mass;
    }

    void comJacobian(std::vector<unsigned int> joint_indices, std::string goal_frame, dynamics_tree::Matrix& jacobian)
    {
      if(m_supporting_tree.getNodes().find(goal_frame) == m_supporting_tree.getNodes().end())
      {
        std::cerr << "Couldn't find goal frame " << goal_frame << " in the tree!" << std::endl;
        return;
      }

      boost::shared_ptr<dynamics_tree::DynamicsTreeNode> goal_node = m_supporting_tree.getNodes()[goal_frame];
      jacobian.resize(6, joint_indices.size());
      for(unsigned int i = 0; i < joint_indices.size(); i++)
      {
        //boost::this_thread::interruption_point();
        boost::shared_ptr<dynamics_tree::DynamicsTreeNode> joint_node = m_supporting_tree.getNodeByIndex(joint_indices[i]);
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

      jacobian /= m_supporting_tree.getTotalMass();
    }

    void move(std::vector<double> last_positions, std::vector<double>& next_positions, dynamics_tree::Vector6 foot_twist, dynamics_tree::Vector6 torso_twist, dynamics_tree::Vector6 com_twist, double dt, bool do_kinematics = false)
    {
      if(do_kinematics)
      {
        processState(next_positions, m_rootTworld);
      }

      //compute jacobians
      dynamics_tree::Matrix Jfoot, JfootT, JfootJfootT, Jtorso, JtorsoT, JtorsoJtorsoT, Jcom, JcomT, JcomJcomT;
      m_supporting_tree.jacobian(m_joint_ids, m_supporting_frame, m_controlled_frame, Jfoot);
      JfootT = Jfoot.transpose();
      JfootJfootT = Jfoot * JfootT;
      m_supporting_tree.jacobian(m_joint_ids, m_supporting_frame, "torso", Jtorso);
      JtorsoT = Jtorso.transpose();
      JtorsoJtorsoT = Jtorso * JtorsoT;
      comJacobian(m_joint_ids, m_supporting_frame, Jcom);
      JcomT = Jcom.transpose();
      JcomJcomT = Jcom * JcomT;

      dynamics_tree::Vector foot_velocities = JfootT * JfootJfootT.inverse() * foot_twist;
      dynamics_tree::Vector torso_velocities = JtorsoT * JtorsoJtorsoT.inverse() * torso_twist;
      dynamics_tree::Vector com_velocities = JcomT * JcomJcomT.inverse() * com_twist;

      //NAO SPECIFIC: hip yaw velocities (idx 2 and 8) must be equal and inverse of one another
      double foot_hip_yaw_avg = (foot_velocities[2] + foot_velocities[8]) / 2.0;
      foot_velocities[2] = foot_hip_yaw_avg;
      foot_velocities[8] = foot_hip_yaw_avg;
      double torso_hip_yaw_avg = (torso_velocities[2] + torso_velocities[8]) / 2.0;
      torso_velocities[2] = torso_hip_yaw_avg;
      torso_velocities[8] = torso_hip_yaw_avg;
      double com_hip_yaw_avg = (com_velocities[2] + com_velocities[8]) / 2.0;
      com_velocities[2] = com_hip_yaw_avg;
      com_velocities[8] = com_hip_yaw_avg;
      //!NAO SPECIFIC

      //simulate forward (euler)
      for(unsigned int j = 0; j < m_joint_ids.size(); j++)
      {
        unsigned int joint_id = m_joint_ids[j];

        double nominal_pose_weight = 0.0;
        double combined_joint_velocity = nominal_pose_weight * (last_positions[joint_id] - m_nominal_positions[joint_id]);
        unsigned int num_velocities = (nominal_pose_weight == 0.0)? 0 : 1;
        if(foot_velocities[j] != 0.0)
        {
          combined_joint_velocity += foot_velocities[j];
          num_velocities++;
        }
        if(torso_velocities[j] != 0.0)
        {
          combined_joint_velocity += torso_velocities[j];
          num_velocities++;
        }
        if(com_velocities[j] != 0.0)
        {
          combined_joint_velocity += com_velocities[j];
          num_velocities++;
        }

        double local_min = std::max((double) m_joint_mins[j], (double) (last_positions[joint_id] - m_joint_vels[j] * dt));
        double local_max = std::min((double) m_joint_maxes[j], (double) (last_positions[joint_id] + m_joint_vels[j] * dt));

        next_positions.at(joint_id) += dt * combined_joint_velocity / (double) num_velocities;
        next_positions.at(joint_id) = dynamics_tree::clamp(next_positions.at(joint_id), local_min, local_max);

        //fix hips again to deal with joint limits
        double hip_yaw_avg = (next_positions[2] + next_positions[8]) / 2.0;
        next_positions[2] = hip_yaw_avg;
        next_positions[8] = hip_yaw_avg;
      }
    }

    void planPose(std::vector<double> last_positions, std::vector<double>& next_positions, CartesianKeyframe k, int maxiter)
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
        processState(next_positions, m_rootTworld);

        dynamics_tree::Matrix4 supportTcontrol;
        m_supporting_tree.lookupTransform(m_supporting_frame, m_controlled_frame, supportTcontrol);

        double twist_scale = 0.1;
        double dx, dy, dz, dR, dP, dY;
        dynamics_tree::Vector6 foot_twist = frameTwist(supportTcontrol, supportTtarget, k.duration, dx, dy, dz, dR, dP, dY);
        foot_twist *= twist_scale;

        dynamics_tree::Vector4 com_vector;
        m_supporting_tree.centerOfMass(m_supporting_frame, com_vector);
        double com_dx = cc.x - com_vector(0);
        double com_dy = cc.y - com_vector(1);
        double com_dz = 0; //cc.z - com_vector(2);

        dynamics_tree::Vector6 com_twist = dynamics_tree::Vector6::Zero();
        com_twist(3) = com_dx / k.duration;
        com_twist(4) = com_dy / k.duration;
        com_twist(5) = com_dz / k.duration;
        com_twist *= twist_scale;

        //check if done
        if(fabs(com_dx) < ct.x && fabs(com_dy) < ct.y && fabs(com_dz) < ct.z && fabs(dx) < pt.x && fabs(dy) < pt.y && fabs(dz) < pt.z && fabs(dR) < pt.R && fabs(dP) < pt.P && fabs(dY) < pt.Y)
        {
          std::cerr << ".";
          return;
        }

        move(next_positions, next_positions, foot_twist, dynamics_tree::Vector6::Zero(), com_twist, k.duration, false);
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

