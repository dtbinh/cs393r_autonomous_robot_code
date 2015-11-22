#ifndef DYNAMICS_TREE_NODE_H
#define DYNAMICS_TREE_NODE_H

#include <boost/thread.hpp>
#include <time.h>
#include <ctime>
#include "matec_joint.h"
#include "matec_link.h"

namespace dynamics_tree
{
  class DynamicsTreeNode
  {
  public:
    typedef boost::function<void(boost::shared_ptr<DynamicsTreeNode> node, std::vector<double>& positions, std::vector<double>& velocities, std::vector<double>& accelerations, std::vector<double>& torques, std::vector<dynamics_tree::Vector6>& external_wrenches)> RecursionOperation;

    //direction in which dynamics should be solved
    enum DynamicsDirection
    {
      FORWARD_DYNAMICS, //torque->accel
      INVERSE_DYNAMICS, //accel->torque
      NUM_DYNAMICS_DIRECTION_TYPES
    };
    std::string dynamics_direction_strings[NUM_DYNAMICS_DIRECTION_TYPES] = {"forward dynamics", "inverse dynamics"};

    DynamicsTreeNode()
    {
      joint_idx = -1;
      joint_type = FIXED_JOINT;
      dynamics_direction = FORWARD_DYNAMICS;
      external_wrench_frame = NO_FRAME; // TODO: add warning about no frame with non-zero specified joint wrenches

      supported_com = dynamics_tree::Vector4::Zero();
      supported_mass = 0.0;

      q = std::numeric_limits<double>::quiet_NaN(); //NaN to force kinematics to trigger at least once
      q_dot = 0.0;
      q_dot_dot = 0.0;
      torque = 0.0;

      iXicom = dynamics_tree::Matrix6::Identity();
      iXip = dynamics_tree::Matrix6::Identity();
      iXO = dynamics_tree::Matrix6::Identity();
      iTicom = dynamics_tree::Matrix4::Identity();
      iTip = dynamics_tree::Matrix4::Identity();
      iTO = dynamics_tree::Matrix4::Identity();

      v = dynamics_tree::Vector6::Zero();
      a = dynamics_tree::Vector6::Zero();
      pA = dynamics_tree::Vector6::Zero();
      pa = dynamics_tree::Vector6::Zero();
      f_ext = dynamics_tree::Vector6::Zero();
      c = dynamics_tree::Vector6::Zero();
      D_inverse = 0.0;
      U = dynamics_tree::Vector6::Zero();
      u = 0.0;
    }

    int joint_idx;
    std::string joint_name;
    JointType joint_type;
    DynamicsDirection dynamics_direction;
    ExternalWrenchFrame external_wrench_frame;
    double q;
    double q_dot;
    double q_dot_dot;
    double torque;

    double viscous_friction;
    double coulomb_friction;
    double max_torque;
    double max_q_dot;
    double min_q;
    double max_q;

    boost::shared_ptr<DynamicsTreeNode> self;
    boost::shared_ptr<DynamicsTreeNode> parent; //todo: support multiple parents
    std::vector<boost::shared_ptr<DynamicsTreeNode> > children; //direct children of this node
    std::vector<boost::shared_ptr<DynamicsTreeNode> > supported; //everything underneath this node

    dynamics_tree::Vector4 supported_com;
    double supported_mass;

    bool enforce_joint_limits;

    std::string link_name;

    double mass;
    dynamics_tree::Matrix6 link_inertia; //inertia of the current link
    dynamics_tree::Matrix6 composite_inertia; //inertia of the current link and all those below it
    dynamics_tree::Matrix6 articulated_body_inertia; //inertia of the articulated system

    dynamics_tree::Vector6 axis; //axis of rotation
    dynamics_tree::Vector3 r_fixed; //fixed joint translation offset
    dynamics_tree::Matrix3 E_fixed; //fixed joint rotation offset
    dynamics_tree::Matrix6 iXicom; //motion transform from center of mass to joint frame
    dynamics_tree::Matrix6 iXip; //motion transform from parent to joint frame
    dynamics_tree::Matrix6 iXO; //motion transform from the inertial to joint frame
    dynamics_tree::Matrix4 iTicom; //homogenous transform from center of mass to joint frame
    dynamics_tree::Matrix4 iTip; //homogenous transform from parent to joint frame
    dynamics_tree::Matrix4 iTO; //homogenous transform from inertial to joint frame

    dynamics_tree::Vector6 v; //spatial velocity
    dynamics_tree::Vector6 a; //spatial acceleration
    dynamics_tree::Vector6 pA; //spatial force
    dynamics_tree::Vector6 pa; //spatial force
    dynamics_tree::Vector6 f_ext; //external force on link
    dynamics_tree::Vector6 c; //centripetal terms
    double D_inverse; //inverse of the joint's component of composite inertia
    dynamics_tree::Vector6 U;
    double u;

    void updateSupport()
    {
      supported.clear();
      supported.push_back(self);
      for(unsigned int i = 0; i < children.size(); i++)
      {
        children[i]->updateSupport();
        supported.insert(supported.end(), children[i]->supported.begin(), children[i]->supported.end());
      }
    }

    bool supports(boost::shared_ptr<DynamicsTreeNode> other_node)
    {
      for(unsigned int i = 0; i < supported.size(); i++)
      {
        if(supported[i] == other_node)
        {
          return true;
        }
      }
      return false;
    }

    void recurse(std::vector<RecursionOperation>& forward_pass_operations, std::vector<RecursionOperation>& backward_pass_operations, std::vector<double>& positions, std::vector<double>& velocities, std::vector<double>& accelerations, std::vector<double>& torques, std::vector<dynamics_tree::Vector6>& external_wrenches)
    {
      for(unsigned int i = 0; i < forward_pass_operations.size(); i++)
      {
        forward_pass_operations[i](self, positions, velocities, accelerations, torques, external_wrenches);
      }
      for(unsigned int i = 0; i < children.size(); i++)
      {
        children[i]->recurse(forward_pass_operations, backward_pass_operations, positions, velocities, accelerations, torques, external_wrenches);
      }
      for(unsigned int i = 0; i < backward_pass_operations.size(); i++)
      {
        backward_pass_operations[i](self, positions, velocities, accelerations, torques, external_wrenches);
      }
    }

    void configureDynamics(std::vector<DynamicsDirection>& dynamics_direction_config, std::vector<ExternalWrenchFrame>& external_wrench_frame_config)
    {
      if(joint_idx >= 0 && joint_idx < (int) dynamics_direction_config.size())
      {
        dynamics_direction = dynamics_direction_config.at(joint_idx);
        external_wrench_frame = external_wrench_frame_config.at(joint_idx);
      }
      else if(joint_type == FIXED_JOINT)
      {
        dynamics_direction = DynamicsTreeNode::INVERSE_DYNAMICS; //fixed joints are always inverse dynamics
      }
      for(unsigned int i = 0; i < children.size(); i++)
      {
        children[i]->configureDynamics(dynamics_direction_config, external_wrench_frame_config);
      }
    }
  };
}

#endif //DYNAMICS_TREE_NODE_H
