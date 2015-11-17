#ifndef MATEC_JOINT_H
#define MATEC_JOINT_H

namespace dynamics_tree
{
  class Link;

  enum JointType
  {
    FIXED_BASE,
    FLOATING_BASE,
    REVOLUTE_JOINT,
    PRISMATIC_JOINT,
    FIXED_JOINT,
    NUM_JOINT_TYPES
  };
  static std::string joint_type_strings[NUM_JOINT_TYPES] = {"fixed base", "floating base", "revolute joint", "prismatic joint", "fixed joint"};

  class Joint
  {
  public:

    int idx;
    std::string name;
    JointType type;
    double q;
    double q_dot;
    double q_dot_dot;
    double torque;
    dynamics_tree::Vector6 axis; //axis of rotation

    double viscous_friction;
    double coulomb_friction;
    double max_torque;
    double max_q_dot;
    double min_q;
    double max_q;

    //parent and child must be specified so that we know when to flip the joint axis
    boost::shared_ptr<Link> parent_link;
    boost::shared_ptr<Link> child_link;

    Joint()
    {
      idx = -1;
      name = "";
      type = FIXED_JOINT;
      q = std::numeric_limits<double>::quiet_NaN(); //NaN to force kinematics to trigger at least once
      q_dot = 0.0;
      q_dot_dot = 0.0;
      torque = 0.0;

      viscous_friction = 0.0;
      coulomb_friction = 0.0;
      max_torque = 0.0;
      max_q_dot = 0.0;
      min_q = 0.0;
      max_q = 0.0;
    }

  };
}

#endif //MATEC_JOINT_H
