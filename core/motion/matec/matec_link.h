#ifndef MATEC_LINK_H
#define MATEC_LINK_H

namespace dynamics_tree
{
  class Joint;

  //frame in which external wrenches are specified
  enum ExternalWrenchFrame
  {
    NO_FRAME,
    INERTIAL_FRAME,
    JOINT_FRAME,
    COM_FRAME,
    NUM_EXTERNAL_WRENCH_FRAME_TYPES
  };
  static std::string external_wrench_frame_strings[NUM_EXTERNAL_WRENCH_FRAME_TYPES] = {"no frame", "inertial frame", "joint frame", "com frame"};

  class Link
  {
  public:
    std::string name; //name of the link
    double mass; //total mass of the link
    dynamics_tree::Vector3 com_position; //position of the CoM in link coordinates
    dynamics_tree::Matrix3 com_inertia; //inertia at the CoM
    dynamics_tree::Matrix6 spatial_inertia; //inertia of the current link
    dynamics_tree::Matrix4 linkToriginallink; //homogenous transform from the original frame location to the shifted dynamics tree version
    dynamics_tree::Matrix4 originallinkTlinkcom; //homogenous transform from center of mass to link frame
    ExternalWrenchFrame external_wrench_frame; //frame in which external wrenches should be expressed

    std::vector<boost::shared_ptr<Joint> > connected_joints;
    std::vector<dynamics_tree::Matrix4> jointToriginallink; //transform from link coordinates to joint coordinates

    Link()
    {
      name = "";
      mass = 1e-12; //small
      com_position = dynamics_tree::Vector3::Zero();
      com_inertia = dynamics_tree::Matrix3::Identity() * mass;
      spatial_inertia = dynamics_tree::Matrix6::Identity();
      originallinkTlinkcom = dynamics_tree::Matrix4::Identity();
      external_wrench_frame = NO_FRAME;
    }

    unsigned int findConnectedJointIndex(boost::shared_ptr<Joint> joint)
    {
      for(unsigned int i = 0; i < connected_joints.size(); i++)
      {
        if(joint == connected_joints[i])
        {
          return i;
        }
      }
      return connected_joints.size();
    }
  };
}

#endif //MATEC_LINK_H
