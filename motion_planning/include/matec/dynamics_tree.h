#ifndef DYNAMICS_TREE_H
#define DYNAMICS_TREE_H

#include "common_functions.h"
#include "dynamics_tree_node.h"

namespace dynamics_tree
{
  class DynamicsTree
  {
  public:
    //initialization
    DynamicsTree();
//    void load(std::vector<std::string>& joint_names, urdf::Model& model, bool floating_base = false);
    void configureDynamics(std::vector<DynamicsTreeNode::DynamicsDirection> dynamics_direction_config, std::vector<ExternalWrenchFrame> external_wrench_frame_config);

    //modification
    bool addFixedFrame(std::string parent, std::string child, dynamics_tree::Matrix4 cTp);
    bool updateFixedFrameOffset(std::string name, dynamics_tree::Matrix4 cTp);

    //kinematics
    void kinematics(std::vector<double>& positions, dynamics_tree::Matrix4& root_link_pose); //propagates joint positions and computes transformation matrices to allow the use of transformation functions
    bool lookupTransform(std::string target_frame, std::string source_frame, dynamics_tree::Matrix4& transform);
    bool lookupMotionTransform(std::string target_frame, std::string source_frame, dynamics_tree::Matrix6& transform);
    bool transformVector(std::string target_frame, std::string source_frame, dynamics_tree::Vector3 input, dynamics_tree::Vector3& output);
    bool transformPoint(std::string target_frame, std::string source_frame, dynamics_tree::Vector4 input, dynamics_tree::Vector4& output);
    bool transformPoint(std::string target_frame, std::string source_frame, dynamics_tree::Vector3 input, dynamics_tree::Vector3& output);
    bool transformPose(std::string target_frame, std::string source_frame, dynamics_tree::Matrix4 input, dynamics_tree::Matrix4& output);
    bool transformTwist(std::string target_frame, std::string source_frame, dynamics_tree::Vector6 input, dynamics_tree::Vector6& output);
    bool transformWrench(std::string target_frame, std::string source_frame, dynamics_tree::Vector6 input, dynamics_tree::Vector6& output);

    //meta properties of a tree (assumes that kinematics and / or dynamics routines have already been performed)
    void computeSupportedCentersOfMassRecursive(boost::shared_ptr<DynamicsTreeNode> node);
    void supportedCenterOfMass(boost::shared_ptr<DynamicsTreeNode> node, dynamics_tree::Vector4& com, double& supported_mass);
    void centerOfMass(std::string target_frame ,dynamics_tree::Vector4& com);

    bool getSupportingSubset(std::vector<unsigned int> desired_joint_indices, std::string tool_frame, std::vector<unsigned int>& supporting_joint_indices);
    bool jacobian(std::vector<unsigned int> joint_indices, std::string goal_frame, std::string tool_frame, dynamics_tree::Matrix& jacobian);
    bool cartesianVelocityToJointVelocities(std::vector<unsigned int> joint_indices, dynamics_tree::Vector6 twist, std::string goal_frame, std::string tool_frame, std::vector<double>& joint_velocities);

    //node access
    std::map<std::string, boost::shared_ptr<DynamicsTreeNode> > getNodes()
    {
      return m_nodes;
    }
    boost::shared_ptr<DynamicsTreeNode> getNodeByLink(std::string link_name)
    {
      std::map<std::string, boost::shared_ptr<DynamicsTreeNode> >::iterator node_iter = m_nodes.find(link_name);
      if(node_iter == m_nodes.end())
      {
        return boost::shared_ptr<DynamicsTreeNode>();
      }
      return node_iter->second;
    }
    boost::shared_ptr<DynamicsTreeNode> getNodeByJoint(std::string joint_name)
    {
      if(m_joint_name_to_node.find(joint_name) != m_joint_name_to_node.end())
      {
        return m_joint_name_to_node[joint_name];
      }
      else
      {
        return boost::shared_ptr<DynamicsTreeNode>();
      }
    }
    boost::shared_ptr<DynamicsTreeNode> getNodeByIndex(unsigned int joint_idx)
    {
      if(m_joint_index_to_node.find(joint_idx) != m_joint_index_to_node.end())
      {
        return m_joint_index_to_node[joint_idx];
      }
      else
      {
        return boost::shared_ptr<DynamicsTreeNode>();
      }
    }

    void setNode(boost::shared_ptr<DynamicsTreeNode> node)
    {
      m_nodes[node->link_name] = node;
      if(node->joint_type != FIXED_JOINT)
      {
        m_joint_index_to_node[node->joint_idx] = node;
        m_joint_name_to_node[node->joint_name] = node;
      }
    }

    void setRootNode(boost::shared_ptr<DynamicsTreeNode> node)
    {
      m_root_node = node;
    }

    boost::shared_ptr<DynamicsTreeNode> getRootNode()
    {
      return m_root_node;
    }

    //getters and setters
    void setFloatingBase(bool floating_base)
    {
      m_floating_base = floating_base;
    }

    void setJointNames(std::vector<std::string> joint_names)
    {
      m_joint_names = joint_names;
    }

    void setTotalMass(double mass)
    {
      m_total_mass = mass;
    }

    double getTotalMass()
    {
      return m_total_mass;
    }

    std::string getInertialFrameName()
    {
      return m_inertial_frame_name;
    }

    void reset()
    {
      m_root_node.reset();
      m_nodes.clear();
      m_joint_index_to_node.clear();
      m_joint_name_to_node.clear();
      m_joint_names.clear();
      m_floating_base = false;
      m_total_mass = 0.0;
    }

  protected:
    boost::shared_ptr<DynamicsTreeNode> m_root_node;
    std::map<std::string, boost::shared_ptr<DynamicsTreeNode> > m_nodes;
    std::map<int, boost::shared_ptr<DynamicsTreeNode> > m_joint_index_to_node;
    std::map<std::string, boost::shared_ptr<DynamicsTreeNode> > m_joint_name_to_node;
    std::vector<std::string> m_joint_names;
    bool m_floating_base;
    double m_total_mass;

    bool m_inertial_frame_name_set;
    std::string m_inertial_frame_name;

//    boost::shared_ptr<DynamicsTreeNode> loadRecursive(boost::shared_ptr<const urdf::Link> link, boost::shared_ptr<DynamicsTreeNode> parent);

    void kinematicsInitialization(dynamics_tree::Matrix4& rootTworld);
    void kinematicsPass(boost::shared_ptr<DynamicsTreeNode> node, std::vector<double>& positions, std::vector<double>& velocities, std::vector<double>& accelerations, std::vector<double>& torques, std::vector<dynamics_tree::Vector6>& external_wrenches);
  };
}

#include "dynamics_tree_initialization.h"
#include "dynamics_tree_kinematics.h"
#include "dynamics_tree_meta_properties.h"

#endif //DYNAMICS_TREE_H
