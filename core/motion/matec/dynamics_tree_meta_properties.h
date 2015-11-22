#ifndef DYNAMICS_TREE_META_PROPERTIES_H
#define DYNAMICS_TREE_META_PROPERTIES_H

namespace dynamics_tree
{
  inline void DynamicsTree::computeSupportedCentersOfMassRecursive(boost::shared_ptr<DynamicsTreeNode> node)
  {
    //boost::this_thread::interruption_point();
    node->supported_mass = node->mass;
    node->supported_com = node->mass * node->iTicom.topRightCorner(4, 1);
    for(unsigned int i = 0; i < node->children.size(); i++) //todo: use preexisting recursion structure?
    {
      computeSupportedCentersOfMassRecursive(node->children.at(i));
      node->supported_mass += node->children.at(i)->supported_mass;
      node->supported_com += (node->children.at(i)->supported_mass * node->children.at(i)->iTip.inverse() * node->children.at(i)->supported_com);
    }
    node->supported_com = (node->supported_mass == 0.0)? dynamics_tree::oneAugmentedZero<dynamics_tree::Vector4>() : (dynamics_tree::Vector4) (node->supported_com / node->supported_mass);
  }

  inline void DynamicsTree::supportedCenterOfMass(boost::shared_ptr<DynamicsTreeNode> node, dynamics_tree::Vector4& com, double& supported_mass)
  {
    //boost::this_thread::interruption_point();
    supported_mass = node->mass;
    com = node->mass * node->iTicom.topRightCorner(4, 1);
    dynamics_tree::Vector4 zero_point = dynamics_tree::Vector4::Zero();
    zero_point(3) = 1.0;

    dynamics_tree::Matrix4 nodeTlink = dynamics_tree::Matrix4::Identity();
    for(unsigned int i = 0; i < node->supported.size(); i++) //todo: cache supported results and / or compute recursively
    {
      lookupTransform(node->link_name, node->supported.at(i)->link_name, nodeTlink);
      supported_mass += node->supported.at(i)->mass;
      com += node->supported.at(i)->mass * nodeTlink * node->supported.at(i)->iTicom * zero_point;
    }
    com /= supported_mass;
  }

  inline void DynamicsTree::centerOfMass(std::string target_frame, dynamics_tree::Vector4& com)
  {
    //boost::this_thread::interruption_point();
    com = dynamics_tree::Vector4::Zero();
    dynamics_tree::Vector4 zero_point = dynamics_tree::Vector4::Zero();
    zero_point(3) = 1.0;
    for(std::map<std::string, boost::shared_ptr<DynamicsTreeNode> >::iterator iter = m_nodes.begin(); iter != m_nodes.end(); iter++)
    {
      com += iter->second->mass * iter->second->iTO.inverse() * iter->second->iTicom * zero_point / m_total_mass;
    }

    if(target_frame != m_inertial_frame_name)
    {
      //com is in the inertial frame, move it to the root of the tree so we can transform it to the target frame
      com = m_root_node->iTO * com; //TODO: don't transform twice!
      transformPoint(target_frame, m_root_node->link_name, com, com);
    }
  }

  inline bool DynamicsTree::getSupportingSubset(std::vector<unsigned int> desired_joint_indices, std::string tool_frame, std::vector<unsigned int>& supporting_joint_indices)
  {
    //boost::this_thread::interruption_point();
    if(m_nodes.find(tool_frame) == m_nodes.end())
    {
      std::cerr << "Couldn't find tool frame " << tool_frame << " in the tree!" << std::endl;
      return false;
    }

    boost::shared_ptr<DynamicsTreeNode> tool_node = m_nodes[tool_frame];
    supporting_joint_indices.clear();
    for(unsigned int i = 0; i < desired_joint_indices.size(); i++)
    {
      boost::shared_ptr<DynamicsTreeNode> joint_node = getNodeByIndex(desired_joint_indices[i]);
      if(!joint_node)
      {
        std::cerr << "Couldn't look up joint at index " << desired_joint_indices[i] << "!" << std::endl;
        return false;
      }
      if(joint_node->supports(tool_node)) //todo: buffer support selection for speed
      {
        supporting_joint_indices.push_back(desired_joint_indices.at(i));
      }
    }

    return true;
  }

  //Jacobian-based functions assume that the tool frame exists in the tree. Use addFixedFrame if it's not part of the URDF
  inline bool DynamicsTree::jacobian(std::vector<unsigned int> joint_indices, std::string goal_frame, std::string tool_frame, dynamics_tree::Matrix& jacobian)
  {
    if(m_nodes.find(tool_frame) == m_nodes.end())
    {
      std::cerr << "Couldn't find tool frame " << tool_frame << " in the tree!" << std::endl;
      return false;
    }
    if(m_nodes.find(goal_frame) == m_nodes.end())
    {
      std::cerr << "Couldn't find goal frame " << goal_frame << " in the tree!" << std::endl;
      return false;
    }

    boost::shared_ptr<DynamicsTreeNode> tool_node = m_nodes[tool_frame];
    boost::shared_ptr<DynamicsTreeNode> goal_node = m_nodes[goal_frame];
    jacobian.resize(6, joint_indices.size());
    for(unsigned int i = 0; i < joint_indices.size(); i++)
    {
      //boost::this_thread::interruption_point();
      boost::shared_ptr<DynamicsTreeNode> joint_node = getNodeByIndex(joint_indices[i]);
      if(!joint_node)
      {
        std::cerr << "Couldn't look up joint at index " << joint_indices[i] << "!" << std::endl;
        return false;
      }
      if(!joint_node->supports(tool_node)) //todo: buffer support selection for speed
      {
        jacobian.block<6, 1>(0, i) = dynamics_tree::Vector6::Zero();
        continue;
      }

      dynamics_tree::Matrix4 goalTtool = goal_node->iTO * tool_node->iTO.inverse();
      dynamics_tree::Matrix4 goalTjoint = goal_node->iTO * joint_node->iTO.inverse();
      goalTjoint.topRightCorner(3, 1) -= goalTtool.topRightCorner(3, 1); //translate goal to tool frame
      dynamics_tree::Matrix6 goalXjoint = dynamics_tree::motionTransformFromAffine(goalTjoint);
      jacobian.block<6, 1>(0, i) = goalXjoint * joint_node->axis;
    }
    return true;
  }

  //twist specified in goal_frame
  //uses jacobian transpose, not inverse
  inline bool DynamicsTree::cartesianVelocityToJointVelocities(std::vector<unsigned int> joint_indices, dynamics_tree::Vector6 twist, std::string goal_frame, std::string tool_frame, std::vector<double>& joint_velocities)
  {
    if(m_nodes.find(tool_frame) == m_nodes.end())
    {
      std::cerr << "Couldn't find tool frame " << tool_frame << " in the tree!" << std::endl;
      return false;
    }
    if(m_nodes.find(goal_frame) == m_nodes.end())
    {
      std::cerr << "Couldn't find goal frame " << goal_frame << " in the tree!" << std::endl;
      return false;
    }

    boost::shared_ptr<DynamicsTreeNode> tool_node = m_nodes[tool_frame];
    boost::shared_ptr<DynamicsTreeNode> goal_node = m_nodes[goal_frame];

    joint_velocities.resize(joint_indices.size());
    for(unsigned int i = 0; i < joint_indices.size(); i++)
    {
      //boost::this_thread::interruption_point();
      boost::shared_ptr<DynamicsTreeNode> joint_node = getNodeByIndex(joint_indices[i]);
      if(!joint_node)
      {
        std::cerr << "Couldn't look up joint at index " << joint_indices[i] << "!" << std::endl;
        return false;
      }
      if(!joint_node->supports(tool_node)) //todo: buffer support for speed
      {
        joint_velocities.at(i) = 0.0;
        continue;
      }

      dynamics_tree::Matrix4 goalTtool = goal_node->iTO * tool_node->iTO.inverse();
      dynamics_tree::Matrix4 goalTjoint = goal_node->iTO * joint_node->iTO.inverse();
      goalTjoint.topRightCorner(3, 1) -= goalTtool.topRightCorner(3, 1); //translate goal to tool frame
      dynamics_tree::Matrix6 goalXjoint = dynamics_tree::motionTransformFromAffine(goalTjoint);
      dynamics_tree::Vector6 Ji = goalXjoint * joint_node->axis;

      //JdQ = dx
      //J: R6xn
      //dQ = J^T * e * (<e, J*J^T*e> / <J*J^T*e, J*J^T*e>)

      dynamics_tree::Vector6 JJTe = Ji * Ji.transpose() * twist;
      double alpha = (double) (twist.transpose() * JJTe) / (double) (JJTe.transpose() * JJTe); //possibly wrong
      if(std::isnan(alpha) || std::isinf(alpha)) //usually happens when error is zero
      {
        alpha = 0;
      }
      joint_velocities.at(i) = alpha * Ji.transpose() * twist;
    }

    return true;
  }
}

#endif //DYNAMICS_TREE_META_PROPERTIES_H
