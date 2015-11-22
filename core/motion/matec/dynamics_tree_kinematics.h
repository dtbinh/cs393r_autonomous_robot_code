#ifndef DYNAMICS_TREE_KINEMATICS_H
#define DYNAMICS_TREE_KINEMATICS_H

namespace dynamics_tree
{
  inline void DynamicsTree::kinematics(std::vector<double>& positions, dynamics_tree::Matrix4& rootTworld)
  {
    if(!m_inertial_frame_name_set)
    {
      m_inertial_frame_name = "world"; //root_link_odom.header.frame_id;
      m_inertial_frame_name_set = true;
    }

    m_root_node->iTO = rootTworld;
    m_root_node->iXO = motionTransformFromAffine(m_root_node->iTO);

    std::vector<DynamicsTreeNode::RecursionOperation> forward, backward;
    std::vector<double> velocities, accelerations, torques;
    std::vector<dynamics_tree::Vector6> external_wrenches;
    forward.push_back(boost::bind(&DynamicsTree::kinematicsPass, this, _1, _2, _3, _4, _5, _6));
    m_root_node->recurse(forward, backward, positions, velocities, accelerations, torques, external_wrenches);
  }

  inline bool DynamicsTree::addFixedFrame(std::string parent, std::string child, dynamics_tree::Matrix4 cTp)
  {
    if(m_nodes.find(parent) == m_nodes.end())
    {
      std::cerr << "Couldn't find parent " << parent << " in the tree!" << std::endl;
      return false;
    }

//    std::cerr << "Adding fixed frame with transform " << child << "T" << parent << "=" << std::endl;
//    std::cerr << cTp << std::endl;

    if(m_nodes.find(child) != m_nodes.end()) //child already exists! clean it up before we replace it
    {
      for(unsigned int i = 0; i < m_nodes[child]->parent->children.size(); i++)
      {
        if(m_nodes[child]->parent->children.at(i) == m_nodes[child])
        {
          m_nodes[child]->parent->children.erase(m_nodes[child]->parent->children.begin() + i);
        }
      }
      //todo: recursively delete children of child
    }

    m_nodes[child].reset(new DynamicsTreeNode());
    m_nodes[child]->iTip = cTp;
    m_nodes[child]->axis = dynamics_tree::Vector6::Ones();
    m_nodes[child]->E_fixed = cTp.topLeftCorner(3, 3);
    m_nodes[child]->iTicom = dynamics_tree::Matrix4::Identity();
    m_nodes[child]->joint_type = FIXED_JOINT;
    m_nodes[child]->iXip = dynamics_tree::motionTransformFromAffine(m_nodes[child]->iTip);
    m_nodes[child]->iXicom = dynamics_tree::motionTransformFromAffine(m_nodes[child]->iTicom);
    m_nodes[child]->mass = 1e-12;
    m_nodes[child]->link_inertia = dynamics_tree::constructInertia(m_nodes[child]->mass, dynamics_tree::Vector3::Zero(), dynamics_tree::Matrix3::Identity() * m_nodes[child]->mass);
    m_nodes[child]->link_name = child;
    m_nodes[child]->parent = m_nodes[parent];
    m_nodes[child]->r_fixed = -m_nodes[child]->E_fixed.inverse() * cTp.topRightCorner(3, 1);
    m_nodes[child]->self = m_nodes[child];
    m_nodes[child]->joint_name = parent + child;
    m_nodes[parent]->children.push_back(m_nodes[child]);

    m_root_node->updateSupport();

    return true;
  }

  inline bool DynamicsTree::updateFixedFrameOffset(std::string name, dynamics_tree::Matrix4 iTip)
  {
    if(m_nodes.find(name) == m_nodes.end())
    {
      std::cerr << "Couldn't find parent " << name << " in the tree!" << std::endl;
      return false;
    }

    m_nodes[name]->E_fixed = iTip.topLeftCorner(3, 3);
    m_nodes[name]->r_fixed = -m_nodes[name]->E_fixed.inverse() * iTip.topRightCorner(3, 1);
    m_nodes[name]->iTip = iTip;
    m_nodes[name]->iXip = dynamics_tree::motionTransformFromAffine(m_nodes[name]->iTip);
    m_nodes[name]->link_name = name;

    return true;
  }

  inline void DynamicsTree::kinematicsPass(boost::shared_ptr<DynamicsTreeNode> node, std::vector<double>& positions, std::vector<double>& velocities, std::vector<double>& accelerations, std::vector<double>& torques, std::vector<dynamics_tree::Vector6>& external_wrenches)
  {
    //boost::this_thread::interruption_point();
    if(node->parent)
    {
      double q = 0.0;
      if(node->joint_type != FIXED_JOINT)
      {
        q = positions.at(node->joint_idx);
      }
      if(q != node->q) //don't repeat past work
      {
        node->q = q;
        dynamics_tree::Matrix3 iEip = Eigen::AngleAxisd(node->q, node->axis.topRows(3)).toRotationMatrix().inverse() * node->E_fixed;

        //from Featherstone:
        //r = position of i in ip coordinates
        //E = rotation matrix to i in ip coordinates
        //iXip = [E,      0]
        //       [-E(rx), E]
        //iTip = [E,    -Er]
        //       [0,      1]

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
  }

  //transforms
  inline bool DynamicsTree::lookupTransform(std::string target_frame, std::string source_frame, dynamics_tree::Matrix4& tTs)
  {
    //todo: restore efficiency
    if(source_frame == target_frame)
    {
      tTs = dynamics_tree::Matrix4::Identity();
      return true;
    }

    dynamics_tree::Matrix4 targetTO, sourceTO;
    std::map<std::string, boost::shared_ptr<DynamicsTreeNode> >::iterator target_iter = m_nodes.find(target_frame);
    if(target_iter != m_nodes.end())
    {
      targetTO = target_iter->second->iTO;
    }
    else if(target_frame == m_inertial_frame_name)
    {
      targetTO = dynamics_tree::Matrix4::Identity();
    }
    else
    {
      std::cerr << "Couldn't find target frame " << target_frame << " in the tree!" << std::endl;
      return false;
    }

    std::map<std::string, boost::shared_ptr<DynamicsTreeNode> >::iterator source_iter = m_nodes.find(source_frame);
    if(source_iter != m_nodes.end())
    {
      sourceTO = source_iter->second->iTO;
    }
    else if(source_frame == m_inertial_frame_name)
    {
      sourceTO = dynamics_tree::Matrix4::Identity();
    }
    else
    {
      std::cerr << "Couldn't find source frame " << source_frame << " in the tree!" << std::endl;
      return false;
    }

    tTs = targetTO * sourceTO.inverse();
    return true;
  }

  inline bool DynamicsTree::lookupMotionTransform(std::string target_frame, std::string source_frame, dynamics_tree::Matrix6& tXs)
  {
    //todo: restore efficiency
    if(source_frame == target_frame)
    {
      tXs = dynamics_tree::Matrix6::Identity();
      return true;
    }

    dynamics_tree::Matrix6 targetXO, sourceXO;
    std::map<std::string, boost::shared_ptr<DynamicsTreeNode> >::iterator target_iter = m_nodes.find(target_frame);
    if(target_iter != m_nodes.end())
    {
      targetXO = target_iter->second->iXO;
    }
    else if(target_frame == m_inertial_frame_name)
    {
      targetXO = dynamics_tree::Matrix6::Identity();
    }
    else
    {
      std::cerr << "Couldn't find target frame " << target_frame << " in the tree!" << std::endl;
      return false;
    }

    std::map<std::string, boost::shared_ptr<DynamicsTreeNode> >::iterator source_iter = m_nodes.find(source_frame);
    if(source_iter != m_nodes.end())
    {
      sourceXO = source_iter->second->iXO;
    }
    else if(source_frame == m_inertial_frame_name)
    {
      sourceXO = dynamics_tree::Matrix6::Identity();
    }
    else
    {
      std::cerr << "Couldn't find source frame " << source_frame << " in the tree!" << std::endl;
      return false;
    }

    tXs = targetXO * sourceXO.inverse();
    return true;
  }

  //vectors
  inline bool DynamicsTree::transformVector(std::string target_frame, std::string source_frame, dynamics_tree::Vector3 input, dynamics_tree::Vector3& output)
  {
    if(source_frame == target_frame)
    {
      output = input;
      return true;
    }

    dynamics_tree::Matrix4 tTs;
    if(!lookupTransform(target_frame, source_frame, tTs))
    {
      return false;
    }

    output = tTs.topLeftCorner(3, 3) * input;
    return true;
  }

  //points
  inline bool DynamicsTree::transformPoint(std::string target_frame, std::string source_frame, dynamics_tree::Vector4 input, dynamics_tree::Vector4& output)
  {
    if(source_frame == target_frame)
    {
      output = input;
      return true;
    }

    dynamics_tree::Matrix4 tTs;
    if(!lookupTransform(target_frame, source_frame, tTs))
    {
      return false;
    }

    output = tTs * input;
    return true;
  }

  inline bool DynamicsTree::transformPoint(std::string target_frame, std::string source_frame, dynamics_tree::Vector3 input, dynamics_tree::Vector3& output)
  {
    dynamics_tree::Vector4 augmented_input, augmented_output;
    augmented_input.topRows(3) = input;
    augmented_input(3) = 1.0;
    transformPoint(target_frame, source_frame, augmented_input, augmented_output);
    output = augmented_output.topRows(3);

    return true;
  }

  //poses
  inline bool DynamicsTree::transformPose(std::string target_frame, std::string source_frame, dynamics_tree::Matrix4 input, dynamics_tree::Matrix4& output)
  {
    if(source_frame == target_frame)
    {
      output = input;
      return true;
    }

    dynamics_tree::Matrix4 tTs;
    if(!lookupTransform(target_frame, source_frame, tTs))
    {
      return false;
    }

    output = tTs * input;
    return true;
  }

  //twists
  inline bool DynamicsTree::transformTwist(std::string target_frame, std::string source_frame, dynamics_tree::Vector6 input, dynamics_tree::Vector6& output)
  {
    if(source_frame == target_frame)
    {
      output = input;
      return true;
    }

    dynamics_tree::Matrix6 tXs;
    if(!lookupMotionTransform(target_frame, source_frame, tXs))
    {
      return false;
    }

    output = tXs * input;
    return true;
  }

  //wrenches
  inline bool DynamicsTree::transformWrench(std::string target_frame, std::string source_frame, dynamics_tree::Vector6 input, dynamics_tree::Vector6& output)
  {
    if(source_frame == target_frame)
    {
      output = input;
      return true;
    }

    dynamics_tree::Matrix6 sXt;
    if(!lookupMotionTransform(source_frame, target_frame, sXt))
    {
      return false;
    }

    //output = tXs.inverse().transpose() * input;
    output = sXt.transpose() * input;
    return true;
  }
}

#endif //DYNAMICS_TREE_KINEMATICS_H
