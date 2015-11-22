#ifndef DYNAMICS_TREE_INITIALIZATION_H
#define DYNAMICS_TREE_INITIALIZATION_H

namespace dynamics_tree
{
  inline DynamicsTree::DynamicsTree()
  {
    m_inertial_frame_name_set = false;
  }

  inline void DynamicsTree::configureDynamics(std::vector<DynamicsTreeNode::DynamicsDirection> dynamics_direction_config, std::vector<ExternalWrenchFrame> external_wrench_frame_config)
  {
    m_root_node->configureDynamics(dynamics_direction_config, external_wrench_frame_config);
  }

//  inline void DynamicsTree::load(std::vector<std::string>& joint_names, urdf::Model& model, bool floating_base)
//  {
//    m_joint_names = joint_names;
//    m_floating_base = floating_base;
//    m_total_mass = 0.0;
//    loadRecursive(model.getRoot(), boost::shared_ptr<DynamicsTreeNode>());
//  }
//
//  inline boost::shared_ptr<DynamicsTreeNode> DynamicsTree::loadRecursive(boost::shared_ptr<const urdf::Link> link, boost::shared_ptr<DynamicsTreeNode> parent)
//  {
//    //TODO: pointer safety
//    //load this node
//    m_nodes[link->name].reset(new DynamicsTreeNode());
//    m_nodes[link->name]->self = m_nodes[link->name];
//    m_nodes[link->name]->parent = parent;
//    m_nodes[link->name]->enforce_joint_limits = false;
//    m_nodes[link->name]->link_name = link->name;
//    std::cerr << "Loading link " << link->name;
//
//    double mass = 1e-12;
//    dynamics_tree::Vector3 com_position = dynamics_tree::Vector3::Zero();
//    dynamics_tree::Matrix3 com_inertia = dynamics_tree::Matrix3::Identity() * 1e-12;
//    if(link->inertial)
//    {
//      com_position << link->inertial->origin.position.x, link->inertial->origin.position.y, link->inertial->origin.position.z;
//      com_inertia << link->inertial->ixx, link->inertial->ixy, link->inertial->ixz, //
//      link->inertial->ixy, link->inertial->iyy, link->inertial->iyz, //
//      link->inertial->ixz, link->inertial->iyz, link->inertial->izz;
//      mass = link->inertial->mass;
//    }
//    m_total_mass += mass;
//    m_nodes[link->name]->link_inertia = dynamics_tree::constructInertia(mass, com_position, com_inertia);
//    m_nodes[link->name]->iXicom.bottomLeftCorner(3, 3) = -dynamics_tree::cross(com_position);
//    m_nodes[link->name]->iTicom.topRightCorner(3, 1) = -com_position;
//
//    //TODO: check inertia orientation, yell if non-identity
//
//    if(link->parent_joint)
//    {
//      //TODO: handle non-fixed/non-revolute joints
//      m_nodes[link->name]->joint_type = (link->parent_joint->type == urdf::Joint::FIXED)? FIXED_JOINT : REVOLUTE_JOINT;
//      m_nodes[link->name]->joint_name = link->parent_joint->name;
//      m_nodes[link->name]->joint_idx = std::find(m_joint_names.begin(), m_joint_names.end(), m_nodes[link->name]->joint_name) - m_joint_names.begin();
//      if((m_nodes[link->name]->joint_idx == (int) m_joint_names.size()) && m_nodes[link->name]->joint_type != FIXED_JOINT)
//      {
//        ROS_ERROR("JOINT %s ASSOCIATED WITH LINK %s WAS NOT FOUND IN THE LIST OF JOINT NAMES! CONVERTING IT INTO A FIXED JOINT!!", m_nodes[link->name]->joint_name.c_str(), link->name.c_str());
//        m_nodes[link->name]->joint_type = FIXED_JOINT;
//        m_nodes[link->name]->joint_idx = -1;
//      }
//      m_nodes[link->name]->axis << link->parent_joint->axis.x, link->parent_joint->axis.y, link->parent_joint->axis.z, 0, 0, 0;
//      m_joint_index_to_node[m_nodes[link->name]->joint_idx] = m_nodes[link->name];
//      m_joint_name_to_node[m_nodes[link->name]->joint_name] = m_nodes[link->name];
//
//      //todo: handle fixed joints better
//      bool axis_ok = false;
//      for(unsigned int i = 0; i < 6; i++)
//      {
//        if(m_nodes[link->name]->axis(i) != 0.0)
//        {
//          axis_ok = true;
//          break;
//        }
//      }
//      if(!axis_ok)
//      {
//        m_nodes[link->name]->axis(0) = 1.0;
//      }
//
//      if(link->parent_joint->limits)
//      {
//        m_nodes[link->name]->max_torque = link->parent_joint->limits->effort;
//        m_nodes[link->name]->max_q_dot = link->parent_joint->limits->velocity;
//        m_nodes[link->name]->min_q = link->parent_joint->limits->lower;
//        m_nodes[link->name]->max_q = link->parent_joint->limits->upper;
//      }
//      if(link->parent_joint->dynamics)
//      {
//        m_nodes[link->name]->viscous_friction = link->parent_joint->dynamics->damping;
//        m_nodes[link->name]->coulomb_friction = link->parent_joint->dynamics->friction;
//      }
//
//      m_nodes[link->name]->r_fixed << link->parent_joint->parent_to_joint_origin_transform.position.x, //
//      link->parent_joint->parent_to_joint_origin_transform.position.y, //
//      link->parent_joint->parent_to_joint_origin_transform.position.z;
//      m_nodes[link->name]->E_fixed = Eigen::Quaterniond(link->parent_joint->parent_to_joint_origin_transform.rotation.w, link->parent_joint->parent_to_joint_origin_transform.rotation.x, link->parent_joint->parent_to_joint_origin_transform.rotation.y, link->parent_joint->parent_to_joint_origin_transform.rotation.z).toRotationMatrix().inverse();
//
//      std::cerr << " with " << ((m_nodes[link->name]->joint_type == FIXED_JOINT)? "fixed" : "revolute") << " parent joint " << m_nodes[link->name]->joint_name << "(" << m_nodes[link->name]->joint_idx << ")";
//    }
//    else if(parent)
//    {
//      std::cerr << "URDF and MATEC don't agree on lineage!" << std::endl;
//    }
//    else
//    {
//      std::cerr << " with no parent joint";
//      m_root_node = m_nodes[link->name];
//      m_root_node->joint_type = m_floating_base? FLOATING_BASE : FIXED_BASE;
//      //TODO: check for multiple nodes with no parent
//    }
//
//    std::cerr << " and children: [";
//    for(unsigned int i = 0; i < link->child_links.size(); i++)
//    {
//      if(i != 0)
//      {
//        std::cerr << ", ";
//      }
//      int idx = std::find(m_joint_names.begin(), m_joint_names.end(), link->child_links[i]->parent_joint->name) - m_joint_names.begin();
//      std::cerr << link->child_links[i]->name << " via " << link->child_links[i]->parent_joint->name << "(" << idx << ")";
//    }
//    std::cerr << "]" << std::endl;
//
//    //recurse
//    for(unsigned int i = 0; i < link->child_links.size(); i++)
//    {
//      boost::shared_ptr<DynamicsTreeNode> child = loadRecursive(link->child_links[i], m_nodes[link->name]);
//      m_nodes[link->name]->children.push_back(child);
//      m_nodes[link->name]->supported.push_back(child);
//      m_nodes[link->name]->supported.insert(m_nodes[link->name]->supported.end(), child->supported.begin(), child->supported.end());
//    }
//
//    std::cerr << "Link " << link->name << " supports: [";
//    for(unsigned int i = 0; i < m_nodes[link->name]->supported.size(); i++)
//    {
//      if(i != 0)
//      {
//        std::cerr << ", ";
//      }
//      std::cerr << m_nodes[link->name]->supported[i]->link_name;
//    }
//    std::cerr << "]" << std::endl;
//
//    return m_nodes[link->name];
//  }
}

#endif //DYNAMICS_TREE_INITIALIZATION_H
