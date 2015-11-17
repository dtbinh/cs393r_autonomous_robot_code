#ifndef DYNAMICS_GRAPH_H
#define DYNAMICS_GRAPH_H

#include <component_tree/component_tree.hpp>
#include "common_functions.h"
#include "dynamics_tree.h"
#include "matec_joint.h"
#include "matec_link.h"

namespace dynamics_tree
{
  class DynamicsGraph
  {
  public:
    DynamicsGraph()
    {
    }

    void loadFromURDF(std::vector<std::string>& joint_names, rpp::ComponentTree& model, bool floating_base = false)
    {
      m_joint_names = joint_names;
      loadRecursive(model.getRootNode());
    }

    void spawnDynamicsTree(std::string root_link, bool floating_base, dynamics_tree::DynamicsTree& tree)
    {
      tree.reset();
      tree.setJointNames(m_joint_names);
      tree.setFloatingBase(floating_base); //TODO: make floating base a config option

      //set up root node
      m_links[root_link]->linkToriginallink = dynamics_tree::Matrix4::Identity();

      boost::shared_ptr<DynamicsTreeNode> root;
      root.reset(new DynamicsTreeNode());
      root->self = root;
      root->parent = boost::shared_ptr<DynamicsTreeNode>();
      root->joint_type = floating_base? FLOATING_BASE : FIXED_BASE;
      root->mass = m_links[root_link]->mass;
      root->iTicom = m_links[root_link]->originallinkTlinkcom;
      root->link_name = m_links[root_link]->name;
      root->link_inertia = dynamics_tree::transformInertia(m_links[root_link]->linkToriginallink, m_links[root_link]->spatial_inertia);
      tree.setNode(root);
      tree.setRootNode(root);

      //add the rest
      m_total_mass = root->mass;
      addDynamicsTreeLinkRecursive(m_links[root_link], boost::shared_ptr<Joint>(), tree);
      tree.setTotalMass(m_total_mass);

      root->updateSupport();
    }

    void print(std::string root_link)
    {
      printLinkRecursive(m_links[root_link], boost::shared_ptr<Joint>(), 0);
    }

  private:
    std::vector<std::string> m_joint_names;
    std::map<std::string, boost::shared_ptr<Link> > m_links;
    std::map<std::string, boost::shared_ptr<Joint> > m_joints;
    double m_total_mass;

    void addDynamicsTreeLinkRecursive(boost::shared_ptr<Link> link, boost::shared_ptr<Joint> caller, dynamics_tree::DynamicsTree& tree)
    {
      bool found_caller = false;
      for(unsigned int i = 0; i < link->connected_joints.size(); i++)
      {
        if(caller && caller == link->connected_joints[i])
        {
          found_caller = true;
          continue;
        }
        addDynamicsTreeJointRecursive(link->connected_joints[i], link, tree);
      }

      if(caller && !found_caller)
      {
        std::cerr << "DIDN'T FIND CALLER " << caller->name << " AT LINK " << link->name << "!" << std::endl;
      }
    }

    void addDynamicsTreeJointRecursive(boost::shared_ptr<Joint> joint, boost::shared_ptr<Link> caller, dynamics_tree::DynamicsTree& tree)
    {
      boost::shared_ptr<Link> next;
      double axis_flip;
      if(caller == joint->parent_link)
      {
        next = joint->child_link;
        axis_flip = 1.0;
      }
      else if(caller == joint->child_link)
      {
        next = joint->parent_link;
        axis_flip = -1.0;
      }
      else
      {
        std::cerr << "CALLER WAS NEITHER PARENT NOR CHILD OF THIS JOINT!" << std::endl;
        return;
      }

//      if(joint->type == FIXED_JOINT)
//      {
//        //determine necessary shifts
//        dynamics_tree::Matrix4 iTip = caller->jointToriginallink[caller->findConnectedJointIndex(joint)] * caller->linkToriginallink.inverse(); //adjust parent transform in case the parent frame shifted due to tree restructuring
//        next->linkToriginallink = next->jointToriginallink[next->findConnectedJointIndex(joint)]; //move the link frame to the joint frame
//      }
//      else
//      {

      //determine necessary shifts
      dynamics_tree::Matrix4 iTip = caller->jointToriginallink[caller->findConnectedJointIndex(joint)] * caller->linkToriginallink.inverse(); //adjust parent transform in case the parent frame shifted due to tree restructuring
      next->linkToriginallink = next->jointToriginallink[next->findConnectedJointIndex(joint)]; //move the link frame to the joint frame

      //populate node
      boost::shared_ptr<DynamicsTreeNode> node;
      node.reset(new DynamicsTreeNode());
      node->self = node;

      node->link_name = next->name;
      node->link_inertia = dynamics_tree::transformInertia(next->linkToriginallink, next->spatial_inertia);
      node->E_fixed = iTip.topLeftCorner(3, 3); //inverted?
      node->r_fixed = -node->E_fixed.inverse() * iTip.topRightCorner(3, 1);
      node->iTicom = next->linkToriginallink * next->originallinkTlinkcom;
      node->iXicom = dynamics_tree::motionTransformFromAffine(node->iTicom);
      node->mass = next->mass;
      m_total_mass += node->mass;

      //TODO: recursively merge fixed joints!
      node->axis = axis_flip * joint->axis;
      node->coulomb_friction = joint->coulomb_friction;
      node->joint_idx = joint->idx;
      node->joint_name = joint->name;
      node->joint_type = joint->type;
      node->min_q = joint->min_q;
      node->max_q = joint->max_q;
      node->max_q_dot = joint->max_q_dot;
      node->max_torque = joint->max_torque;
      node->viscous_friction = joint->viscous_friction;
      node->enforce_joint_limits = false; //TODO: param
      if(node->joint_type == FIXED_JOINT)
      {
        node->dynamics_direction = DynamicsTreeNode::INVERSE_DYNAMICS;
      }

      node->parent = tree.getNodeByLink(caller->name);
      if(!node->parent)
      {
        std::cerr << "Tree did not contain parent named " << caller->name << std::endl;
      }
      node->parent->children.push_back(node);

      tree.setNode(node);

      //recurse
      addDynamicsTreeLinkRecursive(next, joint, tree);
    }

    void tabOut(int tab_depth)
    {
      for(int i = 0; i < tab_depth - 1; i++)
      {
        if(i % 2 == 0)
        {
          std::cerr << "-";
        }
        else
        {
          std::cerr << "=";
        }
      }
      if(tab_depth != 0)
      {
        std::cerr << ">";
      }
    }

    void printLinkRecursive(boost::shared_ptr<Link> link, boost::shared_ptr<Joint> caller, int tab_depth)
    {
      tabOut(tab_depth);
      std::cerr << "(link) " << link->name << std::endl;

      bool found_caller = false;
      for(unsigned int i = 0; i < link->connected_joints.size(); i++)
      {
        if(caller && caller == link->connected_joints[i])
        {
          found_caller = true;
          continue;
        }
        printJointRecursive(link->connected_joints[i], link, tab_depth + 1);
      }

      if(caller && !found_caller)
      {
        std::cerr << "DIDN'T FIND CALLER " << caller->name << " AT LINK " << link->name << "!" << std::endl;
      }
    }

    void printJointRecursive(boost::shared_ptr<Joint> joint, boost::shared_ptr<Link> caller, int tab_depth)
    {
      if(caller == joint->parent_link)
      {
        dynamics_tree::Matrix4 parent_transform = (joint->parent_link->jointToriginallink[joint->parent_link->findConnectedJointIndex(joint)] * joint->child_link->jointToriginallink[joint->child_link->findConnectedJointIndex(joint)].inverse());

        tabOut(tab_depth);
        std::cerr << "(joint) " << joint->name << ":" << parent_transform.topRightCorner(3, 1).transpose() << std::endl;
        printLinkRecursive(joint->child_link, joint, tab_depth + 1);
      }
      else if(caller == joint->child_link)
      {
        dynamics_tree::Matrix4 parent_transform = (joint->child_link->jointToriginallink[joint->child_link->findConnectedJointIndex(joint)] * joint->parent_link->jointToriginallink[joint->parent_link->findConnectedJointIndex(joint)].inverse());

        tabOut(tab_depth);
        std::cerr << "(joint*) " << joint->name << ":" << parent_transform.topRightCorner(3, 1).transpose() << std::endl;
        printLinkRecursive(joint->parent_link, joint, tab_depth + 1);
      }
      else
      {
        std::cerr << "CALLER WAS NEITHER PARENT NOR CHILD OF THIS JOINT!" << std::endl;
      }
    }

    void loadRecursive(boost::shared_ptr<rpp::Component> link)
    {
      m_links[link->name].reset(new Link());
      m_links[link->name]->name = link->name;

      m_links[link->name]->mass = 1e-12;
      m_links[link->name]->com_position = dynamics_tree::Vector3::Zero();
      m_links[link->name]->com_inertia = dynamics_tree::Matrix3::Identity() * 1e-12;
//      if(link->inertial)
//      {
//        m_links[link->name]->com_position << link->inertial->origin.position.x, link->inertial->origin.position.y, link->inertial->origin.position.z;
//        m_links[link->name]->com_inertia << link->inertial->ixx, link->inertial->ixy, link->inertial->ixz, //
//        link->inertial->ixy, link->inertial->iyy, link->inertial->iyz, //
//        link->inertial->ixz, link->inertial->iyz, link->inertial->izz;
//        m_links[link->name]->mass = link->inertial->mass;
//      }
//      m_links[link->name]->spatial_inertia = dynamics_tree::constructInertia(m_links[link->name]->mass, m_links[link->name]->com_position, m_links[link->name]->com_inertia);
//      m_links[link->name]->originallinkTlinkcom.topRightCorner(3, 1) = -m_links[link->name]->com_position;
//
//      if(link->parent_joint)
//      {
//        //create and link the joint
//        m_joints[link->parent_joint->name].reset(new Joint());
//        m_joints[link->parent_joint->name]->name = link->parent_joint->name;
//        m_links[link->name]->connected_joints.push_back(m_joints[link->parent_joint->name]);
//        m_links[link->name]->jointToriginallink.push_back(dynamics_tree::Matrix4::Identity());
//        m_joints[link->parent_joint->name]->child_link = m_links[link->name];
//
//        //set the type, axis, and index
//        switch(link->parent_joint->type)
//        {
//        case urdf::Joint::REVOLUTE:
//          m_joints[link->parent_joint->name]->type = REVOLUTE_JOINT;
//          m_joints[link->parent_joint->name]->axis << link->parent_joint->axis.x, link->parent_joint->axis.y, link->parent_joint->axis.z, 0, 0, 0;
//          m_joints[link->parent_joint->name]->idx = std::find(m_joint_names.begin(), m_joint_names.end(), m_joints[link->parent_joint->name]->name) - m_joint_names.begin();
//          break;
//        case urdf::Joint::PRISMATIC:
//          m_joints[link->parent_joint->name]->type = PRISMATIC_JOINT;
//          m_joints[link->parent_joint->name]->axis << 0, 0, 0, link->parent_joint->axis.x, link->parent_joint->axis.y, link->parent_joint->axis.z;
//          m_joints[link->parent_joint->name]->idx = std::find(m_joint_names.begin(), m_joint_names.end(), m_joints[link->parent_joint->name]->name) - m_joint_names.begin();
//          break;
//        default:
//        case urdf::Joint::FIXED:
//          m_joints[link->parent_joint->name]->type = FIXED_JOINT;
//          m_joints[link->parent_joint->name]->axis << 1, 0, 0, 0, 0, 0; //arbitrary
//          m_joints[link->parent_joint->name]->idx = -1;
//          break;
//        }
//
//        //check joint index
//        if((m_joints[link->parent_joint->name]->idx == (int) m_joint_names.size()) && m_joints[link->parent_joint->name]->type != FIXED_JOINT)
//        {
//          ROS_DEBUG("JOINT %s ASSOCIATED WITH LINK %s WAS NOT FOUND IN THE LIST OF JOINT NAMES! CONVERTING IT INTO A FIXED JOINT!!", m_joints[link->parent_joint->name]->name.c_str(), link->name.c_str());
//          m_joints[link->parent_joint->name]->type = FIXED_JOINT;
//        }
//
//        //set limits
//        if(link->parent_joint->limits)
//        {
//          m_joints[link->parent_joint->name]->max_torque = link->parent_joint->limits->effort;
//          m_joints[link->parent_joint->name]->max_q_dot = link->parent_joint->limits->velocity;
//          m_joints[link->parent_joint->name]->min_q = link->parent_joint->limits->lower;
//          m_joints[link->parent_joint->name]->max_q = link->parent_joint->limits->upper;
//        }
//
//        //set dynamic properties
//        if(link->parent_joint->dynamics)
//        {
//          m_joints[link->parent_joint->name]->viscous_friction = link->parent_joint->dynamics->damping;
//          m_joints[link->parent_joint->name]->coulomb_friction = link->parent_joint->dynamics->friction;
//        }
//      }
//
//      for(unsigned int i = 0; i < link->child_links.size(); i++)
//      {
//        loadRecursive(link->child_links[i]);
//      }
//
//      for(unsigned int i = 0; i < link->child_joints.size(); i++)
//      {
//        m_links[link->name]->connected_joints.push_back(m_joints[link->child_joints[i]->name]);
//
//        dynamics_tree::Matrix4 joint_transform = dynamics_tree::Matrix4::Identity();
//        dynamics_tree::Matrix3 E = Eigen::Quaterniond(link->child_joints[i]->parent_to_joint_origin_transform.rotation.w, link->child_joints[i]->parent_to_joint_origin_transform.rotation.x, link->child_joints[i]->parent_to_joint_origin_transform.rotation.y, link->child_joints[i]->parent_to_joint_origin_transform.rotation.z).toRotationMatrix();
//        dynamics_tree::Vector3 r;
//        r << link->child_joints[i]->parent_to_joint_origin_transform.position.x, link->child_joints[i]->parent_to_joint_origin_transform.position.y, link->child_joints[i]->parent_to_joint_origin_transform.position.z;
//        joint_transform.topLeftCorner(3, 3) = E.inverse();
//        joint_transform.topRightCorner(3, 1) = -E.inverse() * r;
//
//        m_links[link->name]->jointToriginallink.push_back(joint_transform);
//        m_joints[link->child_joints[i]->name]->parent_link = m_links[link->name];
//      }
    }
  };
}

#endif //DYNAMICS_GRAPH_H
