#ifndef DYNAMICS_GRAPH_H
#define DYNAMICS_GRAPH_H

#include <live_params/string_parameter_parsers.hpp>
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
      loadRecursive(model.rootNode());
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
      if(link->xml.child("inertial"))
      {
        m_links[link->name]->mass = link->xml.child("inertial").child("mass").attribute("value").as_double();

        std::vector<double> xyz = rpp::parameterStringToFPVector(link->xml.child("inertial").child("origin").attribute("xyz").as_string());

        m_links[link->name]->com_position << xyz[0], xyz[1], xyz[2];

        pugi::xml_node in = link->xml.child("inertial").child("inertia");
        m_links[link->name]->com_inertia << in.attribute("ixx").as_double(), in.attribute("ixy").as_double(), in.attribute("ixz").as_double(), //
        in.attribute("ixy").as_double(), in.attribute("iyy").as_double(), in.attribute("iyz").as_double(), //
        in.attribute("ixz").as_double(), in.attribute("iyz").as_double(), in.attribute("izz").as_double();
      }
      m_links[link->name]->spatial_inertia = dynamics_tree::constructInertia(m_links[link->name]->mass, m_links[link->name]->com_position, m_links[link->name]->com_inertia);
      m_links[link->name]->originallinkTlinkcom.topRightCorner(3, 1) = -m_links[link->name]->com_position;

      if(link->parent && link->parent->type == "joint")
      {
        //create and link the joint
        m_joints[link->parent->name].reset(new Joint());
        m_joints[link->parent->name]->name = link->parent->name;
        m_links[link->name]->connected_joints.push_back(m_joints[link->parent->name]);
        m_links[link->name]->jointToriginallink.push_back(dynamics_tree::Matrix4::Identity());
        m_joints[link->parent->name]->child_link = m_links[link->name];

        //set the type, axis, and index
        std::string joint_type = link->parent->xml.attribute("type").as_string();
        if(joint_type == "revolute")
        {
          std::vector<double> xyz = rpp::parameterStringToFPVector(link->parent->xml.child("axis").attribute("xyz").as_string());
          m_joints[link->parent->name]->type = REVOLUTE_JOINT;
          m_joints[link->parent->name]->axis << xyz[0], xyz[1], xyz[2], 0, 0, 0;
          m_joints[link->parent->name]->idx = std::find(m_joint_names.begin(), m_joint_names.end(), m_joints[link->parent->name]->name) - m_joint_names.begin();
        }
        else if(joint_type == "prismatic")
        {
          std::vector<double> xyz = rpp::parameterStringToFPVector(link->parent->xml.child("axis").attribute("xyz").as_string());
          m_joints[link->parent->name]->type = PRISMATIC_JOINT;
          m_joints[link->parent->name]->axis << 0, 0, 0, xyz[0], xyz[1], xyz[2];
          m_joints[link->parent->name]->idx = std::find(m_joint_names.begin(), m_joint_names.end(), m_joints[link->parent->name]->name) - m_joint_names.begin();
        }
        else
        {
          m_joints[link->parent->name]->type = FIXED_JOINT;
          m_joints[link->parent->name]->axis << 1, 0, 0, 0, 0, 0; //arbitrary
          m_joints[link->parent->name]->idx = -1;
        }

        //check joint index
        if((m_joints[link->parent->name]->idx == (int) m_joint_names.size()) && m_joints[link->parent->name]->type != FIXED_JOINT)
        {
          ROS_DEBUG("JOINT %s ASSOCIATED WITH LINK %s WAS NOT FOUND IN THE LIST OF JOINT NAMES! CONVERTING IT INTO A FIXED JOINT!!", m_joints[link->parent->name]->name.c_str(), link->name.c_str());
          m_joints[link->parent->name]->type = FIXED_JOINT;
        }

        //set limits
        if(link->parent->xml.child("limits"))
        {
          m_joints[link->parent->name]->max_torque = link->parent->xml.child("limits").attribute("effort").as_double();
          m_joints[link->parent->name]->max_q_dot = link->parent->xml.child("limits").attribute("velocity").as_double();
          m_joints[link->parent->name]->min_q = link->parent->xml.child("limits").attribute("lower").as_double();
          m_joints[link->parent->name]->max_q = link->parent->xml.child("limits").attribute("upper").as_double();
        }

        //set dynamic properties
        if(link->parent->xml.child("dynamics"))
        {
          m_joints[link->parent->name]->viscous_friction = link->parent->xml.child("dynamics").attribute("damping").as_double();
          m_joints[link->parent->name]->coulomb_friction = link->parent->xml.child("dynamics").attribute("friction").as_double();
        }
      }

      for(auto cj : link->children)
      {
        if(cj->type != "joint")
        {
          loadRecursive(cj);
          continue;
        }

        for(auto cl : cj->children)
          loadRecursive(cl);

        std::vector<double> xyz = rpp::parameterStringToFPVector(cj->xml.child("origin").attribute("xyz").as_string());
        std::vector<double> rpy = rpp::parameterStringToFPVector(cj->xml.child("origin").attribute("rpy").as_string());

        dynamics_tree::Matrix4 jointToriginallink = dynamics_tree::Matrix4::Identity();
        dynamics_tree::Matrix3 E = dynamics_tree::rotationMatrix(rpy[0], rpy[1], rpy[2]);
        dynamics_tree::Vector3 r(xyz[0], xyz[1], xyz[2]);
        jointToriginallink.topLeftCorner(3, 3) = E.inverse();
        jointToriginallink.topRightCorner(3, 1) = -E.inverse() * r;

        m_links[link->name]->connected_joints.push_back(m_joints[cj->name]);
        m_links[link->name]->jointToriginallink.push_back(jointToriginallink);
        m_joints[cj->name]->parent_link = m_links[link->name];
      }
    }
  };
}

#endif //DYNAMICS_GRAPH_H
