#include "component_tree/component_tree.hpp"
#include <ros/ros.h>
#include <ros/package.h>

int main(int argc, char **argv)
{
  rpp::ComponentTree tree;
  tree.fromFile(ros::package::getPath("component_tree") + "/arm_test.urdf");

  std::cerr << "Parsed tree:\n" << tree << std::endl;
}
