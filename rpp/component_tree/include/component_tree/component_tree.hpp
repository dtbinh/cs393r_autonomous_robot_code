#ifndef component_tree_HPP
#define component_tree_HPP

#define PUGIXML_HEADER_ONLY
#include "component_tree/pugixml.hpp"
#include "component_tree/pugixml.cpp"
#include <iostream>
#include <fstream>
#include <boost/algorithm/string.hpp>
#include <boost/format.hpp>
#include <boost/regex.hpp>
#include <boost/shared_ptr.hpp>

namespace rpp
{
  class Component
  {
  public:
    Component(pugi::xml_node& node);
    ~Component();

    pugi::xml_node xml;
    std::string name;
    std::string type;

    unsigned int depth;
    unsigned int num_supported_components;

    boost::shared_ptr<Component> parent;
    std::vector<boost::shared_ptr<Component> > children;

    void computeDepth();

    friend std::ostream& operator<<(std::ostream& os, Component& dt);
  };

  class ComponentTree
  {
  public:
    ComponentTree();
    ~ComponentTree();

    bool fromString(std::string xml);
    bool fromFile(std::string filename);

    boost::shared_ptr<Component> getRootNode();
    std::vector<boost::shared_ptr<Component> > getRootNodes();

    friend std::ostream& operator<<(std::ostream& os, ComponentTree& dt);

  private:
    pugi::xml_document m_doc;
    std::map<std::string, boost::shared_ptr<Component> > m_component_tree;
    std::vector<boost::shared_ptr<Component> > m_roots;
  };
}

#include <component_tree/component_impl.hpp>
#include <component_tree/component_tree_impl.hpp>

#endif
