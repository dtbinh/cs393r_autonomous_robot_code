#ifndef COMPONENT_TREE_HPP
#define COMPONENT_TREE_HPP

#define PUGIXML_HEADER_ONLY
#include "pugixml.hpp"
#include "pugixml.cpp"
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

    boost::shared_ptr<Component> rootNode();
    std::vector<boost::shared_ptr<Component> > rootNodes();
    std::set<std::string> componentTypeList();
    std::map<std::string, boost::shared_ptr<Component> > components(std::string type);
    boost::shared_ptr<Component> component(std::string type, std::string name);

    friend std::ostream& operator<<(std::ostream& os, ComponentTree& dt);

  private:
    pugi::xml_document m_doc;
    std::map<std::string, boost::shared_ptr<Component> > m_component_tree;
    std::set<std::string> m_valid_component_types;
    std::map<std::string, std::map<std::string, boost::shared_ptr<Component> > > m_type_map;
    std::vector<boost::shared_ptr<Component> > m_roots;
  };
}

#include <component_tree/component_impl.hpp>
#include <component_tree/component_tree_impl.hpp>

#endif //COMPONENT_TREE_HPP
