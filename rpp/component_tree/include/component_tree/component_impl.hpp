#ifndef COMPONENT_IMPL_HPP
#define COMPONENT_IMPL_HPP

#include <boost/algorithm/string.hpp>

namespace rpp
{
  inline Component::Component(pugi::xml_node& node)
  {
    xml = node;
    depth = 0;
    num_supported_components = 1;
  }

  inline Component::~Component()
  {
  }

  inline void Component::computeDepth()
  {
    for(auto c : children)
    {
      c->depth = depth + 1;
      c->computeDepth();
      num_supported_components += c->num_supported_components;
    }
  }

  inline std::ostream& operator<<(std::ostream& stream, Component& node)
  {
    std::ostringstream oss;
    node.xml.print(oss, "  ");
    char tab_char = '.';
    std::string xml_string = std::string(node.depth, tab_char) + oss.str();
    boost::replace_last(xml_string, "\n", "");
    boost::replace_all(xml_string, "\n", "\n" + std::string(node.depth, tab_char));
    stream << xml_string << std::endl;

    for(auto c : node.children)
    {
      stream << *c;
    }
    return stream;
  }
}

#endif //COMPONENT_IMPL_HPP
