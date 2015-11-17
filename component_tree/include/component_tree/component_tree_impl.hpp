#ifndef COMPONENT_TREE_IMPL_HPP
#define COMPONENT_TREE_IMPL_HPP

namespace rpp
{
  ComponentTree::ComponentTree()
  {
  }

  ComponentTree::~ComponentTree()
  {
  }

//  struct root_less_than
//  {
//    inline bool operator()(const boost::shared_ptr<Component>& c1, const boost::shared_ptr<Component>& c2)
//    {
//      return (c1->num_supported_components > c2->num_supported_components);
//    }
//  };

  bool ComponentTree::fromString(std::string xml)
  {
    m_doc.load(xml.c_str());

    std::map<std::string, std::string> parent_of;
    for(pugi::xml_node node = m_doc.child("robot").first_child(); node; node = node.next_sibling())
    {
      std::string component_name = node.attribute("name").as_string();
      m_component_tree[component_name].reset(new Component(node));
      m_component_tree[component_name]->name = component_name;
      m_component_tree[component_name]->type = node.name();

      if(node.child("parent"))
      {
        parent_of[component_name] = node.child("parent").attribute("link").as_string();
      }
      if(node.child("child"))
      {
        parent_of[node.child("child").attribute("link").as_string()] = component_name;
      }
    }

    for(auto p : parent_of)
    {
      m_component_tree[p.first]->parent = m_component_tree[p.second];
      m_component_tree[p.second]->children.push_back(m_component_tree[p.first]);
    }

    //TODO: remove unconnected nodes (e.g. gazebo references)

    //find roots
    m_roots.clear();
    for(auto n : m_component_tree)
    {
      if(!n.second->parent)
      {
        m_roots.push_back(n.second);
      }
    }

    for(auto r : m_roots)
      r->computeDepth();

    std::sort(m_roots.begin(), m_roots.end(), [](const boost::shared_ptr<Component>& c1, const boost::shared_ptr<Component>& c2) { return (c1->num_supported_components > c2->num_supported_components);});
    std::cerr << "Longest root " << m_roots[0]->name << " had " << m_roots[0]->num_supported_components << " components!" << std::endl;

    return true;
  }

  bool ComponentTree::fromFile(std::string filename)
  {
    std::ifstream ifs(filename.c_str(), std::ios::in | std::ios::binary | std::ios::ate);

    std::ifstream::pos_type fileSize = ifs.tellg();
    ifs.seekg(0, std::ios::beg);

    std::vector<char> bytes(fileSize);
    ifs.read(&bytes[0], fileSize);

    std::string xml = std::string(&bytes[0], fileSize);
    return fromString(xml);
  }
//
//  bool ComponentTree::fromParam(std::string param_name)
//  {
//    if(!ros::param::has(param_name))
//    {
//      ROS_ERROR("Param %s did not exist!", param_name.c_str());
//      return false;
//    }
//
//    std::string xml;
//    ros::param::get(param_name, xml);
//
//    return fromString(xml);
//  }

  boost::shared_ptr<Component> ComponentTree::getRootNode()
  {
    return m_roots[0];
  }

  std::vector<boost::shared_ptr<Component> > ComponentTree::getRootNodes()
  {
    return m_roots;
  }

  std::ostream& operator<<(std::ostream& stream, ComponentTree& tree)
  {
    for(auto r : tree.getRootNodes())
      stream << *r;
    return stream;
  }
}

#endif //COMPONENT_TREE_IMPL_HPP
