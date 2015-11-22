#ifndef COMPONENT_TREE_IMPL_HPP
#define COMPONENT_TREE_IMPL_HPP

namespace rpp
{
  inline ComponentTree::ComponentTree()
  {
  }

  inline ComponentTree::~ComponentTree()
  {
  }

  inline bool ComponentTree::fromString(std::string xml)
  {
    m_doc.load(xml.c_str());

    std::map<std::string, std::string> parent_of;
    for(pugi::xml_node node = m_doc.child("robot").first_child(); node; node = node.next_sibling())
    {
      std::string type = node.name();
      std::string name = node.attribute("name").as_string();
      if(type == "" || name == "")
        continue;

      std::string uid = type+name;
      m_component_tree[uid].reset(new Component(node));
      m_component_tree[uid]->name = name;
      m_component_tree[uid]->type = type;
      m_valid_component_types.insert(type);
      m_type_map[type][name] = m_component_tree[uid];

      if(node.child("parent"))
      {
        parent_of[uid] = "link"+std::string(node.child("parent").attribute("link").as_string());
      }
      if(node.child("reference"))
      {
        parent_of[uid] = "link"+std::string(node.child("reference").attribute("link").as_string());
      }
      if(node.child("child"))
      {
        parent_of["link"+std::string(node.child("child").attribute("link").as_string())] = uid;
      }
    }

    for(auto p : parent_of)
    {
      m_component_tree[p.first]->parent = m_component_tree[p.second];
      m_component_tree[p.second]->children.push_back(m_component_tree[p.first]);
    }

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

    std::sort(m_roots.begin(), m_roots.end(), [](const boost::shared_ptr<Component>& c1, const boost::shared_ptr<Component>& c2)
    { return (c1->num_supported_components > c2->num_supported_components);});
    std::cerr << "Longest root " << m_roots[0]->name << " had " << m_roots[0]->num_supported_components << " components!" << std::endl;

    return true;
  }

  inline bool ComponentTree::fromFile(std::string filename)
  {
    std::ifstream ifs(filename.c_str(), std::ios::in | std::ios::binary | std::ios::ate);

    std::ifstream::pos_type fileSize = ifs.tellg();
    ifs.seekg(0, std::ios::beg);

    std::vector<char> bytes(fileSize);
    ifs.read(&bytes[0], fileSize);

    std::string xml = std::string(&bytes[0], fileSize);
    return fromString(xml);
  }

  inline boost::shared_ptr<Component> ComponentTree::rootNode()
  {
    return m_roots[0];
  }

  inline std::vector<boost::shared_ptr<Component> > ComponentTree::rootNodes()
  {
    return m_roots;
  }

  inline std::set<std::string> ComponentTree::componentTypeList()
  {
    return m_valid_component_types;
  }

  inline std::map<std::string, boost::shared_ptr<Component> > ComponentTree::components(std::string type)
  {
    return m_type_map.at(type);
  }

  inline boost::shared_ptr<Component> ComponentTree::component(std::string type, std::string name)
  {
    return m_type_map.at(type).at(name);
  }

  inline std::ostream& operator<<(std::ostream& stream, ComponentTree& tree)
  {
    for(auto r : tree.rootNodes())
      stream << *r;
    return stream;
  }
}

#endif //COMPONENT_TREE_IMPL_HPP
