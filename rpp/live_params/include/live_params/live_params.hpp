#ifndef LIVE_PARAMS_HPP
#define LIVE_PARAMS_HPP

#include <ros/ros.h>
#include <ros/xmlrpc_manager.h>

#include <boost/bind.hpp>
#include <boost/function.hpp>
#include <boost/thread.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/type_traits.hpp>

namespace rpp
{
  template<typename T>
  class Parameter
  {
  public:
    Parameter()
    {
      m_configured = false;
    }

    void updateParameter(std::string parameter_string)
    {
      m_cached_param = boost::lexical_cast<T>(parameter_string);
      ROS_INFO_STREAM("Param " << m_param_name<< " set to " << m_cached_param);

      if(m_callback)
      {
        m_callback(m_cached_param);
      }

      if(m_param_ptr)
      {
        if(m_mutex)
        {
          m_mutex->lock();
        }

        *m_param_ptr = m_cached_param;

        if(m_mutex)
        {
          m_mutex->unlock();
        }
      }
    }

    void configure(std::string param_name, T default_value)
    {
      m_param_name = param_name;
      m_param_path = ros::names::resolve(param_name);
      m_default_value = default_value;
      m_param_ptr = NULL;
      m_mutex = NULL;

      if(!ros::param::has(m_param_path))
      {
        //param hasn't been set! set it to the default so the master at least knows about it
        ros::param::set(m_param_path, m_default_value);
        m_cached_param = m_default_value;
      }
      else
      {
        ros::param::get(m_param_path, m_cached_param);
      }

      XmlRpc::XmlRpcValue params, result, payload;
      params[0] = ros::this_node::getName();
      params[1] = ros::XMLRPCManager::instance()->getServerURI();
      params[2] = m_param_path;

      if(!ros::master::execute("subscribeParam", params, result, payload, false))
      {
        //subscription failed, possibly because the master is dead?
        while(!ros::master::execute("subscribeParam", params, result, payload, false))
        {
          ROS_WARN_THROTTLE(1.0, "%s: Waiting for parameter %s to become available...", ros::this_node::getNamespace().c_str(), m_param_path.c_str());
        }
      }

      m_configured = true;
    }

    //callback called whenever param_name is updated
    bool subscribe(boost::function<void(T&)> callback)
    {
      if(!m_configured)
      {
        return false;
      }
      m_callback = callback;
      m_callback(m_cached_param);
      return true;
    }

    //param_ptr updated whenever param_name is updated, locking mutex for safety if provided
    bool attach(T* param_ptr, boost::mutex* mutex = NULL)
    {
      if(!m_configured)
      {
        return false;
      }
      m_param_ptr = param_ptr;
      m_mutex = mutex;

      *param_ptr = m_cached_param;

      return true;
    }

    //gets the current value of the parameter, cached if possible
    bool getParam(T& param)
    {
      if(!m_configured)
      {
        return false;
      }

      param = m_cached_param;
      return true;
    }

    //sets the current value of the parameter
    bool setParam(T param)
    {
      if(!m_configured)
      {
        return false;
      }
      m_cached_param = param;
      ros::param::set(m_param_path, param);
      return true;
    }

    T& operator()()
    {
      return m_cached_param;
    }

    friend std::ostream& operator<<(std::ostream& out, Parameter<T>& param)
    {
      out << param();
      return out;
    }

  private:
    std::string m_param_name;
    std::string m_param_path;
    bool m_configured;
    T m_default_value;
    T m_cached_param;

    boost::function<void(T&)> m_callback;

    T* m_param_ptr;
    boost::mutex* m_mutex;
  };

  class ParameterManager
  {
  public:
    static boost::shared_ptr<ParameterManager> getInstance()
    {
      static boost::shared_ptr<ParameterManager> singleton_ptr;
      if(!singleton_ptr)
      {
        singleton_ptr.reset(new ParameterManager());
      }
      return singleton_ptr;
    }

    std::map<std::string, std::vector<boost::function<void(std::string)> > > m_parameter_updates;

    template<typename T>
    void param(std::string param_name, T default_value, Parameter<T>& param)
    {
      param.configure(param_name, default_value);
      m_parameter_updates[ros::names::clean(ros::names::resolve(param_name))].push_back(boost::bind(&Parameter<T>::updateParameter, &param, _1));
    }

  private:
    ParameterManager()
    {
      ros::XMLRPCManager::instance()->unbind("paramUpdate");
      ros::XMLRPCManager::instance()->bind("paramUpdate", boost::bind(&ParameterManager::paramUpdateCallback, this, _1, _2));
    }

    void paramUpdateCallback(XmlRpc::XmlRpcValue& params, XmlRpc::XmlRpcValue& result)
    {
      result[0] = 1;
      result[1] = std::string("");
      result[2] = 0;

      std::string clean_key = ros::names::clean((std::string) params[1]);
      std::stringstream silly_stream;
      params[2].write(silly_stream);
      if(m_parameter_updates.find(clean_key) != m_parameter_updates.end())
      {
        std::vector<boost::function<void(std::string)> > callbacks = m_parameter_updates.find(clean_key)->second;
        for(unsigned int i = 0; i < callbacks.size(); i++)
        {
          callbacks.at(i)(silly_stream.str());
        }
//        boost::function<void(std::string)> fun = m_parameter_updates.find(clean_key)->second;
//        fun(silly_stream.str());
      }
    }
  };
}

#endif //LIVE_PARAMS_HPP
