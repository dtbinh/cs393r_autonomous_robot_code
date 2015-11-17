#include "live_params/live_params.hpp"

std::string string_param;

void callback(std::string& stringy)
{
  string_param = stringy;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "live_param_test");
  ros::NodeHandle nh("~");

  boost::shared_ptr<rpp::ParameterManager> pm = rpp::ParameterManager::getInstance();

  rpp::Parameter<double> double_helper;
  rpp::Parameter<int> int_helper;
  rpp::Parameter<std::string> string_helper;

  pm->param("double_param", 0.0, double_helper);
  pm->param("int_param", 0, int_helper);
  pm->param<std::string>("string_param", "zero", string_helper);

  int int_param;

  string_helper.subscribe(&callback);
  int_helper.attach(&int_param);


  while(ros::ok())
  {
    std::cerr << "D: " << double_helper << ", I: " << int_param << ", S: " << string_param << std::endl;
  }
}
