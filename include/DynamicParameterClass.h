#include <ros/ros.h>
#include <iostream>
#include <XmlRpcException.h>

#include <driver_svh/SVHFingerManager.h>
#include <driver_svh/SVHPositionSettings.h>
#include <driver_svh/SVHCurrentSettings.h>

class DynamicParameter{

 public:

  DynamicParameter(int16_t major_version,int16_t minor_version, XmlRpc::XmlRpcValue parameters);

  // PARAMETERS
  std::vector< std::vector<float> > m_position_settings;
  std::vector<bool>                 m_position_settings_given;

  std::vector< std::vector<float> > m_current_settings;
  std::vector<bool>                 m_current_settings_given;

  std::vector< std::vector<float> > m_home_settings;
  std::vector<bool>                 m_home_settings_given;

 private:

  int  read_file(int16_t major_version,int16_t minor_version, XmlRpc::XmlRpcValue parameters );
  bool xml_rpc_value_to_vector(XmlRpc::XmlRpcValue my_array,std::vector<float>& my_vector);

  std::vector<std::string> m_joint_names;
  std::map< std::string, driver_svh::SVHChannel> m_name_to_enum;


};
