#include <ros/ros.h>
#include <iostream>
#include <XmlRpcException.h>

#include <driver_svh/SVHFingerManager.h>
#include <driver_svh/SVHPositionSettings.h>
#include <driver_svh/SVHCurrentSettings.h>


bool XmlRpcValueToVector(XmlRpc::XmlRpcValue my_array,std::vector<float>& my_vector){
    for (int32_t i = 0; i < my_array.size(); ++i)
    {
        ROS_ASSERT(my_array[i].getType() == XmlRpc::XmlRpcValue::TypeDouble || my_array[i].getType() == XmlRpc::XmlRpcValue::TypeInt);
        if(my_array[i].getType() == XmlRpc::XmlRpcValue::TypeDouble){
            my_vector.push_back(static_cast<double>(my_array[i]));
        }else if(my_array[i].getType() == XmlRpc::XmlRpcValue::TypeInt){
            int value = my_array[i];
            my_vector.push_back(static_cast<double>(value));
        }
    }
    return true;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "DynamicParameterTest");
  ros::NodeHandle private_node("~");



  ROS_INFO("Welcome to the Dynamic Version Parameter Controller Test");

  // Rosparam can only fill plain vectors so we will have to go through them
  std::vector< std::vector<float> > position_settings(driver_svh::eSVH_DIMENSION);
  std::vector<bool> postion_settings_given(driver_svh::eSVH_DIMENSION,false);

  std::vector< std::vector<float> > current_settings(driver_svh::eSVH_DIMENSION);
  std::vector<bool> current_settings_given(driver_svh::eSVH_DIMENSION,false);

  std::vector< std::vector<float> > home_settings(driver_svh::eSVH_DIMENSION);
  std::vector<bool> home_settings_given(driver_svh::eSVH_DIMENSION,false);

  std::map< std::string, driver_svh::SVHChannel> name_to_enum_mapping;

  name_to_enum_mapping["THUMB_FLEXION"]          = driver_svh::eSVH_THUMB_FLEXION;
  name_to_enum_mapping["THUMB_OPPOSITION"]       = driver_svh::eSVH_THUMB_OPPOSITION;
  name_to_enum_mapping["INDEX_FINGER_DISTAL"]    = driver_svh::eSVH_INDEX_FINGER_DISTAL;
  name_to_enum_mapping["INDEX_FINGER_PROXIMAL"]  = driver_svh::eSVH_INDEX_FINGER_PROXIMAL;
  name_to_enum_mapping["MIDDLE_FINGER_DISTAL"]   = driver_svh::eSVH_MIDDLE_FINGER_DISTAL;
  name_to_enum_mapping["MIDDLE_FINGER_PROXIMAL"] = driver_svh::eSVH_MIDDLE_FINGER_PROXIMAL;
  name_to_enum_mapping["RING_FINGER"]            = driver_svh::eSVH_RING_FINGER;
  name_to_enum_mapping["PINKY"]                  = driver_svh::eSVH_PINKY;
  name_to_enum_mapping["FINGER_SPREAD"]          = driver_svh::eSVH_FINGER_SPREAD;

  XmlRpc::XmlRpcValue version_control;
  private_node.getParam("VERSIONS_PARAMETERS", version_control);
  try{
      if(version_control.size() > 0)
      {
        ROS_INFO("There exist %d several versions",version_control.size());
        for (int32_t i = 0; i < version_control.size(); ++i)
        {
             XmlRpc::XmlRpcValue parameter_set_yaml = version_control[i]["parameter_set"];

             int major_version = parameter_set_yaml["major_version"];
             int minor_version = parameter_set_yaml["minor_version"];

             //TODO: MAKE TO CLASS and compare with version number

             ROS_INFO("major version: %d minor version: %d",major_version,minor_version);

             XmlRpc::XmlRpcValue parameters = parameter_set_yaml["PARAMETERS"];
             for(int32_t j = 0; j < parameters.size(); ++j){
                 XmlRpc::XmlRpcValue parameter_yaml = parameters[j]["parameter"];

                 std::string parameter_name = parameter_yaml["name"];
                 ROS_INFO("Parameter Name: %s",parameter_name.c_str());

                 postion_settings_given[name_to_enum_mapping[parameter_name]] = XmlRpcValueToVector(parameter_yaml["position_controller"],position_settings[name_to_enum_mapping[parameter_name]]);
                 current_settings_given[name_to_enum_mapping[parameter_name]] = XmlRpcValueToVector(parameter_yaml["current_controller"] ,current_settings[name_to_enum_mapping[parameter_name]]);
                 home_settings_given[name_to_enum_mapping[parameter_name]]    = XmlRpcValueToVector(parameter_yaml["home_settings"]      ,home_settings[name_to_enum_mapping[parameter_name]]);

                 /*
                 for(int32_t z = 0; z < current_settings[name_to_enum_mapping[parameter_name]].size();z++ )
                    std::cout << current_settings[name_to_enum_mapping[parameter_name]][z] << std::endl;

                 for(int32_t z = 0; z < position_settings[name_to_enum_mapping[parameter_name]].size();z++ )
                    std::cout << position_settings[name_to_enum_mapping[parameter_name]][z] << std::endl;
                 */

             }

        }
      }
  }catch(XmlRpc::XmlRpcException& e){
      ROS_ERROR_STREAM("parsing error: " << e.getMessage() << "! Error code: " << e.getCode());

  }
  return 0;
}
