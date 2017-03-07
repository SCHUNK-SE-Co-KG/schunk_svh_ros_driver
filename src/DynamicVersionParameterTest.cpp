#include <ros/ros.h>
#include <iostream>
#include <XmlRpcException.h>

#include <driver_svh/SVHFingerManager.h>
#include <driver_svh/SVHPositionSettings.h>
#include <driver_svh/SVHCurrentSettings.h>


bool XmlRpcValueToVector(XmlRpc::XmlRpcValue my_array,std::vector<float>& my_vector){
    for (int32_t i = 0; i < my_array.size(); ++i){

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

  std::map< std::string, driver_svh::SVHChannel> name_to_enum;

  std::vector<std::string> joint_names;
  			      

  joint_names.push_back("THUMB_FLEXION");
  joint_names.push_back("THUMB_OPPOSITION");
  joint_names.push_back("INDEX_FINGER_DISTAL");
  joint_names.push_back("INDEX_FINGER_PROXIMAL");
  joint_names.push_back("MIDDLE_FINGER_DISTAL");
  joint_names.push_back("MIDDLE_FINGER_PROXIMAL");
  joint_names.push_back("RING_FINGER");
  joint_names.push_back("PINKY");
  joint_names.push_back("FINGER_SPREAD");

  ROS_INFO("jointes_names:%d",sizeof(joint_names));  


  ROS_ASSERT( static_cast<driver_svh::SVHChannel>( joint_names.size() ) == driver_svh::eSVH_DIMENSION);

  for (int32_t i = driver_svh::eSVH_THUMB_FLEXION; i < driver_svh::eSVH_DIMENSION; ++i){

    driver_svh::SVHChannel channel = static_cast<driver_svh::SVHChannel>(i);
    name_to_enum[ joint_names[i] ] = channel;
  
  }

  XmlRpc::XmlRpcValue version_control;
  private_node.getParam("VERSIONS_PARAMETERS", version_control);
  try{
      if(version_control.size() > 0){

        ROS_INFO("There exist %d several versions",version_control.size());
        for (int32_t i = 0; i < version_control.size(); ++i){

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

                 postion_settings_given[name_to_enum[parameter_name]] = XmlRpcValueToVector(parameter_yaml["position_controller"],position_settings[name_to_enum[parameter_name]]);
                 current_settings_given[name_to_enum[parameter_name]] = XmlRpcValueToVector(parameter_yaml["current_controller"] ,current_settings[name_to_enum[parameter_name]]);
                 home_settings_given[name_to_enum[parameter_name]]    = XmlRpcValueToVector(parameter_yaml["home_settings"]      ,home_settings[name_to_enum[parameter_name]]);

                 /*
                 for(int32_t z = 0; z < current_settings[name_to_enum[parameter_name]].size();z++ )
                    std::cout << current_settings[name_to_enum[parameter_name]][z] << std::endl;

                 for(int32_t z = 0; z < position_settings[name_to_enum[parameter_name]].size();z++ )
                    std::cout << position_settings[name_to_enum[parameter_name]][z] << std::endl;
                 */

             }
        }
      }
  }catch(XmlRpc::XmlRpcException& e){
      ROS_ERROR_STREAM("parsing error: " << e.getMessage() << "! Error code: " << e.getCode());

  }
  return 0;
}
