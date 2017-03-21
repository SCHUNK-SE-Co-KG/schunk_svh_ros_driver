#include <DynamicParameterClass.h>

/*
 * The DynamicParameter class reads out the parameter file and searches for the proposed version!
 */

DynamicParameter::DynamicParameter(int16_t major_version, int16_t minor_version, XmlRpc::XmlRpcValue parameters):
  m_position_settings(driver_svh::eSVH_DIMENSION),
  m_position_settings_given(driver_svh::eSVH_DIMENSION,false),
  m_current_settings(driver_svh::eSVH_DIMENSION),
  m_current_settings_given(driver_svh::eSVH_DIMENSION,false),
  m_home_settings(driver_svh::eSVH_DIMENSION),
  m_home_settings_given(driver_svh::eSVH_DIMENSION,false)
 {
 
  
  
  //joint names used for parameter mapping from string to SVHChannel
  //currently hardcoded...
  m_joint_names.push_back("THUMB_FLEXION");
  m_joint_names.push_back("THUMB_OPPOSITION");
  m_joint_names.push_back("INDEX_FINGER_DISTAL");
  m_joint_names.push_back("INDEX_FINGER_PROXIMAL");
  m_joint_names.push_back("MIDDLE_FINGER_DISTAL");
  m_joint_names.push_back("MIDDLE_FINGER_PROXIMAL");
  m_joint_names.push_back("RING_FINGER");
  m_joint_names.push_back("PINKY");
  m_joint_names.push_back("FINGER_SPREAD");

  ROS_ASSERT(static_cast<driver_svh::SVHChannel>( m_joint_names.size() ) == driver_svh::eSVH_DIMENSION);

  //mapping from name to enum
  for (size_t i = driver_svh::eSVH_THUMB_FLEXION; i < driver_svh::eSVH_DIMENSION; ++i)
  {

    driver_svh::SVHChannel channel = static_cast<driver_svh::SVHChannel>(i);
    m_name_to_enum[ m_joint_names[i] ] = channel;
  
  }

  int result = read_file(major_version,minor_version,parameters);

  if (result  == -1)
    ROS_ERROR("ATTENTION: YOU HAVE AN INCORRECT PARAMETER FILE!!!");
    exit(0);
  }else if( result == 0){
  else if(result == 0)
    ROS_ERROR("DID NOT FIND THE CORRECT PARAMETER FILE FOR THE PROPOSED VERSION! FALLBACK ON THE DEFAULT PARAMETERS");
  }

}

/*
 *  This function converts a RpcValue Vector to a std::vector
 */


bool DynamicParameter::xml_rpc_value_to_vector(XmlRpc::XmlRpcValue my_array,std::vector<float>& my_vector){
    my_vector.clear();

    for (size_t i = 0; i < my_array.size(); ++i)
    {

        ROS_ASSERT(my_array[i].getType() == XmlRpc::XmlRpcValue::TypeDouble || my_array[i].getType() == XmlRpc::XmlRpcValue::TypeInt);

	if(my_array[i].getType() == XmlRpc::XmlRpcValue::TypeDouble)
        {

	  my_vector.push_back(static_cast<double>(my_array[i]));
        
	}
        else if(my_array[i].getType() == XmlRpc::XmlRpcValue::TypeInt)
        {

            int value = my_array[i];
            my_vector.push_back(static_cast<double>(value));

        }
    }

    return true;
}


/*
 *   This function reads a yaml parameter file!
 *   RETURN-CODES: -1 -> parsing error
 *                 0 -> default fallback
 *                 1 -> found correct parameters      
 */

int DynamicParameter::read_file(int16_t major_version_target,int16_t minor_version_target, XmlRpc::XmlRpcValue parameters){

  bool default_state = false;
  bool correct_version = false;
  bool same_major_version = false;
  ROS_INFO("Trying to search Version: %d.%d", major_version_target,minor_version_target);
  
  int major_version_last_read;
  int minor_version_last_read;
  try
  {
      if(parameters.size() > 0)
      {
	
        ROS_DEBUG("There exist %d several versions",parameters.size());
        for (size_t i = 0; i < parameters.size(); ++i)
        {

             XmlRpc::XmlRpcValue parameter_set_yaml = parameters[i]["parameter_set"];
	     
	     //the following parameters are for this version
             int major_version_read = parameter_set_yaml["major_version"];
             int minor_version_read = parameter_set_yaml["minor_version"];
	     
	     if(major_version_read == major_version_target && minor_version_read == minor_version_target)
	       correct_version = true;
	     if(major_version_read == major_version_target && minor_version_read < minor_version_target)
	       same_major_version = true;
	     if(major_version_read == 0 && minor_version_read == 0)
	       default_state = true;

	     //if the major_version fits to the target_version we succeed, first parameter set should be always the default set!!!
	     if(correct_version || same_major_version || default_state)
             {
	       major_version_last_read = major_version_read;
	       minor_version_last_read = minor_version_read;

	       ROS_DEBUG("major version: %d minor version: %d",major_version_read,minor_version_read);

	       XmlRpc::XmlRpcValue parameters = parameter_set_yaml["PARAMETERS"];
	       for(size_t j = 0; j < parameters.size(); ++j)
               {
                   XmlRpc::XmlRpcValue parameter_yaml = parameters[j]["parameter"];

                   std::string parameter_name = parameter_yaml["name"];
                   ROS_DEBUG("Parameter Name: %s",parameter_name.c_str());

                   m_position_settings_given[m_name_to_enum[parameter_name]] = xml_rpc_value_to_vector(parameter_yaml["position_controller"],m_position_settings[m_name_to_enum[parameter_name]]);
		   m_current_settings_given[m_name_to_enum[parameter_name]]  = xml_rpc_value_to_vector(parameter_yaml["current_controller"] ,m_current_settings[m_name_to_enum[parameter_name]]);
		   m_home_settings_given[m_name_to_enum[parameter_name]]     = xml_rpc_value_to_vector(parameter_yaml["home_settings"]      ,m_home_settings[m_name_to_enum[parameter_name]]);
               }
	       //First Reading of parameters
               if(default_state == true)
	       {
		 default_state = false;
	       }else if(correct_version)
               {
		 ROS_INFO("DID FIND CORRECT VERSION");
		 return 1;
	       }
               else
	       {
                 ROS_INFO("DID FIND SAME MAJOR VERSION");
		 same_major_version = false;
	       }
	     }
	  }
      }
      ROS_ERROR_STREAM("parsing error: " << e.getMessage() << "! Error code: " << e.getCode());
      ROS_ERROR_STREAM("parsing error: " << e.getMessage() << "! Error code: " << e.getCode());
      return -1;
  }
  ROS_INFO_STREAM("DID NOT FIND CORRECT VERSION: " << major_version_target << "." << minor_version_target << "switched to: " << major_version_last_read << "." <<  minor_version_last_read);
  return 0;
}
