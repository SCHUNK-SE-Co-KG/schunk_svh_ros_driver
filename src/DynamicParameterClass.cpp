// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!\file
 *
 * \author  Andreas Konle <konle@fzi.de>
 * \author  Felix Mauch <mauch@fzi.de>
 * \date    2017-2-22
 *
 */
//----------------------------------------------------------------------



#include <DynamicParameterClass.h>

/*
 * The DynamicParameter class reads out the parameter file and searches for the proposed version!
 */

DynamicParameter::DynamicParameter(const uint16_t major_version,
                                   const uint16_t minor_version,
                                   XmlRpc::XmlRpcValue& parameters)
{
  // joint names used for parameter mapping from string to SVHChannel
  // currently hardcoded...
  m_name_to_enum[driver_svh::eSVH_THUMB_FLEXION]          = "THUMB_FLEXION";
  m_name_to_enum[driver_svh::eSVH_THUMB_OPPOSITION]       = "THUMB_OPPOSITION";
  m_name_to_enum[driver_svh::eSVH_INDEX_FINGER_DISTAL]    = "INDEX_FINGER_DISTAL";
  m_name_to_enum[driver_svh::eSVH_INDEX_FINGER_PROXIMAL]  = "INDEX_FINGER_PROXIMAL";
  m_name_to_enum[driver_svh::eSVH_MIDDLE_FINGER_DISTAL]   = "MIDDLE_FINGER_DISTAL";
  m_name_to_enum[driver_svh::eSVH_MIDDLE_FINGER_PROXIMAL] = "MIDDLE_FINGER_PROXIMAL";
  m_name_to_enum[driver_svh::eSVH_RING_FINGER]            = "RING_FINGER";
  m_name_to_enum[driver_svh::eSVH_PINKY]                  = "PINKY";
  m_name_to_enum[driver_svh::eSVH_FINGER_SPREAD]          = "FINGER_SPREAD";

  ROS_ASSERT(static_cast<driver_svh::SVHChannel>(m_name_to_enum.size()) ==
             driver_svh::eSVH_DIMENSION);

  try
  {
    read_file(major_version, minor_version, parameters);
  }
  catch (XmlRpc::XmlRpcException& e)
  {
    ROS_ERROR_STREAM("parsing error: " << e.getMessage() << "! Error code: " << e.getCode());
    ROS_ERROR("ATTENTION: YOU HAVE AN INCORRECT PARAMETER FILE!!!");
    exit(0);
  }

//   else if (result == 0)
//   {
//     ROS_ERROR("DID NOT FIND THE CORRECT PARAMETER FILE FOR THE PROPOSED VERSION! "
//               "FALLBACK ON THE DEFAULT PARAMETERS");
//   }
}

/*
 *  This function converts a RpcValue Vector to a std::vector
 */


bool DynamicParameter::xml_rpc_value_to_vector(XmlRpc::XmlRpcValue my_array,
                                               std::vector<float>& my_vector)
{
  my_vector.clear();

  for (size_t i = 0; i < my_array.size(); ++i)
  {
    ROS_ASSERT(my_array[i].getType() == XmlRpc::XmlRpcValue::TypeDouble ||
               my_array[i].getType() == XmlRpc::XmlRpcValue::TypeInt);

    if (my_array[i].getType() == XmlRpc::XmlRpcValue::TypeDouble)
    {
      my_vector.push_back(static_cast<double>(my_array[i]));
    }
    else if (my_array[i].getType() == XmlRpc::XmlRpcValue::TypeInt)
    {
      int value = my_array[i];
      my_vector.push_back(static_cast<double>(value));
    }
  }

  return true;
}


void DynamicParameter::read_file(const uint16_t major_version_target,
                                const uint16_t minor_version_target,
                                XmlRpc::XmlRpcValue& parameters)
{
  ROS_INFO("Trying to search controller parameters for version: %d.%d", major_version_target, minor_version_target);

  m_settings.major_version = 0;
  m_settings.minor_version = 0;

  if (parameters.size() > 0)
  {
    ROS_DEBUG("There exist %d different parameter versions", parameters.size());
    for (size_t i = 0; i < parameters.size(); ++i)
    {
      XmlRpc::XmlRpcValue parameter_set_yaml = parameters[i]["parameter_set"];

      // the following parameters are for this version
      uint16_t major_version_read = int(parameter_set_yaml["major_version"]);
      uint16_t minor_version_read = int(parameter_set_yaml["minor_version"]);
      if (m_settings.major_version > major_version_read)
      {
        ROS_DEBUG("Skipping version %d.%d as better matching version was already found",
                  major_version_read,
                  minor_version_read
        );
        continue;
      }

      bool correct_version =
        major_version_read == major_version_target && minor_version_read == minor_version_target;
      bool same_major_version = major_version_read == major_version_target && minor_version_read <= minor_version_target;
      bool default_state      = major_version_read == 0 && minor_version_read == 0;

      if (correct_version || same_major_version || default_state)
      {
        ROS_DEBUG("major version: %d minor version: %d", major_version_read, minor_version_read);

        for (std::map<driver_svh::SVHChannel, std::string>::iterator it = m_name_to_enum.begin();
              it != m_name_to_enum.end();
              ++it)
        {
          m_settings.major_version = major_version_read;
          m_settings.minor_version = minor_version_read;
          if (parameter_set_yaml.hasMember(it->second))
          {
            ROS_DEBUG("Parameter Name: %s", it->second.c_str());

            m_settings.position_settings_given[it->first] = xml_rpc_value_to_vector(
              parameter_set_yaml[it->second]["position_controller"], m_settings.position_settings[it->first]);
            m_settings.current_settings_given[it->first] = xml_rpc_value_to_vector(
              parameter_set_yaml[it->second]["current_controller"], m_settings.current_settings[it->first]);
            m_settings.home_settings_given[it->first] = xml_rpc_value_to_vector(
              parameter_set_yaml[it->second]["home_settings"], m_settings.home_settings[it->first]);
          }
          else
          {
            std::stringstream error_stream;
            error_stream << "Could not find parameters for channel " << it->second;
            throw(XmlRpc::XmlRpcException(error_stream.str(), -1));
          }
        }

        // First Reading of parameters
        if (default_state == true)
        {
          default_state = false;
        }
        else if (correct_version)
        {
          ROS_INFO("DID FIND CORRECT VERSION");
          return;
        }
        else
        {
          ROS_DEBUG("DID FIND SAME MAJOR VERSION");
        }
      }
    }
  }
  ROS_INFO_STREAM("DID NOT FIND EXACT VERSION: " << major_version_target << "."
                                                 << minor_version_target
                                                 << " Falling back to: "
                                                 << m_settings.major_version
                                                 << "."
                                                 << m_settings.minor_version);
}
