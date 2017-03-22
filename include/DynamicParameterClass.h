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

#ifndef SCHUNK_SVH_DRIVER_DYNAMIC_PARAMETER_CLASS_H_INCLUDED
#define SCHUNK_SVH_DRIVER_DYNAMIC_PARAMETER_CLASS_H_INCLUDED

#include <XmlRpcException.h>
#include <iostream>
#include <ros/ros.h>

#include <driver_svh/SVHCurrentSettings.h>
#include <driver_svh/SVHFingerManager.h>
#include <driver_svh/SVHPositionSettings.h>

class DynamicParameter
{
public:
  DynamicParameter(int16_t major_version, int16_t minor_version, XmlRpc::XmlRpcValue parameters);

  // PARAMETERS
  std::vector<std::vector<float> > m_position_settings;
  std::vector<bool> m_position_settings_given;

  std::vector<std::vector<float> > m_current_settings;
  std::vector<bool> m_current_settings_given;

  std::vector<std::vector<float> > m_home_settings;
  std::vector<bool> m_home_settings_given;

private:
  int read_file(int16_t major_version, int16_t minor_version, XmlRpc::XmlRpcValue parameters);
  bool xml_rpc_value_to_vector(XmlRpc::XmlRpcValue my_array, std::vector<float>& my_vector);

  std::vector<std::string> m_joint_names;
  std::map<std::string, driver_svh::SVHChannel> m_name_to_enum;
};

#endif // #ifdef SCHUNK_SVH_DRIVER_DYNAMIC_PARAMETER_CLASS_H_INCLUDED
