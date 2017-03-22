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
  DynamicParameter(const uint16_t major_version, const uint16_t minor_version, XmlRpc::XmlRpcValue& parameters);

  struct Settings
  {
    Settings()
    : position_settings(driver_svh::eSVH_DIMENSION)
    , position_settings_given(driver_svh::eSVH_DIMENSION, false)
    , current_settings(driver_svh::eSVH_DIMENSION)
    , current_settings_given(driver_svh::eSVH_DIMENSION, false)
    , home_settings(driver_svh::eSVH_DIMENSION)
    , home_settings_given(driver_svh::eSVH_DIMENSION, false)
    , major_version(0)
    , minor_version(0)
    {}

    std::vector<std::vector<float> > position_settings;
    std::vector<bool> position_settings_given;

    std::vector<std::vector<float> > current_settings;
    std::vector<bool> current_settings_given;

    std::vector<std::vector<float> > home_settings;
    std::vector<bool> home_settings_given;

    uint16_t major_version;
    uint16_t minor_version;
  };

  const Settings& getSettings() const {return m_settings;}

private:
  void read_file(const uint16_t major_version, const uint16_t minor_version, XmlRpc::XmlRpcValue& parameters);
  bool xml_rpc_value_to_vector(XmlRpc::XmlRpcValue my_array, std::vector<float>& my_vector);

  Settings m_settings;

  // Stores an enum-string matching map
  std::map<driver_svh::SVHChannel, std::string> m_name_to_enum;
};

#endif // #ifdef SCHUNK_SVH_DRIVER_DYNAMIC_PARAMETER_CLASS_H_INCLUDED
