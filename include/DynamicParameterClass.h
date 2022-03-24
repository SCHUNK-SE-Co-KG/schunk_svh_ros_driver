// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// This file is part of the SCHUNK SVH Driver suite.
//
// This program is free software licensed under the LGPL
// (GNU LESSER GENERAL PUBLIC LICENSE Version 3).
// You can find a copy of this license in LICENSE folder in the top
// directory of the source code.
//
// © Copyright 2017 SCHUNK Mobile Greifsysteme GmbH, Lauffen/Neckar Germany
// © Copyright 2017 FZI Forschungszentrum Informatik, Karlsruhe, Germany
//
// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!\file
 *
 * \author  Andreas Konle <konle@fzi.de>
 * \author  Felix Mauch <mauch@fzi.de>
 * \date    2017-2-22
 *
 * This file contains the DynamicParameter for automatically setting the correct and
 * best parameters according to the connected Schunk SVH hardware.
 */
//----------------------------------------------------------------------

#ifndef SCHUNK_SVH_DRIVER_DYNAMIC_PARAMETER_CLASS_H_INCLUDED
#define SCHUNK_SVH_DRIVER_DYNAMIC_PARAMETER_CLASS_H_INCLUDED

#include <XmlRpcException.h>
#include <iostream>
#include <ros/ros.h>

#include <schunk_svh_library/control/SVHCurrentSettings.h>
#include <schunk_svh_library/control/SVHFingerManager.h>
#include <schunk_svh_library/control/SVHPositionSettings.h>

/*!
 * \brief Class to read parameter file and set the correct hardware parameters
 */
class DynamicParameter
{
public:
  /*!
   * \brief DynamicParameter constructs a new dynamic parameter handler
   * \param major_version The major version of the svh hardware
   * \param minor_version The minor version of the svh hardware
   * \param parameters An array of type XmlRpcValue with parsed parameter file
   */
  DynamicParameter( const uint16_t major_version,
                    const uint16_t minor_version,
                    XmlRpc::XmlRpcValue& parameters);


  /*!
   * \brief Inline struct to store the settings
   */
  struct Settings
  {
    Settings()
    : position_settings(driver_svh::SVH_DIMENSION)
    , position_settings_given(driver_svh::SVH_DIMENSION, false)
    , current_settings(driver_svh::SVH_DIMENSION)
    , current_settings_given(driver_svh::SVH_DIMENSION, false)
    , home_settings(driver_svh::SVH_DIMENSION)
    , home_settings_given(driver_svh::SVH_DIMENSION, false)
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
  /*!
   * \brief This method parses the given XmlRpcValue array and sets the corresponding parameters
   * \param major_version The major version of the svh hardware
   * \param minor_version The minor version of the svh hardware
   * \param parameters An array of type XmlRpcValue with parsed parameter file
   */
  void parse_parameters(const uint16_t major_version,
                        const uint16_t minor_version,
                        XmlRpc::XmlRpcValue& parameters);


  /*!
   * \brief Converts the given XmlRpcValue array to a std:vector
   * /param my_array XmlRpcValue array created by reading the parameter file
   * /param my_vector Vector to write the array values to
   */
  bool xml_rpc_value_to_vector(XmlRpc::XmlRpcValue my_array, std::vector<float>& my_vector);

  //! Stores the settings received by the firmware
  Settings m_settings;

  //! Stores an enum-string matching map
  std::map<driver_svh::SVHChannel, std::string> m_name_to_enum;
};

#endif // #ifdef SCHUNK_SVH_DRIVER_DYNAMIC_PARAMETER_CLASS_H_INCLUDED
