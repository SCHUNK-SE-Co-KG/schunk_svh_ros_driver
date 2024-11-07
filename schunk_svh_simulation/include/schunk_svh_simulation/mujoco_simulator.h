// Copyright 2024 SCHUNK SE & Co. KG
//
// This program is free software: you can redistribute it and/or modify it
// under the terms of the GNU General Public License as published by the Free
// Software Foundation, either version 3 of the License, or (at your option)
// any later version.
//
// This program is distributed in the hope that it will be useful, but WITHOUT
// ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
// FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
// more details.
//
// You should have received a copy of the GNU General Public License along with
// this program. If not, see <https://www.gnu.org/licenses/>.
// --------------------------------------------------------------------------------

//-----------------------------------------------------------------------------
/*!\file    mujoco_simulator.h
 *
 * \author  Stefan Scherzinger <stefan.scherzinger@de.schunk.com>
 * \date    2024/10/01
 *
 */
//-----------------------------------------------------------------------------

#pragma once

#include <cstdio>
#include <cstring>
#include <map>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include "GLFW/glfw3.h"
#include "mujoco/mujoco.h"

namespace schunk_svh_simulation
{
/**
 * @brief MuJoCo's physics engine with rendering and basic window mouse interaction
 *
 * It's implemented as a singleton class, which circumvents the problem of
 * using global function pointers for the control callback.
 *
 * User code interfaces this class by getting an instance and calling static
 * functions on it.  It's designed to run with an independent simulation rate,
 * disjoint from ROS2-control in a separate thread.
 *
 */
class MuJoCoSimulator
{
private:
  MuJoCoSimulator();

  // Lock the mutex for these calls
  void syncStates();

public:
  // Modern singleton approach
  MuJoCoSimulator(const MuJoCoSimulator &) = delete;
  MuJoCoSimulator & operator=(const MuJoCoSimulator &) = delete;
  MuJoCoSimulator(MuJoCoSimulator &&) = delete;
  MuJoCoSimulator & operator=(MuJoCoSimulator &&) = delete;

  // Use this in ROS2 code
  static MuJoCoSimulator & getInstance()
  {
    static MuJoCoSimulator simulator;
    return simulator;
  }

  // MuJoCo data structures
  mjModel * m = NULL;  // MuJoCo model
  mjData * d = NULL;   // MuJoCo data
  mjvCamera cam;       // abstract camera
  mjvOption opt;       // visualization options
  mjvScene scn;        // abstract scene
  mjrContext con;      // custom GPU context

  // mouse interaction
  bool button_left = false;
  bool button_middle = false;
  bool button_right = false;
  double lastx = 0;
  double lasty = 0;

  // Buffers for data exchange with ROS2-control
  const int svh_dof = 9;
  std::vector<double> pos_cmd;
  std::vector<double> pos_state;
  std::vector<double> vel_state;
  std::vector<double> eff_state;
  std::vector<double> curr_state;

  // Joint order and transmissions in the MuJoCo model:
  // --------------------------------------------------
  //  0  : Thumb_Opposition
  //  1  : Thumb_Flexion
  //  2  : j3  = 1.01511 * Thumb_Flexion
  //  3  : j4  = 1.44889 * Thumb_Flexion
  //  4  : Index_spread  = 0.5 * Finger_Spread
  //  5  : Index_Finger_Proximal
  //  6  : Index_Finger_Distal
  //  7  : j14  = 1.0450 * Index_Finger_Distal
  //  8  : Middle_Finger_Proximal
  //  9  : Middle_Finger_Distal
  //  10 : j15  = 1.0454 * Middle_Finger_Distal
  //  11 : Finger_Spread
  //  12 : Pinky
  //  13 : j13  = 1.35880 * Pinky
  //  14 : j17  = 1.42307 * Pinky
  //  15 : Ring_spread  = 0.5 * Finger_Spread
  //  16 : Ring_Finger
  //  17 : j12  = 1.3588 * Ring_Finger
  //  18 : j16  = 1.42093 * Ring_Finger

  // Joint order in the SVH model:
  // -----------------------------
  // 0  : Thumb_Flexion
  // 1  : Thumb_Opposition
  // 2  : Index_Finger_Distal
  // 3  : Index_Finger_Proximal
  // 4  : Middle_Finger_Distal
  // 5  : Middle_Finger_Proximal
  // 6  : Ring_Finger
  // 7  : Pinky
  // 8  : Finger_Spread

  std::map<int, int> mujoco_idx = {
    // clang-format off
    {0, 1},
    {1, 0},
    {2, 6},
    {3, 5},
    {4, 9},
    {5, 8},
    {6, 16},
    {7, 12},
    {8, 11},
    // clang-format on
  };

  std::map<int, std::pair<double, int>> transmissions = {
    // clang-format off
    // mujoco idx, transmission, ros2 joint idx.
    {0, {1.0, 1}},
    {1, {1.0, 0}},
    {2, {1.01511, 0}},
    {3, {1.44889, 0}},
    {4, {0.5, 8}},
    {5, {1.0, 3}},
    {6, {1.0, 2}},
    {7, {1.0450, 2}},
    {8, {1.0, 5}},
    {9, {1.0, 4}},
    {10, {1.0454, 4}},
    {11, {1.0, 8}},
    {12, {1.0, 7}},
    {13, {1.35880, 7}},
    {14, {1.42307, 7}},
    {15, {0.5, 8}},
    {16, {1.0, 6}},
    {17, {1.3588, 6}},
    {18, {1.42093, 6}},
    // clang-format on
  };

  // Safety guards for buffers
  std::mutex state_mutex;
  std::mutex command_mutex;

  // Keyboard callback
  static void keyboardCB(GLFWwindow * window, int key, int scancode, int act, int mods);
  void keyboardCBImpl(GLFWwindow * window, int key, int scancode, int act, int mods);

  // Mouse button callback
  static void mouseButtonCB(GLFWwindow * window, int button, int act, int mods);
  void mouseButtonCBImpl(GLFWwindow * window, int button, int act, int mods);

  // Mouse move callback
  static void mouseMoveCB(GLFWwindow * window, double xpos, double ypos);
  void mouseMoveCBImpl(GLFWwindow * window, double xpos, double ypos);

  // Scroll callback
  static void scrollCB(GLFWwindow * window, double xoffset, double yoffset);
  void scrollCBImpl(GLFWwindow * window, double xoffset, double yoffset);

  // Control input callback for the solver
  static void controlCB(const mjModel * m, mjData * d);
  void controlCBImpl(const mjModel * m, mjData * d);

  // Call this in a separate thread
  static int simulate(const std::string & model_xml, const std::string & mesh_dir);
  int simulateImpl(const std::string & model_xml, const std::string & mesh_dir);

  // Non-blocking
  void read(
    std::vector<double> & pos, std::vector<double> & vel, std::vector<double> & eff,
    std::vector<double> & curr);
  void write(const std::vector<double> & pos);
};

}  // namespace schunk_svh_simulation
