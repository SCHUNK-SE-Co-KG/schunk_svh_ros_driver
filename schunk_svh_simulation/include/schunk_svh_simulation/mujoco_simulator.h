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
#include <mutex>
#include <string>
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
  std::vector<double> pos_cmd;
  std::vector<double> pos_state;
  std::vector<double> vel_state;
  std::vector<double> eff_state;

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
  static int simulate(const std::string & model_xml);
  int simulateImpl(const std::string & model_xml);

  // Non-blocking
  void read(std::vector<double> & pos, std::vector<double> & vel, std::vector<double> & eff);
  void write(const std::vector<double> & pos);
};

}  // namespace schunk_svh_simulation
