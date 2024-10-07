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
/*!\file    mujoco_simulator.cpp
 *
 * \author  Stefan Scherzinger <stefan.scherzinger@de.schunk.com>
 * \date    2024/10/01
 *
 */
//-----------------------------------------------------------------------------

#include "schunk_svh_simulation/mujoco_simulator.h"

#include <memory>

namespace schunk_svh_simulation
{
MuJoCoSimulator::MuJoCoSimulator() {}

void MuJoCoSimulator::keyboardCB(GLFWwindow * window, int key, int scancode, int act, int mods)
{
  getInstance().keyboardCBImpl(window, key, scancode, act, mods);
}

void MuJoCoSimulator::keyboardCBImpl(
  [[maybe_unused]] GLFWwindow * window, int key, [[maybe_unused]] int scancode, int act,
  [[maybe_unused]] int mods)
{
  // backspace: reset simulation
  if (act == GLFW_PRESS && key == GLFW_KEY_BACKSPACE) {
    mj_resetData(m, d);
    mju_copy(d->qpos, m->key_qpos, 7);  // initial states from xml
    mj_forward(m, d);
  }
}

// mouse button callback
void MuJoCoSimulator::mouseButtonCB(GLFWwindow * window, int button, int act, int mods)
{
  getInstance().mouseButtonCBImpl(window, button, act, mods);
}

void MuJoCoSimulator::mouseButtonCBImpl(
  GLFWwindow * window, [[maybe_unused]] int button, [[maybe_unused]] int act,
  [[maybe_unused]] int mods)
{
  // update button state
  button_left = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS);
  button_middle = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS);
  button_right = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS);

  // update mouse position
  glfwGetCursorPos(window, &lastx, &lasty);
}

// mouse move callback
void MuJoCoSimulator::mouseMoveCB(GLFWwindow * window, double xpos, double ypos)
{
  getInstance().mouseMoveCBImpl(window, xpos, ypos);
}

void MuJoCoSimulator::mouseMoveCBImpl(GLFWwindow * window, double xpos, double ypos)
{
  // no buttons down: nothing to do
  if (!button_left && !button_middle && !button_right) {
    return;
  }

  // compute mouse displacement, save
  double dx = xpos - lastx;
  double dy = ypos - lasty;
  lastx = xpos;
  lasty = ypos;

  // get current window size
  int width, height;
  glfwGetWindowSize(window, &width, &height);

  // get shift key state
  bool mod_shift =
    (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS ||
     glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS);

  // determine action based on mouse button
  mjtMouse action;
  if (button_right) {
    action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
  } else if (button_left) {
    action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
  } else {
    action = mjMOUSE_ZOOM;
  }

  // move camera
  mjv_moveCamera(m, action, dx / height, dy / height, &scn, &cam);
}

// scroll callback
void MuJoCoSimulator::scrollCB(GLFWwindow * window, double xoffset, double yoffset)
{
  getInstance().scrollCBImpl(window, xoffset, yoffset);
}

void MuJoCoSimulator::scrollCBImpl(
  [[maybe_unused]] GLFWwindow * window, [[maybe_unused]] double xoffset, double yoffset)
{
  // emulate vertical mouse motion = 5% of window height
  mjv_moveCamera(m, mjMOUSE_ZOOM, 0, -0.05 * yoffset, &scn, &cam);
}

void MuJoCoSimulator::controlCB(const mjModel * m, mjData * d)
{
  getInstance().controlCBImpl(m, d);
}

void MuJoCoSimulator::controlCBImpl([[maybe_unused]] const mjModel * m, mjData * d)
{
  command_mutex.lock();

  for (size_t i = 0; i < pos_cmd.size(); ++i) {
    // Joint-level impedance control
    d->ctrl[i] = stiff[i] * (pos_cmd[i] - d->qpos[i]) +             // stiffness
                 damp[i] * (vel_cmd[i] - d->actuator_velocity[i]);  // damping
  }
  command_mutex.unlock();
}

int MuJoCoSimulator::simulate(const std::string & model_xml)
{
  return getInstance().simulateImpl(model_xml);
}

int MuJoCoSimulator::simulateImpl(const std::string & model_xml)
{
  // Make sure that the ROS2-control system_interface only gets valid data in read().
  // We lock until we are done with simulation setup.
  state_mutex.lock();

  // load and compile model
  char error[1000] = "Could not load binary model";
  m = mj_loadXML(model_xml.c_str(), nullptr, error, 1000);
  if (!m) {
    mju_error_s("Load model error: %s", error);
    return 1;
  }

  // Set initial state with the keyframe mechanism from xml
  d = mj_makeData(m);
  mju_copy(d->qpos, m->key_qpos, m->nq);

  // Initialize buffers for ROS2-control.
  pos_state.resize(m->nu);
  vel_state.resize(m->nu);
  eff_state.resize(m->nu);
  pos_cmd.resize(m->nu);
  vel_cmd.resize(m->nu);
  stiff.resize(m->nu);
  damp.resize(m->nu);

  // Start where we are
  syncStates();
  state_mutex.unlock();

  // init GLFW
  if (!glfwInit()) {
    mju_error("Could not initialize GLFW");
  }

  // create window, make OpenGL context current, request v-sync
  GLFWwindow * window = glfwCreateWindow(1200, 900, "Demo", NULL, NULL);
  glfwMakeContextCurrent(window);
  glfwSwapInterval(1);

  // initialize visualization data structures
  mjv_defaultCamera(&cam);
  mjv_defaultOption(&opt);
  mjv_defaultScene(&scn);
  mjr_defaultContext(&con);

  // create scene and context
  mjv_makeScene(m, &scn, 2000);
  mjr_makeContext(m, &con, mjFONTSCALE_150);

  // install GLFW mouse and keyboard callbacks
  glfwSetKeyCallback(window, keyboardCB);
  glfwSetMouseButtonCallback(window, mouseButtonCB);
  glfwSetCursorPosCallback(window, mouseMoveCB);
  glfwSetScrollCallback(window, scrollCB);

  // Connect our specific control input callback for MuJoCo's engine.
  mjcb_control = MuJoCoSimulator::controlCB;

  // Simulate in realtime
  while (!glfwWindowShouldClose(window)) {
    // advance interactive simulation for 1/60 sec
    //  Assuming MuJoCo can simulate faster than real-time, which it usually can,
    //  this loop will finish on time for the next frame to be rendered at 60 fps.
    //  Otherwise add a cpu timer and exit this loop when it is time to render.
    mjtNum simstart = d->time;
    while (d->time - simstart < 1.0 / 60.0) {
      mj_step(m, d);
    }

    // get framebuffer viewport
    mjrRect viewport = {0, 0, 0, 0};
    glfwGetFramebufferSize(window, &viewport.width, &viewport.height);

    // update scene and render
    mjv_updateScene(m, d, &opt, NULL, &cam, mjCAT_ALL, &scn);
    mjr_render(viewport, &scn, &con);

    // swap OpenGL buffers (blocking call due to v-sync)
    glfwSwapBuffers(window);

    // process pending GUI events, call GLFW callbacks
    glfwPollEvents();

    // Provide fresh data for ROS2-control
    state_mutex.lock();
    syncStates();
    state_mutex.unlock();
  }

  // free visualization storage
  mjv_freeScene(&scn);
  mjr_freeContext(&con);

  // free MuJoCo model and data
  mj_deleteData(d);
  mj_deleteModel(m);
  glfwTerminate();

  return 0;
}

void MuJoCoSimulator::read(
  std::vector<double> & pos, std::vector<double> & vel, std::vector<double> & eff)
{
  // Realtime in ROS2-control is more important than fresh data exchange.
  if (state_mutex.try_lock()) {
    pos = pos_state;
    vel = vel_state;
    eff = eff_state;
    state_mutex.unlock();
  }
}

void MuJoCoSimulator::write(
  const std::vector<double> & pos, const std::vector<double> & vel,
  const std::vector<double> & stiff, const std::vector<double> & damp)
{
  // Realtime in ROS2-control is more important than fresh data exchange.
  if (command_mutex.try_lock()) {
    pos_cmd = pos;
    vel_cmd = vel;
    this->stiff = stiff;
    this->damp = damp;
    command_mutex.unlock();
  }
}

void MuJoCoSimulator::syncStates()
{
  for (auto i = 0; i < m->nu; ++i) {
    pos_state[i] = d->qpos[i];
    vel_state[i] = d->actuator_velocity[i];
    eff_state[i] = d->actuator_force[i];
  }
}

}  // namespace schunk_svh_simulation
