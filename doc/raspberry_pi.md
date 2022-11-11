# Installation on a Raspberry Pi 4

This is a step-by-step tutorial for setting-up the Schunk SVH ROS2 driver on a freshly formatted Raspberry Pi 4.

1. [Prerequisites](#prerequisites)
2. [Create the Raspberry Pi 4 image](#create-image)
3. [Connect to the Raspberry Pi 4](#connect-to-rp4)
4. [Install ROS2 on the Raspberry Pi 4](#install-ros2)
4. [Install the Schunk SVH ROS2 driver](#install-schunk-driver)
4. [Test the Schunk SVH](#test-schunk-driver)

## Prerequisites <a name="prerequisites"></a>
You will need the following equipment:
  - A PC with Ubuntu 20.04. You'll also need *sudo* privileges and internet via Wifi. We refer to this PC as *laptop* in the following.
  - A Raspberry Pi 4, which we refer to as *RP4* in the following.
  - A USB-to-USBC cable for powering the *RP4* via the *laptop*
  - An Ethernet cable to connect the *laptop* with the *RP4*
  - A Schunk SVH hand with a suitable serial-to-USB2 adapter

In this tutorial, we assume that you do not have a micro HDMI cable for the *RP4* and that you operate the *RP4* on the command line via `ssh` from the *laptop*.

## Create the Raspberry Pi 4 image <a name="create-image"></a>
1. We use the very handy tool `rpi-imager` for setting-up the *RP4*'s SD card. It has an intuitive GUI and directly downloads suitable images for us.
   Install it via
   ```bash
   sudo snap install rpi-imager
   ```
   on the *laptop*.

2. Insert the *RP4*'s SD card (with adapter) into your laptop's SD card drive and open
   the imager tool with
   ```bash
   rpi-imager
   ```
   in a new terminal.

3. In the GUI, click on `CHOOSE STORAGE` and select the *RP4*'s SD card.

4. Click on `CHOOSE OS` and select `Other general-purpose OS` -> `Ubuntu` -> `Ubuntu Server 20.04.5 LTS (64Bit)`.
   We use `Ubuntu 20.04` because that gives us native support for ROS Noetic *and* ROS2 Foxy.
   Note that there's no special Ubuntu *Desktop* image available for the RP4 as discussed [here](https://askubuntu.com/questions/1348560). Instead, the process is to install the *Server* version
   and later install `ubuntu-desktop` on top if required. We'll be using only the Ubuntu *Server* in this tutorial.

5. Click on the gear symbol for further configuration:
   - select `Image customization options` -> `to always use`.
   - set a hostname, e.g. `schunk`
   - enable SSH with `Use password authentication`
   - Set username and password as desired. You will be using these credentials later for ssh from the *laptop* to the *RP4*.
   - Set locale settings and keyboard layout according to your preference.

6. Now click `SAVE`, then `WRITE` and confirm to proceed with erasing the SD card.
   The imager is now writing the image. When finished, remove the SD card from the *laptop* and insert it back into the *RP4*.
   Now power the *RP4* with the USBC cable via the *laptop*.


## Connect to the Raspberry Pi 4 <a name="connect-to-rp4"></a>
Directly connect the *laptop* with the *RP4* via the Ethernet cable. The
*RP4*'s LAN port should have DHCP as default config. Normally, we thus would
need to setup a DHCP server on our *laptop*'s LAN port and pass internet downlink to the *RP4*.
Luckily, that's all handled automatically by recent Ubuntu versions in a simple network configuration.

1. In a terminal, call
   ```bash
   nm-connection-editor
   ```
   to open the Network Connections on the *laptop*.

2. Add a new `Ethernet` connection and give it a `Connection name`, e.g. `svh_dhcp`.
   In the `IPv4 Settings`, choose `Shared to other computers` as `Method`.

3. That's it. Save and leave everything else untouched.
   Close the connection editor and select that new connection.
   We now need to find the address that the DHCP has given the *RP4*.
   In a terminal, call
   ```bash
   nmap -sP 10.42.0.1/24
   ```
   which should give us something similar to this:
   ```bash
   Starting Nmap 7.80 ( https://nmap.org ) at 2022-10-27 01:22 CEST
   Nmap scan report for ids-opel (10.42.0.1)
   Host is up (0.0011s latency).
   Nmap scan report for 10.42.0.166
   Host is up (0.00040s latency).
   Nmap done: 256 IP addresses (2 hosts up) scanned in 15.52 seconds
   ```
   In this case, the *laptop* (ids-opel) has the `10.42.0.1` and the `RP4` has the `10.42.0.166`.
   You might need to replug the Ethernet cable on the *RP4* and try again if the *RP4* does not appear on the network.
   Also note that the *RP4*'s Ubuntu needs some time to boot and prepare `ssh`.

4. Open a terminal on the *Laptop* and ssh to the *RP4* with
   ```bash
   ssh user@10.42.0.166
   ```
   where `user` is what you chose as credentials when setting up the *RP4* image from above.
   On the *RP4* get the latest updates with
   ```bash
   sudo apt update && sudo apt upgrade
   ```
   Reboot the system with
   ```bash
   sudo reboot
   ```




## Install ROS2 on the Raspberry Pi 4 <a name="install-ros2"></a>
1. Back on the *RP4* in a terminal, install ROS2 according to the [official documentation](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html)
   and choose the `Desktop Install (Recommended)` version.

2. When the installation is finished, source the ROS2 installation with
   ```bash
   source /opt/ros/foxy/setup.bash
   ```
3. In the same terminal, install `rosdep` with
   ```bash
   sudo apt install python3-rosdep
   ```
   This tool is required for installing the right dependencies. It works both for ROS1 and ROS2.
   Be careful to not install `python3-rosdep2` (note the trailing `2`), which is in fact the *outdated* version, as discussed [here](https://answers.ros.org/question/359586/).

4. Next, call
   ```bash
   rosdep update
   ```
   which prepares the ROS dependency management for the Schunk SVH driver that we will install later.

5. Our freshly installed Ubuntu will also need a C++ Compiler. Install it with
   ```bash
   sudo apt install build-essential
   ```

6. As a final step, install `Colcon` for a ROS2 context according to [these](https://colcon.readthedocs.io/en/released/user/installation.html#in-the-context-of-the-ros-project) instructions.

## Install the Schunk SVH ROS2 driver <a name="install-schunk-driver"></a>
Setup a new workspace with
```bash
mkdir -p $HOME/ros2_foxy_ws/src && cd "$_"
```
and follow the installation instructions from [here](https://github.com/fzi-forschungszentrum-informatik/schunk_svh_ros_driver/tree/ros2-foxy#installation).

## Test the Schunk SVH <a name="test-schunk-driver"></a>
In this tutorial, we'll control the hand via a script on the *laptop* and assume that you have both ROS2 Foxy *and* the Schunk SVH driver installed here as well.
As an alternative, you could, of course, control the hand directly on the *RP4*.

1. On the *RP4*, open a terminal and set your `ROS_IP` with
   ```bash
   export ROS_IP=10.42.0.166  # the RP4's IP
   ```
   Make sure to use the `RP4`'s IP address as it appeared on the network from above.
   This environment variable is important for ROS2 so that messages coming from the *RP4* are received correctly.

2. In the same terminal, follow the getting started steps from [here](https://github.com/fzi-forschungszentrum-informatik/schunk_svh_ros_driver/tree/ros2-foxy#getting-started) for starting the driver.

3. The Schunk SVH driver now runs on the *RP4*.
   On the *laptop*, open a terminal, navigate to your ROS2 workspace and
   ```bash
   source /opt/ros/foxy/setup.bash
   source install/local_setup.bash
   export ROS_MASTER_URI=http://10.42.0.166:11311  # the RP4's IP
   export ROS_IP=10.42.0.1  # the laptop's IP
   ```
   Remember to use the IP addresses on *your* network from above.
   This makes sure that this *laptop*'s terminal can communicate with the Schunk SVH driver that runs on the *RP4*.

4. Finally, start a minimalistic GUI on the *laptop* with
   ```bash
   ros2 run schunk_svh_driver example.py
   ```
   and move the individual joints.
