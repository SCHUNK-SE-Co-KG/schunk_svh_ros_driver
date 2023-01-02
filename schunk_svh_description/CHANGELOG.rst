^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package schunk_svh_description
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.0.1 (2023-01-02)
------------------
* Unify version numbers
  The version convention is as follows:
  `0.1.x` for ROS1 Melodic and Noetic
  `2.0.x` for ROS2 Foxy
  `2.1.x` for ROS2 Humble
  We start with `2.0.0` and bumb that with bloom to `2.0.1` for the first
  release.
* Update maintainer info
  Let's keep the list short for now.
* Merge branch 'update-license-ros2' into 'ros2-foxy-devel'
  Switch license to GPLv3
  See merge request ros/schunk_svh_driver!24
* Update SPDX license indicator in package.xml
  This is according to
  `here <https://www.gnu.org/licenses/identify-licenses-clearly.html>`_
* Add a high-level readme file
  Also add a minimal readme for the description sub package.
* Merge branch 'meta-package' into 'ros2-foxy-devel'
  Make this a meta package
  See merge request ros/schunk_svh_driver!21
* Merge remote-tracking branch 'schunk_svh_description/prepare-inclusion-into-meta-package' into meta-package
* Move everything into a equally named sub package
  We'll merge that into the `schunk_svh_ros_driver` meta package.
* Contributors: Stefan Scherzinger
