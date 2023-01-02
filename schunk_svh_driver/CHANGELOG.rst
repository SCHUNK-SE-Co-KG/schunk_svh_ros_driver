^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package schunk_svh_driver
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.0.1 (2023-01-02)
------------------
* Unify version numbers
  The version convention is as follows:
  `0.1.x` for ROS1 Melodic and Noetic
  `2.0.x` for ROS2 Foxy
  `2.1.x` for ROS2 Humble
  We start with `2.0.0` and bumb that with bloom to `2.0.1` for the first
  release.
* Add missing execution dependencies
  We need those for our current, minimal setup.
* Update maintainer info
  Let's keep the list short for now.
* Merge branch 'update-license-ros2' into 'ros2-foxy-devel'
  Switch license to GPLv3
  See merge request ros/schunk_svh_driver!24
* Add license notice to all development files
  The text is in accordance with the recommendations from
  `here <https://www.gnu.org/licenses/gpl-howto.html>`_
  in the section *The license notices*.
* Update SPDX license indicator in package.xml
  This is according to
  `here <https://www.gnu.org/licenses/identify-licenses-clearly.html>`_
* Add license text for the GPLv3
  The license text is from
  `here <https://www.gnu.org/licenses/gpl-3.0.txt>`_ after following the
  recommendations from `here <https://www.gnu.org/licenses/gpl-howto.html>`_
* Remove debug comment from launch file
* Update the driver's sup package readme
* Fix build warnings about unused parameters
* Merge branch 'improve-test-gui' into 'ros2-foxy-devel'
  Improve test gui
  See merge request ros/schunk_svh_driver!22
* Improve the gui layout
  The sliders are now horizontally arranged and labeled.
  Also increase the trajectory speed.
* Add a slider for each individual joint
  This allows to test if each joint behaves as expected.
* Merge branch 'meta-package' into 'ros2-foxy-devel'
  Make this a meta package
  See merge request ros/schunk_svh_driver!21
* Add execution dependency for the description package
* Prepare making this a meta repository
  We'll integrate the `schunk_svh_description` into this package.
* Contributors: Stefan Scherzinger
