^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package schunk_svh_description
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Merge pull request `#18 <https://github.com/fzi-forschungszentrum-informatik/schunk_svh_ros_driver/issues/18>`_ from fzi-forschungszentrum-informatik/add-gazebo-example
  Add an example setup for Gazebo
* Update the plugin to mimic joints
  The new plugin is now maintained
  [here](https://github.com/roboticsgroup/roboticsgroup_upatras_gazebo_plugins).
  Also add suitable `PID` gains for the mimicked joints.
* Add a minimal Gazebo example
* Update maintainer info
* Merge branch 'description_package_xml' into 'update-and-upgrade'
  Description package xml
  See merge request ros/schunk_svh_driver!25
* Added xml schema
* Removed boilerplate comments from svh_description package.xml
* Merge branch 'update-license-ros1' into 'update-and-upgrade'
  Switch license to the GPLv3
  See merge request ros/schunk_svh_driver!23
* Update cmake version for the description package
  We used an ancient version without no reason.
  CMake 3.10 is Bionic's (Ubuntu 18.04) shipping version.
* Update SPDX license indicator in package.xml
  This is according to
  [here](https://www.gnu.org/licenses/identify-licenses-clearly.html)
* Support left and right hands through launch parameters
  We now load the correct ROS-control configuration depending on whether
  users start a `right_hand` or a `left_hand`.
  This is also passed to the URDF robot description.
* Update top-level readme
  Also add lightweight readmes for each sub package.
* Merge branch 'meta-package' into 'update-and-upgrade'
  Meta package
  See merge request ros/schunk_svh_driver!20
* Fix installs
* Merge remote-tracking branch 'schunk_svh_description/prepare-inclusion-into-meta-package' into meta-package
  We continue the `schunk_svh_description` within this meta repository.
* Move everything into an equally named sub package
  We will later merge this package into the `schunk_svh_ros_driver` meta
  repository.
* Contributors: Felix Exner, Stefan Scherzinger
