^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package schunk_svh_driver
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.2 (2023-01-05)
------------------

0.1.1 (2022-12-20)
------------------
* Update the plugin to mimic joints
  The new plugin is now maintained
  `here <https://github.com/roboticsgroup/roboticsgroup_upatras_gazebo_plugins>`_.
  Also add suitable `PID` gains for the mimicked joints.
* Add `PID` gains for Gazebo
  We now have stable joint position control.
* Use a gazebo-specific controllers file
  We'll probably need a specifically tweaked set of `PID` gains for
  Gazebo. The default position controllers are not stable.
* Add a minimal Gazebo example
* Remove unused launch parameters
* Add missing install for the dynamic parameter test
* Remove unused simulation parameter from launch
  That's no longer supported.
* Add `joint_state_controller` as run dependency
* Run-depend on `joint_trajectory_controller`
  We need this thing for joint actuation in the examples.
* Merge branch 'fix-license-typo' into 'update-and-upgrade'
  Fix typo in license notice
  See merge request ros/schunk_svh_driver!26
* Fix typo in license notice
* Merge branch 'update-license-ros1' into 'update-and-upgrade'
  Switch license to the GPLv3
  See merge request ros/schunk_svh_driver!23
* Apply clang-format through pre-commit hook
* Add license notice to all development files
  The text is in accordance with the recommendations from
  `here <https://www.gnu.org/licenses/gpl-howto.html>`_
  in the section *The license notices*.
* Remove old license texts
* Update SPDX license indicator in package.xml
  This is according to
  `here <https://www.gnu.org/licenses/identify-licenses-clearly.html>`_.
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
* Use the new `schunk_svh_msgs` package
* Start making this a meta package
  We move everything into an equally called sub package.
  This should maintain our Git history.
* Contributors: Stefan Scherzinger
