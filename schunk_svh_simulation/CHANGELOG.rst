^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package schunk_svh_simulation
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.2 (2023-01-05)
------------------
* Correctly depend on `schunk_svh_description`
  We only need that as an execution dependency.
* Contributors: Stefan Scherzinger

0.1.1 (2022-12-20)
------------------
* Merge pull request `#18 <https://github.com/fzi-forschungszentrum-informatik/schunk_svh_ros_driver/issues/18>`_ from fzi-forschungszentrum-informatik/add-gazebo-example
  Add an example setup for Gazebo
* Add a readme for the simulation
* Add a dedicated simulation package
  That's cleaner and keeps Gazebo simulation specific configuration
  separate from the actual driver.
* Contributors: Stefan Scherzinger
