^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package robot_localization
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

3.9.2 (2025-03-21)
------------------
* Same change on rolling (`#918 <https://github.com/cra-ros-pkg/robot_localization/issues/918>`_)
  Co-authored-by: Ferry Schoenmakers <ferry.schoenmakers@nobleo.nl>
* Fixing bug with diagonal covariance loading (`#909 <https://github.com/cra-ros-pkg/robot_localization/issues/909>`_)
* Switching stamped control to default true for rolling/future distributions to align with Nav2 and ROS 2 Control (`#910 <https://github.com/cra-ros-pkg/robot_localization/issues/910>`_)
* Added subscription to stamped topic (`#898 <https://github.com/cra-ros-pkg/robot_localization/issues/898>`_)
  * Added subscription to stamped topic
* Fixing IMU differential test (`#897 <https://github.com/cra-ros-pkg/robot_localization/issues/897>`_)
* Contributors: Ferry Schoenmakers, Pablo, Steve Macenski, Tom Moore

3.9.1 (2024-08-29)
------------------
* Switch robot_localization to modern CMake idioms. (`#895 <https://github.com/cra-ros-pkg/robot_localization/issues/895>`_)
* Fix warnings when building against Rolling. (`#896 <https://github.com/cra-ros-pkg/robot_localization/issues/896>`_)
* Resolve mixing of UTM and local transforms in local cartesian mode (`#886 <https://github.com/cra-ros-pkg/robot_localization/issues/886>`_)
* Spam the logs a little bit less (`#880 <https://github.com/cra-ros-pkg/robot_localization/issues/880>`_)
* Contributors: Chris Lalancette, JayHerpin, Tim Clephas

3.9.0 (2024-04-24)
------------------

3.8.0 (2024-04-21)
------------------
* Fixing yaml linking in rolling (`#878 <https://github.com/cra-ros-pkg/robot_localization/issues/878>`_)
* Contributors: Chris Lalancette

3.6.1 (2024-04-16)
------------------
* TF Prefix Bug (`#876 <https://github.com/cra-ros-pkg/robot_localization/issues/876>`_)
* Update ukf.yaml to match ekf.yaml (`#867 <https://github.com/cra-ros-pkg/robot_localization/issues/867>`_)
  Add missing *_pose_use_child_frame parameter.
* Fix throttle duration (`#866 <https://github.com/cra-ros-pkg/robot_localization/issues/866>`_)
* Migrate static tfs to ros2 format. (`#864 <https://github.com/cra-ros-pkg/robot_localization/issues/864>`_)
* Update issue templates
* Feature/set utm service (`#856 <https://github.com/cra-ros-pkg/robot_localization/issues/856>`_)
* fix: modify dual_ekf_navsat_example.launch file to remap the correct imu topic (`#857 <https://github.com/cra-ros-pkg/robot_localization/issues/857>`_)
* fix header timestamp (`#852 <https://github.com/cra-ros-pkg/robot_localization/issues/852>`_)
  Co-authored-by: Luke Chang <luke@boxfish.nz>
* Wait for odometry message before setting manual datum so that the base and world frame names can be set. (`#836 <https://github.com/cra-ros-pkg/robot_localization/issues/836>`_)
  * wait for odom msg before setting manual datum
* Test navsat transform functionality (`#838 <https://github.com/cra-ros-pkg/robot_localization/issues/838>`_)
* Utm using geographiclib ros2 branch (`#833 <https://github.com/cra-ros-pkg/robot_localization/issues/833>`_)
  * Add single test for navsat_conversions
  * Add a southern point to the navsat_transform test
  * LLtoUTM using GeographicLib
  * Use GeographicLib for UTMtoLL conversions
  * Linting
  * Forgot include
  * Fix compilation
  * Calculate gamma because it's a function output and was supplied before
  * Also test for gamma conversion
  * Align naming and install
* bugfix (`#809 <https://github.com/cra-ros-pkg/robot_localization/issues/809>`_): check if covariance values are specified or not (`#810 <https://github.com/cra-ros-pkg/robot_localization/issues/810>`_)
* Contributors: Daisuke Sato, Luke Chang, Mukunda Bharatheesha, Tim Clephas, Tom Greier, Tom Moore, joeldushouyu, rafal-gorecki, thandal

3.3.2 (2022-12-11)
------------------
* Port PR `#753 <https://github.com/cra-ros-pkg/robot_localization/issues/753>`_ and `#728 <https://github.com/cra-ros-pkg/robot_localization/issues/728>`_ to ROS2 version (`#765 <https://github.com/cra-ros-pkg/robot_localization/issues/765>`_)
  * compiling version of commit `#753 <https://github.com/cra-ros-pkg/robot_localization/issues/753>`_ and `#728 <https://github.com/cra-ros-pkg/robot_localization/issues/728>`_ ported to ros2 rolling
  * format fixes
  * fix time source disagreement by converting to seconds beforehand, append parameter usage, fix linting
  * fix linting and uncrustify
* Adding support for setting diagonals for covariance matrices (`#755 <https://github.com/cra-ros-pkg/robot_localization/issues/755>`_)
* Fixing and cleaning up interface tests (`#754 <https://github.com/cra-ros-pkg/robot_localization/issues/754>`_)
  * Fixing and cleaning up interface tests
* Linting and header cleanup, part 1 (`#752 <https://github.com/cra-ros-pkg/robot_localization/issues/752>`_)
  * Header cleanup and logging fixes
* UKF update (`#751 <https://github.com/cra-ros-pkg/robot_localization/issues/751>`_)
  * Pulling UKF changes from Noetic
  * Adding a reset service to make tests more deterministic
* Fix angles dependency (`#747 <https://github.com/cra-ros-pkg/robot_localization/issues/747>`_)
  * Add angles to CMakeLists.txt
* Using angles library to normalize angles (`#739 <https://github.com/cra-ros-pkg/robot_localization/issues/739>`_)
  * Using angles library for innovation angle normalization
* Read predict_to_current_time from ROS parameters (`#737 <https://github.com/cra-ros-pkg/robot_localization/issues/737>`_)
  Co-authored-by: Zygfryd Wieszok <zwieszok@autonomous-systems.pl>
* Fixed state history reversion (`#736 <https://github.com/cra-ros-pkg/robot_localization/issues/736>`_)
  Co-authored-by: Zygfryd Wieszok <zwieszok@autonomous-systems.pl>
* Fixing code style divergence for ament_uncrustify in main ROS2 branch (`#743 <https://github.com/cra-ros-pkg/robot_localization/issues/743>`_)
* This fixes `#732 <https://github.com/cra-ros-pkg/robot_localization/issues/732>`_ (broadcast_cartesian_transform) (`#733 <https://github.com/cra-ros-pkg/robot_localization/issues/733>`_)
* Contributors: Anish, Haoguang Yang, Marek Piechula, RoboTech Vision, Tom Moore, Tony Najjar, Zygfryd Wieszok

3.3.1 (2022-02-25)
------------------
* Moving to C++17 support (`#725 <https://github.com/cra-ros-pkg/robot_localization/issues/725>`_)
* SHARED linking for Geographiclib (`#624 <https://github.com/cra-ros-pkg/robot_localization/issues/624>`_) (`#712 <https://github.com/cra-ros-pkg/robot_localization/issues/712>`_)
  * remove GeographicLib specific linking option
  Co-authored-by: Achmad Fathoni <fathoni.id@gmail.com>
* Contributors: Stephan Sundermann, Tom Moore

2.4.0 (2017-06-12)
------------------
* Updated documentation
* Added reset_on_time_jump option
* Added feature to optionally publish utm frame as parent in navsat_transform_node
* Moved global callback queue reset
* Added initial_state parameter and documentation
* Fixed ac/deceleration gains default logic
* Added gravity parameter
* Added delay and throttle if tf lookup fails
* Fixed UKF IMUTwistBasicIO test
* Added transform_timeout parameter
* Set gps_odom timestamp before tf2 lookuptransform
* Removed non-portable sincos calls
* Simplified logic to account for correlated error
* Added dynamic process noise covariance calculation
* Fixed catkin_package Eigen warning
* Added optional publication of acceleration state
* Contributors: Brian Gerkey, Enrique Fernandez, Jochen Sprickerhof, Rein Appeldoorn, Simon Gene Gottlieb, Tom Moore

2.3.1 (2016-10-27)
------------------
* Adding gitignore
* Adding remaining wiki pages
* Adding config and prep pages
* Adding navsat_transform_node documentation
* use_odometry_yaw fix for n_t_n
* Fixing issue with manual pose reset when history is not empty
* Getting inverse transform when looking up robot's pose.
* Sphinx documentation
* Removing forward slashes from navsat_transform input topics for template launch file
* Adding example launch and parameter files for a two-level EKF setup with navsat_transform_node
* Adding yaml file for navsat_transform_node, and moving parameter documentation to it.
* Updating EKF and UKF parameter templates with usage comments
* Contributors: Tom Moore, asimay

2.3.0 (2016-07-28)
------------------
* Fixed issues with datum usage and frame_ids
* Fixed comment for wait_for_datum
* Fixing issue with non-zero navsat sensor orientation offsets
* Fixing issue with base_link->gps transform wrecking the 'true' UTM position computation
* Using correct covariance for filtered GPS
* Fixed unitialized odometry covariance bug
* Added filter history and measurement queue behavior
* Changing output timestamp to more accurately use the time stamp of the most recently-processed measurement
* Added TcpNoDelay()
* Added parameter to make transform publishing optional
* Fixed differential handling for pose data so that it doesn't care about the message's frame_id
* Updated UKF config and launch
* Added a test case for the timestamp diagnostics
* Added reporting of bad timestamps via diagnostics
* Updated tests to match new method signatures
* Added control term
* Added smoothing capability for delayed measurements
* Making variables in navsat_transform conform to ROS coding standards
* Contributors: Adel Fakih, Ivor Wanders, Marc Essinger, Tobias Tueylue, Tom Moore

2.2.3 (2016-04-24)
------------------
* Cleaning up callback data structure and callbacks and updating doxygen comments in headers
* Removing MessageFilters
* Removing deprecated parameters
* Adding the ability to handle GPS offsets from the vehicle's origin
* Cleaning up navsat_transform.h
* Making variables in navsat_transform conform to ROS coding standards

2.2.2 (2016-02-04)
------------------
* Updating trig functions to use sincos for efficiency
* Updating licensing information and adding Eigen MPL-only flag
* Added state to imu frame transformation
* Using state orientation if imu orientation is missing
* Manually adding second spin for odometry and IMU data that is passed to message filters
* Reducing delay between measurement reception and filter output
* Zero altitute in intital transform too, when zero altitude param is set
* Fixing regression with conversion back to GPS coordinates
* Switched cropping of orientation data in inovationSubset with mahalanobis check to prevent excluding measurements with orientations bigger/smaller than ± PI
* Fix Jacobian for EKF.
* Removing warning about orientation variables when only their velocities are measured
* Checking for -1 in IMU covariances and ignoring relevant message data
* roslint and catkin_lint applied
* Adding base_link to datum specification, and fixing bug with order of measurement handling when a datum is specified. Also added check to make sure IMU data is transformable before using it.
* Contributors: Adnan Ademovic, Jit Ray Chowdhury, Philipp Tscholl, Tom Moore, ayrton04, kphil

2.2.1 (2015-05-27)
------------------
* Fixed handling of IMU data w.r.t. differential mode and relative mode

2.2.0 (2015-05-22)
------------------
* Added tf2-friendly tf_prefix appending
* Corrected for IMU orientation in navsat_transform
* Fixed issue with out-of-order measurements and pose resets
* Nodes now assume ENU standard for yaw data
* Removed gps_common dependency
* Adding option to navsat_transform_node that enables the use of the heading from the odometry message instead of an IMU.
* Changed frame_id used in setPoseCallback to be the world_frame
* Optimized Eigen arithmetic for signficiant performance boost
* Migrated to tf2
* Code refactoring and reorganization
* Removed roll and pitch from navsat_transform calculations
* Fixed transform for IMU data to better support mounting IMUs in non-standard orientations
* Added feature to navsat_transform_node whereby filtered odometry data can be coverted back into navsat data
* Added a parameter to allow future dating the world_frame->base_link_frame transform.
* Removed deprecated differential setting handler
* Added relative mode
* Updated and improved tests
* Fixing source frame_id in pose data handling
* Added initial covariance parameter
* Fixed bug in covariance copyinh
* Added parameters for topic queue sizes
* Improved motion model's handling of angular velocities when robot has non-zero roll and pitch
* Changed the way differential measurements are handled
* Added diagnostics

2.1.7 (2015-01-05)
------------------
* Added some checks to eliminate unnecessary callbacks
* Updated launch file templates
* Added measurement outlier rejection
* Added failure callbacks for tf message filters
* Added optional broadcast of world_frame->utm transform for navsat_transform_node
* Bug fixes for differential mode and handling of Z acceleration in 2D mode

2.1.6 (2014-11-06)
------------------
* Added unscented Kalman filter (UKF) localization node
* Fixed map->odom tf calculation
* Acceleration data from IMUs is now used in computing the state estimate
* Added 2D mode

2.1.5 (2014-10-07)
------------------
* Changed initial estimate error covariance to be much smaller
* Fixed some debug output
* Added test suite
* Better compliance with REP-105
* Fixed differential measurement handling
* Implemented message filters
* Added navsat_transform_node

2.1.4 (2014-08-22)
------------------
* Adding utm_transform_node to install targets

2.1.3 (2014-06-22)
------------------
* Some changes to ease GPS integration
* Addition of differential integration of pose data
* Some documentation cleanup
* Added UTM transform node and launch file
* Bug fixes

2.1.2 (2014-04-11)
------------------
* Updated covariance correction formulation to "Joseph form" to improve filter stability.
* Implemented new versioning scheme.

2.1.1 (2014-04-11)
------------------
* Added cmake_modules dependency for Eigen support, and added include to silence boost::signals warning from tf include

