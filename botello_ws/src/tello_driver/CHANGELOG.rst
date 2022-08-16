^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package tello_driver
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.3.0 (2019-10-12)
------------------
* Merge branch 'development' of https://github.com/appie-17/tello_driver into development
* CMake configuration include functional tello_driver for ROS repo
* Update README.md
* Fix installation clone from branche develop-0.7.0
* Gamepad_node fix flattrim to square, altitude and attitude limit params
* Manual takeoff topic
* Revert unfinished features
* Addition of gamepad_teleop_node.py and joy node
* Contributors: appie-17

0.2.0 (2019-10-02 15:29)
------------------------
* tello_driver 0.2.0 dependends on codec_image_transport instead of h264_image_transport
* Merge branch 'codec_image_transport' into development
* Remove cloning h264_image_transport git repository
* Replace h264_image_transport 3rd party package by codec_image_transport ROS
* Add tag V0.1.0
* Contributors: appie-17

0.1.0 (2019-10-02 11:51:23 +0200)
---------------------------------
* Update documentation
* Merge branch 'development' of https://github.com/appie-17/tello_driver into development
* Update documentation
* Fix tello_driver_node odom- and image frames
* Development of Swarmlab tello_driver
* Update README.md
* Update README.md
* Update README.md
* added fast mode; bugfixes
* added cmd_XYZ to status msg + stream_h264_video rosparam + test scripts
* video settings and improvements
* working code for controlling 2(+) drones using wifi docker proxy
* updated gitignore
* improved networking
* client_port
* comments
* docs and small fixes
* missing launch files + docs
* improved node + sync with TelloPy
* working wrapper
* typo
* readme
* added proxy docker container for connecting to multiple drones
* first try using pytello (will switch to TelloPy)
* Initial commit
* Contributors: Anqi Xu, Jordy, appie-17
