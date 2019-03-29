# Baxter-Robotic-Arm

## This project is ongoing.

Interfacing the Baxter arm with a kinect in gazebo. The URDF has been modified from the original to include the Kinect URDF.

This project relies on the [Baxter Simulator](http://sdk.rethinkrobotics.com/wiki/Simulator_Installation) and the [Octomap](http://wiki.ros.org/octomap) library for ROS so it should be installed prior to merging this repository.

This project also relies on [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page) so you should also make sure that it is installed before continuing. To install it, [download](http://eigen.tuxfamily.org/index.php?title=Main_Page) the latest version of Eigen and go through the following steps:
1. Unzip the contents and then navigate to the directory.
2. Create a folder name "build" in the current directory.
3. In the "build" directory, run "cmake .."
4. After this process finishes run "make" (Note: you may not see any output from this)
5. Run "sudo make install" and it will install all the required Eigen libraries.

Once this is done, the project should run with no problem since the linking of this library has been handled in the CMake files. 

