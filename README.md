# torsten_robot

This is the repository containing the ROS driver, configurations and additional packages of the TORsten mobile intralogistics robot. You can find additional informations and specifications of the system at http://torsten.torwegge.de/en/.

## Packages

The torsten_robot repository provides the following packages for ROS integration of the TORsten mobile intralogistics robot:

| Package | Description |
| --- | --- |
| **torsten_description** | Provides the mesh files and the static and dynamic transformation tree based on URDF description of the robot. |
| **torsten_driver** | The package contains the low-level communication to components with the CAN interface to the SEW Eurodrive motion PLC. The package maps the motion commands and system state into the ROS ecosystem and provides functionalities e.g. for moving bolts up and down. |
| **torsten_msgs** | Provides message  types  for controlling e.g. the bolts  of  the  system or providing system state information. |
| **torsten_navigation** | Contains  launch-files  for  the  integration  of  the  system  into  the ROS navigation stack |
| **torsten_robot** | Meta-package |
| **torsten_simulation** | Integration of the TORsten mobile intralogistics robot in the gazebo simulation environment |


# Installation instructions

The following instructions describes the installation procedure of the TORsten base computation unit. The base system is connected via CAN bus with the motion controller (SEW - PLC) of the vehicle. Additional communication is performed via Ethernet (e.g. connecting to laser scanners - SICK TIM571).

## Preliminaries

If you haven't installed the Robot Operating System (ROS) on your PC you can follow the instructions of the ros.org homepage (http://wiki.ros.org/kinetic/Installation/Ubuntu). The torsten_robot packages has been tested and deployed with ROS Kinetic Kame under Ubuntu 16.04.

## PCAN-USB

The base navigation unit is connected with the motion controller via CAN. The used interface module is PCAN USB adapter of PEAK Systems. For installation use the following link to download the "Driver Package for Proprietary Purposes":
> https://www.peak-system.com/fileadmin/media/linux/index.htm

Extract the driver package in your download directory:
```
tar -zxvf peak-linux-driver-8.8.0.tar.gz
```

Enter the driver package directory and install the driver:
```
cd peak-linux-driver-8.8.0/
make clean
make
sudo make install
```

# Packages

## torsten_description

The package contains the URDF description files of the TORsten mobile robot. You can run the description including RViz visualization using:

```
roslaunch torsten_description display_torsten_urdf.launch
```

[![TORsten robot description](http://img.youtube.com/vi/NI4llVJWaIg/0.jpg)](http://www.youtube.com/watch?v=NI4llVJWaIg "TORsten robot description")

## torsten_simulation

The torsten mobile robot is integrated in the move_base (http://wiki.ros.org/move_base) for navigation. The teb_local_planner (http://wiki.ros.org/teb_local_planner) is used for trajectory planning and collision avoidance. For localization AMCL (http://wiki.ros.org/amcl) is used.

```
roslaunch torsten_gazebo_robot torsten_base_sim.launch
```

[![TORsten gazebo integration](http://img.youtube.com/vi/ZZ93vayvxvY/0.jpg)](http://www.youtube.com/watch?v=ZZ93vayvxvY "TORsten gazebo integration")

