# turtlebot-ros
A guide for installing ROS Kinetic on a Turtlebot 2.0 (with Kobuki base).

# Contents

  - [turtlebot-ros](#turtlebot-ros)
  - [Contents](#contents)
  - [Resources](#resources)
    - [Run Multiple Programs in One Terminal](#run-multiple-programs-in-one-terminal)
    - [Terms](#terms)
    - [Command to Source ROS Programs](#command-to-source-ros-programs)
  - [Instructions](#instructions)
    - [Initial Setup](#initial-setup)
    - [Network Configuration](#network-configuration)
      - [Verify Communication](#verify-communication)
      - [Time Synchronization](#time-synchronization)
  - [Controlling the Turtlebot](#controlling-the-turtlebot)

# Resources

* [ROS Tutorials](http://wiki.ros.org/ROS/Tutorials)
* [ROS Basics Google Slides](https://docs.google.com/presentation/d/18JNSxixeIJML2rsXyQE-y0mSsnM_MSaF4crSbgW7DJE)
* [ROS Coordinate Frames](https://docs.google.com/presentation/d/14wKMJJGwoejncPXbATD35nIewAzpzuVFKDwLzp3rFOc)
* [ROS Installation on Turtlebot Tutorials](http://wiki.ros.org/turtlebot/Tutorials/indigo) - Using ROS Indigo tutorials as those for Kinetic have not been created.

## Run Multiple Programs in One Terminal
To have multiple commands running without opening multiple connections to a remote machine:

  1. Perform a <kbd>CTRL</kbd>+<kbd>Z</kbd>. This will exit back to the command prompt without killing the process.
  2. Run `bg` to make sure that the process continues.
  3. Run your next command. They will both run side-by-side, but you can only control the latest.

* To see previous processes, run `jobs`. This will list all currently backgrounded commands, with unique ids.
* To access a specific process, run `fg %N`, where `N` is the process id.

## Terms
* 'Turtlebot': Raspberry Pi connected to the Turtlebot 2.0 on a Kobuki base. The Raspberry Pi should be running some flavor of Linux.
* 'Remote Machine'/'Host'/'Your Machine': Your system, connected to the same network as the Turtlebot.

## Command to Source ROS Programs
This will be used both on the Turtlebot and on the remote machine.

    source /opt/ros/kinetic/setup.bash

# Instructions

## Initial Setup
Give the robot a brain.

* Install the base kinetic packages.
* Install `git`
* `git clone https://github.com/FRC900/2017Preseason.git`

Installation command adapted from the [Turtlebot Installation for ROS Indigo](http://wiki.ros.org/turtlebot/Tutorials/indigo/Turtlebot%20Installation#turtlebot.2BAC8-Tutorials.2BAC8-indigo.2BAC8-Debs_Installation.Ubuntu_Package_Install) page. We replaced all instances of `indigo` with `kinetic`.

    sudo apt-get install ros-kinetic-turtlebot ros-kinetic-turtlebot-apps ros-kinetic-turtlebot-interactions ros-kinetic-turtlebot-simulator ros-kinetic-kobuki-ftdi ros-kinetic-ar-track-alvar-msgs

* Packages that failed to install (did not exist): `ros-kinetic-rocon-remocon`, `ros-kinetic-rocon-qt-library`

Run rosrun kobuki_ftdi create_udev_rules

## Network Configuration
Give the robot speech.

[Network Configuration](http://wiki.ros.org/turtlebot/Tutorials/indigo/Network%20Configuration)

Make sure that OpenSSH Server is installed on the Turtlebot.

    sudo apt update
    sudo apt install openssh-server

Find the Ethernet IP of the Turtlebot with `ifconfig`, and SSH into it from your machine.

On the Turtlebot, edit `~/.bashrc` and add these lines to the end:

    ROS_MASTER_URI=http://localhost:11311
    ROS_HOSTNAME=`hostname -I`

On your machine, edit `~/.bashrc` and add these lines to the end. Replace `<Turtlebot-IP>` with the address of the Turtebot.

    ROS_MASTER_URI=http://<Turtlebot-IP>:11311
    ROS_HOSTNAME=`hostname -I`

Reboot the Turtlebot.

### Verify Communication
Verify that you can communicate with the Turtlebot by...

  1. SSH into the Turtlebot.
  2. Run the [source script](#command-to-source-ros-programs) on both the Turtlebot and remote machine.
  3. Start a ROS Server on the Turtlebot with `roscore`.
  4. On your machine, run `rostopic list`.

If you get an output similar to

    /rosout
    /rosout_agg

* If it fails with `ERROR: Unable to communicate with master!`, double-check that all of the IP's are configured correctly, and that the ROS server started without error on the Turtlebot.

Now verify that the Turtlebot can receive data from the remote connection by:

  1. On the remote machine: `rostopic pub -r10 /hello std_msgs/String "hello"`
  2. On the Turtlebot: `rostopic echo /hello`

If the Turtlebot prints out lots of

    ---
    data: hello

messages, it's working.

### Time Synchronization
Clock synchronization is important for ROS. Chrony has been found to be the best ntp client over lossy wireless connections.<br>
Run these on both systems.

    sudo apt install chrony ntpdate
    sudo ntpdate ntp.ubuntu.com


# Controlling the Turtlebot
Crash the robot into people.

These steps will allow controlling the robot with an Xbox 360 controller. For use with other controllers, see the [Ros Wiki](http://wiki.ros.org/turtlebot/Tutorials/indigo#Teleoperation)

  1. Make sure that `setup.bash` is [sourced](#command-to-source-ros-programs) on both the Turtlebot and your machine.
  2. Install the Kobuki dashboard software on the remote machine with: `sudo apt-get install ros-kinetic-kobuki-dashboard`.
  3. Run `roscore` on the Turtlebot.
  4. With the ROS Core still running, run `roslaunch turtlebot_bringup minimal.launch --screen` on the Turtlebot.

Warnings are generally fine, but if it brings up any errors see the [Troubleshooting Guide](http://wiki.ros.org/turtlebot_bringup/Tutorials/indigo/TurtleBot%20Bringup#Bringup_Troubleshooting).

If no errors are presented, continue!

* Run `rqt -s kobuki_dashboard` on your machine.

It might take a minute, but a window should open.<br>
To test connectivity, change the color of the LEDs on the Turtlebot using the indicator buttons on the top of the window. If the lights change color on the robot, it's working! If nothing happens, \[annoy Marshall about the networks not being bridged].

  5. Connect the Xbox controller to the Raspberry Pi.

### TODO: Write a ROS publisher-subscriber model to send controller command to Turtlebot from remote machine

  6. With both the `roscore` and first `roslaunch` command running, run: `roslaunch turtlebot_teleop xbox360_teleop.launch --screen`

The Turtlebot can now be driven by the controller! Hold the top left bumber and use the left stick to drive.

***
