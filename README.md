a bunch of tutorials:

http://wiki.ros.org/p2os-purdue
http://wiki.ros.org/p2os/Tutorials/p2os%20with%20local%20computer%20USB%20communication
http://wiki.ros.org/p2os/Tutorials/Getting%20Started%20with%20p2os
http://wiki.ros.org/p2os/Tutorials/Controlling%20Pioneers%20using%20p2os
http://wiki.ros.org/p2os-purdue/Tutorials/GMapping%20With%20Pioneer-3dx
http://wiki.ros.org/slam_gmapping/Tutorials/MappingFromLoggedData

-------------------------------------------------
# TODO::

1. ??? who even knows

2. sometimes the navigation stack works, sometimes it doesn't

    a. running the exact same commands as documented below.. mind you
    
3. the navigator.py successfully sets a goal pose

    a. as displayed on the rviz (when the navigation stack decides to work)
    
    b. but it doesn't seem to really want to go there
    
    c. again, this might be because the navigation stack was tired cuz it was so late
    
4. honestly who knows :-)


---------------------------------------------------------------------
=====================================================================
========================RUNNING THE NAVIGATION/CONTROLLER============
=========================INSTRUCTIONS================================

0. add the following lines to your ~/.bashrc
    
    export ROS_MASTER_URI=http://10.0.1.22:11311
    
    export ROS_IP=10.0.1.20         //or you're ip address
    
    
1. Need to ssh into your pi, with 4 tabs (at least!)
2.
    a. turn on the pi with the dongle connected (should auto connect)

    b. connect laptop to UA-DSL

    c1. arp -a (and look for an ip address of the pi)

            OR

    c2. (just plug in pi to a monitor to find it's ipaddress on wlan1)

    d. ssh ubuntu@10.0.1.22     (or whatever ip address)

    e. on the four tabs that you open, run the following (while connected to pi)

---------TAB 1 (raspi)------------------------------------

$ roscore

----------------------------------------------------------
---------TAB 2 (raspi)------------------------------------

$ cs ~/homework4

$ roslaunch p2os_launch pioneer.launch

----------------------------------------------------------
---------TAB 3 (raspi)------------------------------------

$ cd ~/homework4/

$ rosparam set use_sim_time true

$ roslaunch p2os_launch navigation.launch

----------------------------------------------------------
--------------TAB 3.5 (LAPTOP BASE STATION)--------------

$ rosrun rviz rviz

----------------------------------------------------------
---------TAB 4 (LAPTOP BASE STATION)------------------------------------

$ cd ~/catkin_ws

$ rosrun homework3 navigator.py
