a bunch of tutorials:

http://wiki.ros.org/p2os-purdue
http://wiki.ros.org/p2os/Tutorials/p2os%20with%20local%20computer%20USB%20communication
http://wiki.ros.org/p2os/Tutorials/Getting%20Started%20with%20p2os
http://wiki.ros.org/p2os/Tutorials/Controlling%20Pioneers%20using%20p2os
http://wiki.ros.org/p2os-purdue/Tutorials/GMapping%20With%20Pioneer-3dx
http://wiki.ros.org/slam_gmapping/Tutorials/MappingFromLoggedData

-------------------------------------------------
# TODO::

1. need to learn how to use 'navigation stack' to localize robot in our map based on its starting position?
  -(can we just set the starting position in rviz? probably)
2. need to make the AI controller that will:
  a. move to the goal position from the start position
  b. do so while avoiding obstacles in the map (and using the map)
  c. do so WHILE AVOIDING NEW, LOCAL OBSTACLES

---------------------------------------------------------------------
=====================================================================
========================RUNNING THE NAVIGATION/CONTROLLER============
=========================INSTRUCTIONS================================

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

--------------------------------------------------
---------TAB 2 (raspi)------------------------------------

$ roslaunch p2os_launch pioneer.launch

// why map server error?? do we need this?

--------------------------------------------------
---------TAB 3 (raspi)------------------------------------

$ rosrun map_server map_server mymap.yaml

//note, we can probably modify the launch file to do this for us automatically, and this might be what she wants since we're turning in the launch file too

--------------------------------------------------
---------TAB 4 (raspi)------------------------------------

$ cd ~/catkin_ws

$ rosrun homework3 pioneerController.py
