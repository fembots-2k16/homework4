# INSTRUCTIONS: to make the map
-------------------------------------------------
//------------------INSTRUCTIONS------------------
------------------------------------------------

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

// NOTE: ignoring the error message about map_server seemed to be ok

--------------------------------------------------
---------TAB 3 (raspi)------------------------------------

$ rosbag record -O mylaserdata /scan /tf

--------------------------------------------------
---------TAB 4 (raspi)------------------------------------

$ rostopic pub /cmd_motor_state p2os_msgs/MotorState 1
$ rosrun teleop_twist_keyboard teleop_twist_keyboard.py

// can now move the robot around with your keyboard
// remember to always press k after issuing any command to make it stop and
//   only move forward or turn a little bit at a time

--------------------------------------------------
=====================================================

2. After finishing moving the robot around, you can stop all the processes and you should have a
    file called mylaserdata.bag (in your ~/ directory)

3. Now, you need to convert the bag to a map??

    A. (optional) Have roscore running on your laptop along with rviz if you want to visualize the map?

        -----------TAB 1 (LAPTOP)----------------

        $ roscore

        -----------------------------------------
        -----------TAB 2 (LAPTOP)----------------

        $ rosrun rviz rviz

        -----------------------------------------
   ==================================================

   B. CONVERT THE BAG TO A MAP ON THE RASPBERRY PI (can use previous ssh tabs)

       ------------TAB 1 (raspi)-----------------

       $ roscore

       ------------------------------------------
       ------------TAB 2 (raspi)-----------------

       $ rosparam set use_sim_time true

       $ rosrun gmapping slam_gmapping scan:=scan

       ------------------------------------------
       ------------TAB 3 (raspi)-----------------

       $ rosbag play --clock mylaserdata.bag

       // let the command above finish all the way through, and then...

       $ rosrun map_server map_saver -f mymap

       ------------------------------------------

   ====================================================s
