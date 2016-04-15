# SETUP

    $ cd ~
    
    $ git clone https://github.com/fembots-2k16/homework4

----------------------------------------------------------

# running it

---------TAB 1--------------------------------------------

    $ roscore

---------TAB 2-------------------------------------------

    $ cd ~/homework4

    $ roslaunch pioneer.launch

---------TAB 3--------------------------------------------

    $ cd ~/homework4/

    # make sure you're plugged into the robot, the robot is on

    $ chmod 777 /dev/ttyACM0

    $ chmod 777 /dev/ttyUSB0
    
    $ source ~/catkin_ws/devel/setup.bash

    $ roslaunch navigation.launch

--------------TAB 3.5-------------------------------------

    $ rviz

---------TAB 4--------------------------------------------

    $ cd ~/homework4

    $ chmod 777 navigator2.py

    $ source ~/catkin_ws/devel/setup.bash

    $ ./navigator2.py

