# Sailing Robot Project
[![Build Status](https://travis-ci.org/Maritime-Robotics-Student-Society/sailing-robot.svg?branch=master)](https://travis-ci.org/Maritime-Robotics-Student-Society/sailing-robot) 
[![Code ship](https://codeship.com/projects/c7f9b0a0-ab07-0133-6c63-4a9cc2c4d260/status?branch=master)]()
[![Donate](https://img.shields.io/badge/Donate-PayPal-green.svg)](https://www.paypal.com/cgi-bin/webscr?cmd=_s-xclick&hosted_button_id=TUL52K7SWV2GL) and see your name below!

```
                                          This is Southampton Sailing Robot Team's code repository.
                -..                       We are building an one meter long autonomous sailing boat  
               oMNm+.                     that compete for the 2016 World Sailing Robot Competition.
               o: ./o/                    These code are based on ROS framework. 
               o:`Ns.                     Click WiKi page for more information. 
               o: dMMy-                   
               o: sMMMMy`                 Our project wouldn't be possible without kind sponsors: 
             : o: /MMMMMN/               
            :+ +/ .MMMMMMMs               Southampton University 
            m/ +/  MMMMMMMMd`             Southampton University Alumni (matching funding)
           sM- +/  NMMMMMMMMd`            Bob Preston
          :MM- +/  NMMMMMMMMMm.           Alistair Lynn
         .NMM. +/  NMMMMMMMMMMm`          Timfkmiller
         dMMM. +/  MMMMMMMMMMMMd`         Matt Brown
        yMMMM. +/ `MMMMMMMMMMMMMh         schneider
       oMMMMM- +/ -MMMMMMMMMMMMMMo        Soon Sun Gan
      +MMMMMM: +/ +MMMMMMMMMMMMMMM:       arnaud_wiertz
     +MMMMMMM+ ++ sMMMMMMMMMMMMMMMN`      Kerrine Lee
    /MNNmmNNMs /+ dMMMMMMMMMMMMMMMMh      Alessandro Romano
    .`      `` -: yso/::-....--:/+oy.     Lee Kwong Yong
  `southamptonuniversitysailingrobot-     andybs
   .odMMMMMMMMMMMMMMMMMMMMMMMMMMMMms-     Simone Provenzano
     `-:////////////////////////-         Alex Ziang 
                                          
                                          and many anonymous donors who support us silently.
                                          Thank you for your support!
```
First, get [ROS](http://www.ros.org/) set up. The easiest way to do this is with the [virtual machine](https://github.com/Maritime-Robotics-Student-Society/sailing-robot/wiki/Virtual-Machine-for-Windows-Mac-users).

Now, clone the repository. If you're not familiar with git, you'll want to read up on that soon, but for now, you can run this command in a terminal:

    git clone https://github.com/Maritime-Robotics-Student-Society/sailing-robot.git

You now have a copy of the code on your computer. Change into the new folder:

    cd sailing-robot

This folder is a [catkin workspace](http://wiki.ros.org/catkin/workspaces). To get started, run:

    catkin_make
    source devel/setup.bash  # This sets up environment variables

To run the tests:

    catkin_make run_tests

To launch the nodes

    roslaunch sailing_robot test-all-systems.launch

To run simulator, you need install dependencies
    
    sudo apt-get install python-scipy 
    sudo pip install --user LatLon shapely scipy
    
then 

    roslaunch sailing_robot simulator.launch &
    sleep 10
    rviz
    

