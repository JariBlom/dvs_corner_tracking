# dvs_corner_tracking
Corner tracking based on Scaramuzza's corner detection for DVS

In order for this program to work you'll have to have the corner detector installed from:
https://github.com/uzh-rpg/rpg_corner_events

Then you can build the tracker by cd'ing into it like this:
cd ~/catkin_ws/src/rpg_corner_events

source the required files:
source ~/catkin_ws/devel/setup.bash

Build the detector and launch it:
catkin build corner_event_detector
roslaunch corner_event_detector bag.launch

This allows you to run the corner tracker on a bag file in a second terminal using:
cd bag_file_location
source ~/catkin_ws/devel/setup.bash
rosbag play bagfile.bag
