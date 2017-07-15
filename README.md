## Installation

Make sure turtlebot is installed:

```
sudo apt-get updates
sudo apt-get install ros-indigo-turtlebot ros-indigo-turtlebot-apps ros-indigo-turtlebot-interactions ros-indigo-turtlebot-simulator ros-indigo-kobuki-ftdi ros-indigo-rocon-remocon ros-indigo-rocon-qt-library ros-indigo-ar-track-alvar-msgs
```

To install this package, do the following:
```
cd catkin/src
git clone https://github.com/Abdullah-Abduldayem/turtlebot_mall_navigation.git
```

## Run
To launch the simulator, open a terminal and type the following:

```
roslaunch turtlebot_mall_navigation simulator.launch
```

The turtlebot appears as a red square, and obstacles are blue squares. The obstacles can be moved by clicking and dragging them to simulate dynamic obstacles.

To command the turtlebot to move to a shop, open a new terminal and run the following:
```
rosrun turtlebot_mall_navigation store_button.py <num>
```

where `num` is a value between 1 and 12, inclusive. This command can be executed multiple times consequitively. Stores are visited in a first-come-first-served manner.

