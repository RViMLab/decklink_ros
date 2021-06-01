# DeckLink ROS
Exposes the a IDeckLinkVideoInputFrame to ROS.

# Build
To build, download and install [Blackmagic Desktop Video](https://www.blackmagicdesign.com/uk/support/) and [Desktop Video SDK](https://www.blackmagicdesign.com/uk/support/). Do

```shell
source /opt/ros/melodic/setup.bash # or similar
catkin_make -DDECKLINK_SDK_DIR="path/to/Blackmagic_DeckLink_SDK"
```

# Launch
To laaunch the decklink ros node, build and source the package, then run
```shell
roslaunch decklink_ros decklink_ros_node.launch
```

# Note
Code based on the [decklink_capture](https://github.com/andreasBihlmaier/decklink_capture) implementation.
