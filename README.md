# DeckLink ROS
Exposes the a IDeckLinkVideoInputFrame to ROS.

## Build
To build, download and install [Blackmagic Desktop Video](https://www.blackmagicdesign.com/uk/support/) and [Desktop Video SDK](https://www.blackmagicdesign.com/uk/support/). Do

```shell
colcon build --cmake-args -DDECKLINK_SDK_DIR="path/to/Blackmagic_DeckLink_SDK"  # e.g. -DDECKLINK_SDK_DIR="$HOME/Downloads/Blackmagic_DeckLink_SDK_12.2.2/Blackmagic DeckLink SDK 12.2.2/Linux"
```

## Launch
To laaunch the decklink ros node, build and source the package, then run
```shell
source install/setup.bash && \
ros2 launch decklink_ros decklink_ros_node.launch.py
```

## Note
Code based on the [decklink_capture](https://github.com/andreasBihlmaier/decklink_capture) implementation.
