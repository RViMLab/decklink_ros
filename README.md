# DeckLink ROS
Exposes the a IDeckLinkVideoInputFrame to ROS.

# Build
To build, download and install [Blackmagic Desktop Video](https://www.blackmagicdesign.com/uk/support/) and [Desktop Video SDK](https://www.blackmagicdesign.com/uk/support/). Do

```shell
source /opt/ros/melodic/setup.bash # or similar
catkin_make -DDECKLINK_SDK_DIR="path/to/Blackmagic_DeckLink_SDK"
```

# Launch

