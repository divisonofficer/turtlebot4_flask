# Jai LoadMap

## Hardware info

Processor : Asus ROG Zephyrus G14 laptop (Ryzen R9 7000s, ubuntu 22.04)
Cameras : JAI Fusion 1600 camera (2 cameras, wired with LAN)


## Build

### prerequisite

install [ebus-sdk](https://www.pleora.com/machine-vision-automation/ebus-sdk/) at /opt/jai/ebus_sdk/Ubuntu-22.04-x86_64/lib
install [ros-humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html) to use ROS2 features.
install [turtlebot4-desktop](https://turtlebot.github.io/turtlebot4-user-manual/software/turtlebot4_desktop.html) to use sensor_msgs package
install [Google Proto](https://protobuf.dev/getting-started/cpptutorial/) to use data serialization

```
sudo apt install ros-humble-turtlebot4-desktop
```
install opencv4


### build sourcecode

```
cd JAI_ROS_bridge
cmake .
make
```

## Features
- auto detection of JAI fusion camera (with known Mac Address)
- Link 2 different camera and obtain packets simuletanously
- Emit Image output via ROS2 topic
- Take control signal via ROS2 topic
- Auto Configuration

## ROS Interaction

### Devices
- jai_1600_left
- jai_1600_right

### channel
- channel_0 (rgb)
- channel_1 (nir)

### topics
- /<device_name>/stream_trigger
  Open / Close Camera Stream
- /<device_name>/auto_exposure_hold_trigger
  Hold / Release Auto Exposure Value (For fixed exposure capture)
- /<device_name>/<channel>
Image Stream (CompressedImage)
For 8Bit image, Parse 1d Array to 2D array

For 10bit/ 12bit image, After parsing 1d Array, The high bits over 8, constitute single image, low bits under 8 consistute another image. and two image concated horizontally.

- /{device_name}/{channel}/device_param
Camera Node emit it's parameter when modification occured.
Compressed with Google Proto
- /{device_name}/{channel}/manual_configure
Camera Node read the parameter update requests from this topic.
Compressed with Google Proto




