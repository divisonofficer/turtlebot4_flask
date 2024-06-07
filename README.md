# Robot-Driven Image Capture System



## Hardwares
Turtlebot4
Ubuntu22.04 Laptop
JAI Fusion Camera
ELL14 Rotation Mount
Glass Polarizer



## Software Structure

### ReactApp
UI/UX

### ClientApp
ROS Middleware

#### CaptureApp
Camera Capture Scenario Manager
Polarization Capture System

#### Ell
Polarizer RotationStage Controller

#### Jai
JAI ebus Abstract layer for ROS Humble

#### Jai_Bridge
Middleware between HTTP and ROS for JAI camera control

#### laptop_monitor
Middleware between HTTP and Ubuntu system feature of Laptop

#### Slam
Middleware between HTTP and ROS Slam system



### RosApp
Turtlebot4 ROS System
