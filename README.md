# 🛰️ Naima u-blox GPS ROS2 Driver

A lightweight and efficient ROS 2 Humble driver for interfacing with **u-blox GPS modules** over serial communication.  
Tested on **Ubuntu 22.04** with the **u-blox M8030** chipset.

---

## ✨ Features

- 📡 Publishes GPS data on `/fix` using `sensor_msgs/msg/NavSatFix`
- ⚙️ Automatically initializes and configures the u-blox GPS module
- 🤖 Fully compatible with ROS 2 Humble
- 💬 Simple launch system for easy deployment

---

## 🛠️ Installation

Follow the steps below to install and build the package in your ROS 2 workspace:

```bash
# Create a new workspace
mkdir -p ~/ublox_ws/src
cd ~/ublox_ws/src

# Clone the repository
git clone https://github.com/men3m-4/naima_ublox_gps.git

# Build the workspace
cd ..
colcon build

# Source the environment
source install/setup.bash
```
## 🚀 Usage

### 🔧 Basic Launch

To launch the u-blox GPS node:

```bash
ros2 launch ublox_gps ublox_gps_node_1-launch.py
```
## 🛠️ Build Issues

If `colcon build` fails, try cleaning the workspace and rebuilding by following these steps:

```bash
rm -rf build install log
colcon build
```

## 📡 Topics

The following topics are published by the node:

| Topic         | Type                                   | Description            |
|---------------|----------------------------------------|------------------------|
| `/fix`        | `sensor_msgs/msg/NavSatFix`            | Latitude, Longitude, Altitude |
| `/gps_status` | `diagnostic_msgs/msg/DiagnosticArray`  | GPS module health and diagnostics |


## 🖼️ Sample Output

When the node runs successfully, it outputs messages similar to the following:

```bash
[ublox_gps_node-1] [INFO]: GPS module initialized
[ublox_gps_node-1] [INFO] Publishing fix:
  latitude: 29.9821
  longitude: 31.2345
  altitude: 45.2


