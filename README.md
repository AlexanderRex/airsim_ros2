# Airsim Pack

## Prerequisites:
- Git
- C++ Compiler (e.g., GCC)
- Development tools (build-essential, clang-8, etc.)
- PX4 Sitl
- ROS2

### Cloning and Building Unreal Engine 4.27 on Linux
```bash
git clone https://github.com/EpicGames/UnrealEngine.git -b 4.27
```

### Install and build UnrealEngine:
```bash
cd UnrealEngine
./Setup.sh
./GenerateProjectFiles.sh
make
```

### Install Airsim:
```bash
git clone https://github.com/Microsoft/AirSim.git
cd AirSim
./setup.sh
./build.sh
```

### Install PX4 Sitl

Use following instructions:
https://microsoft.github.io/AirSim/px4_sitl/

### How to launch simulation

### Launch Airsim
```bash
./UnrealEngine/Engine/Binaries/Linux/UE4Editor ~/AirSim/Unreal/Environments/Blocks/Blocks.uproject
```
If you want to change your simulation settings specify setting.json and put it in ~/Documents 

### Launch PX4
```bash
cd PX4/PX4-Autopilot/
make px4_sitl_default none_iris
```
### Launch ROS2
```bash
cd AirSim/ros2
colcon build
source ./install/setup.bash
```
After that you will be able to use launch files to use your rosnodes

## Useful links
Aruco QR navigation:
https://github.com/meurissemax/autonomous-drone

Simple obstacle avoidance
https://github.com/Ironteen/Obstacle-Avoidance-in-AirSim


