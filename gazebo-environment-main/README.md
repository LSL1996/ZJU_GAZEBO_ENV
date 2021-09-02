# 基于Gazebo的单机全链路仿真
用于项目所需,整理以下配置环境以避免重复劳动

## Overview
**Gazebo**使用的模型配置环境:[rotors_simulator](https://github.com/ethz-asl/rotors_simulator)

**Planner**:[ego-planner-swarm](https://github.com/ZJU-FAST-Lab/ego-planner-swarm),修改了traj_serveru以适应Gazebo的控制接口,规划时只启动了单机

**vio**:[VID-Fusion](https://github.com/ZJU-FAST-Lab/VID-Fusion),关闭了外力的估计,原理等于[VINS-Mono](https://github.com/HKUST-Aerial-Robotics/VINS-Mono)

## Compilation
**1. Gazebo环境配置**
    (Ubuntu 18.04 with ROS Melodic and Gazebo 9,其他的配置环境请参考[rotors_simulator](https://github.com/ethz-asl/rotors_simulator))
```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt update
sudo apt install ros-melodic-desktop-full ros-melodic-joy ros-melodic-octomap-ros ros-melodic-mavlink
sudo apt install python-wstool python-catkin-tools protobuf-compiler libgoogle-glog-dev ros-melodic-control-toolbox
sudo rosdep init
rosdep update
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
source ~/.bashrc
sudo apt install python-rosinstall python-rosinstall-generator build-essential
```
下载代码后
```
cd /src
catkin_init_workspace
cd ..
catkin init
rosdep install --from-paths src -i
```
更新之前安装的Gazebo版本
```
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
sudo apt update
sudo apt install gazebo9 gazebo9-* ros-melodic-gazebo-*
sudo apt upgrade
```
(**非必须步骤**)如果在进行下面所有的环境配置和编译过程之后,Gazebo无法正常打开,则运行下列命令
```
sudo apt-get remove ros-melodic-gazebo* gazebo*
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
sudo apt-get update
sudo apt-get install gazebo9 gazebo9-* ros-melodic-gazebo-*
sudo apt upgrade
```
**2.VIO环境配置**
安装[Ceres Solver](https://github.com/ceres-solver/ceres-solver),先配置依赖
```
sudo apt install libgoogle-glog-dev
sudo apt install libeigen3-dev
sudo apt install libatlas-base-dev
sudo apt install libsuitesparse-dev
sudo apt install libmetis-dev
sudo apt install liblapack-dev libcxsparse3.1.4 libgflags-dev libgtest-dev
sudo apt update
```
在home目录下对[Ceres Solver](https://github.com/ceres-solver/ceres-solver)源码进行编译
```
git clone https://github.com/ceres-solver/ceres-solver.git
cd /ceres-solver
mkdir build
cd build
cmake ..
make
sudo make install
```
在项目路径下,先对两个包进行编译
```
catkin build quadrotor_msgs mav_msg
```
**3.编译所有代码**
```
catkin build
```

## 使用说明
```
source devel/setup.bash
chmod 777 run.sh
./run.sh
```
开始运行后等待15s左右,rviz和gazebo陆续打开,无人机会先自动飞一段轨迹完成vins初始化,完成结束后,可以在rviz里按快捷键G,然后使用鼠标在地图中选择一个终点,触发轨迹规划,无人机自动起飞.

## 接口说明
1.**Planner**
- Subscribers
    位姿信息:/vid_estimator/imu_propagate
    深度图:/hummingbird/vi_sensor/camera_depth/depth/disparity
    终点:/hummingbird/goal
- Publishers
    控制:/hummingbird/command/trajectory

2.**VIO**
- Subscribers
    Imu信息:/hummingbird/ground_truth/imu
    图像信息:/hummingbird/vi_sensor/left/image_raw
- Publishers
    位姿信息:/vid_estimator/imu_propagate
