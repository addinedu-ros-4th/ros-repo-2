# AMR Logistics Automation System
<div align="center">
    <img src="https://github.com/addinedu-ros-4th/ros-repo-2/assets/118419026/dd1da530-6c66-4815-94ef-1b23f385e5c4" width="700" height="300">
</div>


<div align="center">

| [![Video 4](https://img.youtube.com/vi/72EWf1t_NVw/sddefault.jpg)](https://youtu.be/72EWf1t_NVw) |
|:---:|
| [최종 시연 영상](https://youtu.be/72EWf1t_NVw) |

</div>


## Table of Contents
  * [1. 🤖Project Introduction](#1-project-introduction)
    + [1.1. Project Overview](#11-project-overview)
    + [1.2. Project Position](#12-project-position)
    + [1.3. Tech Stack](#13-tech-stack)
    + [1.4. Reference](#14-reference)
  * [2. 📋Role Contributions](#2-role-contributions)
    + [2.1. System Architecture](#21-system-architecture)
    + [2.2. Hardware Architecture](#22-hardware-architecture)
    + [2.3. Software Architecture](#23-software-architecture)
    + [2.4. 3D Pose Estimation](#24-3d-pose-estimation)
      - [2.4.1. Camera Calibration](#241-camera-calibration)
      - [2.4.2. ArUCo Navigation](#242-aruco-navigation)
    + [2.5. Human Following](#25-human-following)
  * [3. ✅Prerequisite](#3-prerequisite)
  * [4. ⏩Usage](#4-usage)


## 1. 🤖Project Introduction
**Duration: 2024.04.17 - 2024.06.13**

### 1.1. Project Overview
- Utilizing forklift frame-based autonomous driving robots in logistics processes such as inbound, outbound, and collection.
- Implementing a multi-robot control system to enhance the efficiency of logistics operations.
- Optimizing logistics center operations through Human-Robot Interaction (HRI).
- Implementing systems for inbound product registration, shelf inventory management, and consumer order processing.

<img src="https://github.com/AUTO-KKYU/AMR-Logistics-Automation/assets/118419026/25960a9e-af9d-46dc-9f70-1cfc5d5e6168">

### 1.2. Project Position
<table>
  <thead>
    <tr>
      <th style="text-align:center;">Name</th>
      <th style="text-align:center;">Classification</th>
      <th style="text-align:center;">Role</th>
    </tr>
  </thead>
  <tbody>
    <tr>
      <td style="text-align:center;">YONG TAK SONG</td>
      <td style="text-align:center;">Team Leader</td>
      <td>- Robot System Integration and Control <br> - Robot Mechanism Design <br> - SLAM & NAV <br> - Architecture and Algorithm Design <br> - Setting up Driving Environment <br> - Detailed Feature Improvement and Code Integration </td>
    </tr>
    <tr>
      <td style="text-align:center;">DONG GYU KIM</td>
      <td style="text-align:center;">Team Member</td>
      <td>- ArUco Navigation <br> - DL based Human Following Robot <br> - Analysis User/System Requirement <br> - Architecture and Algorithm Design <br> - Setting up Driving Environment <br> - Confluence Management </td>
    </tr>
    <tr>
      <td style="text-align:center;">KYEOM HEE YOU</td>
      <td style="text-align:center;">Team Member</td>
      <td>- SLAM <br> - Admin GUI <br> - Database Construction
 <br> - Setting up Driving Environment  </td>
    </tr>
    <tr>
      <td style="text-align:center;">JEA HYUCK LEE</td>
      <td style="text-align:center;">Team Member</td>
      <td>- SLAM & NAV <br> - Path Planning Algorithm Design <br> - Multi-Robot Control System Implementation <br> - Detailed Feature Improvement and Code Integration <br> - Setting up Driving Environment  </td>
    </tr>
    <tr>
      <td style="text-align:center;">HA RIN JANG</td>
      <td style="text-align:center;">Team Member</td>
      <td>- SLAM <br> - Admin GUI <br> - User GUI
 <br> - Database Construction </td>
    </tr>
    <tr>
      <td style="text-align:center;">GA EUN CHOI</td>
      <td style="text-align:center;">Team Member</td>
      <td>- Multi-Robot Task Handling <br> - Communication Interface and Protocol Design <br> - Robot Communication Server Construction <br> - GitHub and Jira Management </td>
    </tr>
  </tbody>
</table>




### 1.3. Tech Stack
||||
|:---:|:---|:---|
|**Develop EnV**|<img src="https://img.shields.io/badge/Ubuntu-E95420?style=for-the-badge&logo=Ubuntu&logoColor=white"> <img src="https://img.shields.io/badge/VISUAL STUDIO CODE-007ACC?style=for-the-badge&logo=VisualStudioCode&logoColor=white">|
|**TECH**|<img src="https://img.shields.io/badge/python-3670A0?style=for-the-badge&logo=python&logoColor=ffdd54"> <img src="https://img.shields.io/badge/C++-00599C?style=for-the-badge&logo=cplusplus&logoColor=white"> <img src="https://img.shields.io/badge/ros2-%2322314E?style=for-the-badge&logo=ros&logoColor=white"> <img src="https://img.shields.io/badge/numpy-%23013243.svg?style=for-the-badge&logo=numpy&logoColor=white"> <img src="https://img.shields.io/badge/OpenCV-5C3EE8?style=for-the-badge&logo=opencv&logoColor=white"> <img src="https://img.shields.io/badge/PyQt5-%23217346.svg?style=for-the-badge&logo=Qt&logoColor=white"> <img src="https://img.shields.io/badge/mysql-4479A1.svg?style=for-the-badge&logo=mysql&logoColor=white"> |
|**H/W**|<img src="https://img.shields.io/badge/-RaspberryPi 4-C51A4A?style=for-the-badge&logo=Raspberry-Pi"> <img src="https://img.shields.io/badge/-Arduino Mega-00979D?style=for-the-badge&logo=Arduino&logoColor=white">|
|**COMMUNICATION**|<img src="https://img.shields.io/badge/confluence-%23172BF4.svg?style=for-the-badge&logo=confluence&logoColor=white"> <img src="https://img.shields.io/badge/jira-%230A0FFF.svg?style=for-the-badge&logo=jira&logoColor=white"> <img src="https://img.shields.io/badge/Slack-4A154B?style=for-the-badge&logo=Slack&logoColor=white">  <img src="https://img.shields.io/badge/github-181717?style=for-the-badge&logo=github&logoColor=white">|

### 1.4. Reference
- Presentation : https://docs.google.com/presentation/d/1zWYl33Bm2CBIjSyX9Pe78l9VBHL1Ysp2sf91z0QALls/edit?usp=drive_link
- Additional Information, Contact Email: dknjy3313@gmail.com

## 2. 📋Project Design

### 2.1. System Architecture

<img src="https://github.com/AUTO-KKYU/AMR-Logistics-Automation/assets/118419026/9a9e5e40-1a1d-45e5-b85e-094c319c2f4b">

### 2.2. Hardware Architecture
<img src= "https://github.com/AUTO-KKYU/AMR-Logistics-Automation/assets/118419026/5f911581-c86e-44b8-a48b-21adc2f979da">

### 2.3. Software Architecture
<img src="https://github.com/AUTO-KKYU/AMR-Logistics-Automation/assets/118419026/e0b8a28a-6034-4173-8754-e2a6924e7939">

## 3. 💡Technology

### 3.1. 3D Pose Estimation

#### 3.1.1. Camera Calibration
<img src= "https://github.com/AUTO-KKYU/AMR-Logistics-Automation/assets/118419026/ed869dcf-58b9-48e1-8ad6-d9a389a769ce">

#### 3.1.2. ArUCo Navigation
<img src= "https://github.com/AUTO-KKYU/AMR-Logistics-Automation/assets/118419026/acb06277-3130-40a1-b1d1-ec2a3675a078">

### 3.2. Human Following
<img src="https://github.com/AUTO-KKYU/AMR-Logistics-Automation/assets/118419026/eeb94530-05e6-4cb7-ba7e-3fccffb6b9e1">

### 3.3.1 Path Planning
<img src="![Screenshot from 2024-06-18 13-37-49](https://github.com/addinedu-ros-4th/ros-repo-2/assets/61307553/18116ef2-704d-460a-a252-a59d1cb2c58b)">

## 4. ✅Prerequisite

**How to install ROS2 Humble on PC [Ubuntu 22.04]**
- Follow the guidelines within the site
    - environment : https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html
    - dev tools : https://docs.ros.org/en/humble/Installation/Alternatives/Ubuntu-Development-Setup.html#install-development-tools-and-ros-tools
    - colcon : https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html#install-colcon

**ArUCo version on OpenCV**
```sh
pip install opencv-contrib-python==4.7.0.68 opencv-python==4.7.0.68
```

**Raspberry pi 4 GPIO settings**
```sh
# GPIO 그룹 설정
sudo groupadd gpio
# 사용자를 gpio 그룹에 추가
sudo usermod -aG gpio kkyu_rasp
# udev 규칙 파일 설정
sudo nano /etc/udev/rules.d/99-gpio.rules
# udev 규칙 내용 작성
SUBSYSTEM=="gpio*", PROGRAM="/bin/sh -c '\\
chown -R root:gpio /sys/class/gpio && chmod -R 770 /sys/class/gpio;\\
chown -R root:gpio /sys/devices/virtual/gpio && chmod -R 770 /sys/devices/virtual/gpio;\\
chown -R root:gpio /sys$devpath && chmod -R 770 /sys$devpath\\
'"
# udev 규칙 적용
sudo udevadm control --reload-rules
sudo udevadm trigger
sudo reboot now
```

**Raspberry pi 4 permission settings**
```sh
sudo chown root:gpio /dev/gpiomem
sudo chmod g+rw /dev/gpiomem
sudo chmod a+rw /dev/i2c-* 
```

**Raspberry pi 4 camera settings**
```sh
sudo chmod 777 /dev/video0
```

**Websocket Dependency**
```sh
pip install websocket-client
pip install PyQt5 websocket-client
```

**Others**
```sh
sudo apt install ros-humble-test-msgs
```

**.bashrc settings**
```sh
echo "Humble is activated.!"
source /opt/ros/humble/setup.bash
echo "Minibot is activated.!"
source ~/ros-repo-2/robot/install/local_setup.bash
echo "Server is activated.!"
source ~/ros-repo-2/server/install/local_setup.bash
echo "ROS_DOMAIN_ID is set 213"  # 각 로봇에 대한 ID : 213, 214, 215
export ROS_DOMAIN_ID=213

sudo chmod 777 /dev/video0
sudo chown root:gpio /dev/gpiomem
sudo chmod g+rw /dev/gpiomem
sudo chmod a+rw /dev/i2c-*
```

## 5. ⏩Usage

**General Setup**

```sh
git clone https://github.com/AUTO-KKYU/AMR-Logistics-Automation.git

cd ros-repo-2/robot
colcon build
source ./install/local_setup.bash

cd ../server
colcon build
source ./install/local_setup.bash
```

*- Recommend: Use the SSH method for remote access*
- ex) ssh -X your_rasp_name@your_rasp_ip
  
**Robot 1/2/3 PC (Raspberry Pi PC)**
- Domain ID : 91 / 92 / 93

```sh
# bringup robot : arduino (encoder motor) + Raspberry (YD LIDAR)
ros2 launch minibot_bringup bringup_robot.launch.py

# Aruco & Step motor (fork lift) Node
ros2 launch robot_aruco aruco_detect.launch.py

# human following mode
ros2 launch human_following following.launch.py
```

**Robot 1/2/3 PC (LOCAL PC)**
```sh
export ROS_DOMAIN_ID = 91 / 92 / 93

# check ROS_DOMAIN_ID SETTINGS
printenv grep -i ROS_DOMAIN_ID

source /opt/ros/humble/setup.bash
source ~/.bashrc

# Robot Controller
ros2 run robot_controller robot_controller

# slam toolbox + nav2 package
ros2 launch minibot_navigation2 bringup_launch.py map:=`ros2 pkg prefix minibot_navigation2`/share/minibot_navigation2/maps/re_map.yaml

# rviz visualization
rviz2 -d `ros2 pkg prefix minibot_navigation2`/share/minibot_navigation2/rviz/nav2_view.rviz
```

**Server**
```sh
export ROS_DOMAIN_ID = 90
source /opt/ros/humble/setup.bash
source ~/.bashrc

# websocket bridge
ros2 launch rosbridge_server rosbridge_websocket_launch.xml

# ros2 domain bridge
ros2 run domain_bridge domain_bridge ~/your_path/server/config/bridge_config.yaml

# task allocator server
ros2 launch task_manager task_manager_launch.py
```

**Consumer GUI**
```sh
(venv) (root folder) python3 gui/consumer/src/ui_main.py
```

**Manager GUI**
```sh
(inside gui folder) ros2 run manager.pkg manager_main.py
```

**Robot test Service**
```sh
# aruco control service call
# location : I1,I2,I1 / O1,O2,O3 / P1,P2,P3 / R1,R2 / A1,A2 / B1,B2 / C1,C2
# direction : forward, backward
ros2 service call /aruco_control task_msgs/srv/ArucoCommand "{location: O1, direction: forward}"

# step control service call
# floor : 1,2
# direction : up, down
ros2 service call /step_control task_msgs/srv/StepControl "{floor: 2, direction: up}"

# camera control service call
# data : True / False
ros2 service call /camera_control std_srvs/srv/SetBool "{data: True}"
```

**Server test Service**
```sh
# waypoint move service call
# location : I1,I2,I1 / O1,O2,O3 / P1,P2,P3 / R1,R2 / A1,A2 / B1,B2 / C1,C2
# lift : Up, Down
ros2 service call /allocate_task_92 task_msgs/srv/AllocateTask "{location: "I2", lift: 'Up'}"
```

| [![Video 1](https://img.youtube.com/vi/8Qt1J9MkPb4/sddefault.jpg)](https://youtu.be/8Qt1J9MkPb4) | [![Video 2](https://img.youtube.com/vi/RSHB1A6I8J8/sddefault.jpg)](https://youtu.be/RSHB1A6I8J8) |
|:---:|:---:|
| <div align="center">[ArUCo 3D Pose Estimation](https://youtu.be/8Qt1J9MkPb4)</div> | <div align="center">[Human Following Robot](https://youtu.be/RSHB1A6I8J8)</div> |
