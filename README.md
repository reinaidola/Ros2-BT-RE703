# Behavior Tree using Groot2
---

Team member:
- Nama - NIM
- Nama - NIM

the robot task is to move in a squere using 4 checkpoint with a distance about 3 meter

---

## 🧠 Behavior Tree Design (Groot2)

![Behavior Tree Diagram](src/ratna_control_pkg/assets/control_bt_gaze.svg)

---

## System Requirements

- Ubuntu 22.04  
- ROS 2 (Humble recommended)
- Gazebo classic

---

## Install behaviortree-cpp-v3

```
sudo apt update
sudo apt install ros-humble-behaviortree-cpp-v3
```

## Install Groot2

install the Groot2 using this link https://www.behaviortree.dev/groot

## Clone this repo
```
git clone https://github.com/MarcellinoAcel/ratna_ws.git 
cd ratna_ws
source install/setup.bash
colcon build
```

## run the orb_slam
```
cd ~/ratna_ws
```
```
./ratna_run.sh
```