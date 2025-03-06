# Tracailer

This repository is for the paper, "Tracailer: An Efficient Trajectory Planner for Tractor-Trailer Robots in Unstructured Environments".

## Quick Start

### Step One:

Install the [ros](https://wiki.ros.org/ROS/Installation) and the requirements below.

**ros dependence**: (use ros noetic with Ubuntu20.04 as an example)

```
sudo apt install ros-noetic-tf2-geometry-msgs
sudo apt install ros-noetic-ackermann-msgs
sudo apt install libompl-dev
```

**casadi for mpc controller:**

Go to the [website of casadi](https://github.com/casadi/casadi) and install casadi. For example, you can:

```
git clone https://github.com/casadi/casadi.git
cd casadi
```

📢**Important!**: Open the `CMakeLists.txt` and set the `WITH_LAPACK_DEF` and `WITH_QPOASES_DEF` from `OFF` to `ON`, then

```
mkdir build && cd build
cmake ..
make
sudo make install
```

**NOTE:** We may have forgotten other dependencies 😟, sorry!

### Step Two:

Build the project: (you can change the trailer num defined in `src/planner/CMakeLists.txt` by changing the definition of `TRAILER_NUM`)

```
git clone https://github.com/ZJU-FAST-Lab/tracailer.git
cd tracailer
catkin_make
```

### Step Three:

run the project:

```
source devel/setup.bash
roslaunch planner run_all.launch
```

When you see the robot in the Rviz like below, you can use `2D Pose Estimate` to trigger planning.

https://github.com/user-attachments/assets/bf2bb8f0-ab63-451f-96da-7b097f58948a
