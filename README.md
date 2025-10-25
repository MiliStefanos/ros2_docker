# Robotics Technical Assessment

This project it's a simple ros2 project running in an ubuntu docker image.

It uses **ROS 2 Humble** and includes the **Eigen**, **yaml-cpp**, and **rviz_visual_tools** packages. 

## Build and Run

In order to build the project, run from root folder

```bash
docker compose build
```

In order to run
```bash
docker compose up -d
```

## Build and run linear_algebra_service pkg

Build pkg with colcon
```bash
colcon build --packages-select linear_algebra_service
```

Source workspace
```bash
source install/setup.bash
```

Run server node
```bash
ros2 run linear_algebra_service server
```

Run client node
```bash
ros2 run linear_algebra_service client
```