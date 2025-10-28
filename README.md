# Robotics Technical Assessment

This project is a simple ROS 2 workspace running inside a Docker container based on **Ubuntu** and **ROS 2 Humble**.  
It includes dependencies such as **Eigen**, **yaml-cpp**, and **rviz_visual_tools**.

---

## Docker Usage

### Download repo
```bash
git clone git@github.com:MiliStefanos/ros2_docker.git
```

### Build the Docker image
```bash
docker compose build
```

### Run the container
```bash
docker compose up -d
```

### Enter the running container
```bash
docker exec -it ros2_container bash
```

---

## Clean the Workspace

Inside the container:
```bash
rm -rf build install log
```

If you get a **permission denied** error, fix it with:
```bash
sudo chown -R ros:ros /ros2_ws
```

---

## Build the Workspace

### Source ROS 2
```bash
source /opt/ros/humble/setup.bash
```

### Build all packages
```bash
colcon build
```

### Or build selected packages
```bash
colcon build --packages-select linear_algebra_msgs
colcon build --packages-select linear_algebra_service
```

---

## Configure Input

Edit the matrix size in the input configuration file:
```yaml
# File: /ros2_ws/src/linear_algebra_service/config/input.yaml
rows_num: 10  # Set m >= 3
```

---

## Run the Nodes

### Source the workspace
```bash
source install/setup.bash
```

### Terminal 1 — Run the client node
```bash
ros2 run linear_algebra_service client
```

### Terminal 2 — Run the server node
```bash
ros2 run linear_algebra_service server
```



---

## Functionality Summary

- The **client**:
  - Reads a matrix row count from a YAML file.
  - Generates a random matrix `A` (m×3) and vector `b` (m).
  - Sends them to the server via a ROS 2 service.

- The **server**:
  - Computes the least-squares solution to `Ax ≈ b`.
  - Applies a random 3D rotation and displacement.
  - Returns the transformed result to the client.

- The **client**:
  - Applies inverse transformation.
  - Prints the final computed solution.
  - Publishes it to the `/least_squares_solution` topic.

---

## Optional Tips

To inspect active topics:
  ```bash
  ros2 topic list
  ```

To echo the published solution:
  ```bash
  ros2 topic echo /least_squares_solution
  ```

---

## ur20 robot - rviz

Build and run container. (need the following to run rviz from container)
  ```bash
  docker compose build && docker compose up -d
  xhost +local:root
  docker run -it --env="DISPLAY" --env="QT_X11_NO_MITSHM=1" --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" ros2_docker-ros2_container
  ```

  Run launch file
  ```bash
  sudo chown -R ros:ros /ros2_ws
  rm -rf install/ log/ build/
  source /opt/ros/humble/setup.bash
  colcon build --packages-select ur20_display
  colcon build --packages-select ur_description
  source install/setup.bash
  ros2 launch ur20_display state_publisher.launch.py
  ```
