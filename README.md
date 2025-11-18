# Courswork 1 - 3D Reconstruction for Robotic Manipulation
## SfM Reconstruction + MoveIt2 Grasping Demonstration

This project implements a full sensing–perception–manipulation pipeline, integrating:

- Sparse Structure-from-Motion (SfM) 3D reconstruction  
- Point cloud processing and scaling  
- Point cloud visualization in RViz  
- Motion planning and execution using MoveIt2 and a UR5e robot  

This repository fulfills all requirements of the Sensing and Perception Coursework and provides a stable, reproducible demo.

---

# How to Run the System (3-Terminal Setup)

Before running any terminal, **each terminal must execute the following workspace setup**:

```bash
cd ~/ros2_ws
colcon build --packages-select moveit_demo
source install/setup.bash
````

---

## 🖥 Terminal 1 — Launch UR5e Driver

```bash
ros2 launch ur_robot_driver ur_control.launch.py \
    ur_type:=ur5e \
    use_mock_hardware:=true \
    robot_ip:=192.168.56.101
```

This launches the simulated UR5e hardware interface required for MoveIt2 planning.

---

## 🖥 Terminal 2 — Publish the Point Cloud for RViz

```bash
ros2 run pointcloud_pub publish_ply
```

Then open RViz and:

1. Click **Add**
2. Crear visualization by topic: **/sfm/pointcloud**
3. Set **Fixed Frame = base_link**

Once the point cloud is successfully displayed:

👉 **Press `Ctrl+C` to stop publishing.**

Stopping the publisher is recommended because continuous point cloud updates may interfere with MoveIt2 planning in Terminal 3.

---

## 🖥 Terminal 3 — Run the Grasping Demo

```bash
ros2 launch moveit_demo object_grasping_demo_updated.launch.py
```

This script performs the complete manipulation sequence:

1. Move to a pre-grasp joint-space posture
2. Pause briefly for demonstration clarity
3. Move vertically above the object (Cartesian pose)
4. Move to a predefined placement location
5. Return to the SRDF-defined `home` configuration

The full motion should be visible in RViz.

---

# Notes

* Ensure all three terminals run `source install/setup.bash`.
* Stop the point cloud publisher before starting the grasping demo.
* Rebuild (`colcon build`) whenever you modify a Python script inside the package.
* The PLY publisher automatically loads the latest processed SfM point cloud.

---

# Core Code Explanation

In my implementation, the overall sensing–perception–manipulation pipeline is built around three main code modules:

(1) SfM point cloud processing,  
(2) Point cloud publishing,  
(3) Robot grasping and motion planning.  
Below, I explain the role of each module, the algorithmic ideas behind it, and how I implemented it.

---

## 1. SfM Point Cloud Processing (`process_points.py`)

### Purpose
I use this script to clean, filter and scale the sparse 3D point cloud generated from the SfM reconstruction stage, ensuring the cloud is usable for robotic grasping.

### How It Works
1. **Automatically load the latest SfM output**  
   The script searches the `results` folder and selects the most recent `.npy` and `.json` files so the system always processes the newest reconstruction.

2. **Convert from homogeneous coordinates**  
   If SfM outputs (x, y, z, 1), I remove the last dimension to keep only valid 3D coordinates.

3. **Outlier filtering (95th percentile rejection)**  
   - Compute the centroid  
   - Compute distance of every point to the centroid  
   - Remove points above the 95th percentile  
   This suppresses noisy points and prevents incorrect grasp targets.

4. **Scale normalization**  
   - Compute the full x/y/z range  
   - Scale the cloud so its maximum extent becomes ~0.3 m  
   This ensures the reconstructed object fits into the UR5e workspace.

5. **Save processed results (PLY + NPY)**  
   I output a `.ply` file for RViz visualization and a `.npy` file for the grasping pipeline.

### Why This Matters
MoveIt planning requires the object to lie within a physically realistic workspace.  
Without filtering and scaling, IK may fail or produce unreachable trajectories.  
This step greatly improves stability and reproducibility.

---

## 2. Point Cloud Publisher (`publish_ply.py`)

### Purpose
This node publishes the processed point cloud into ROS2 so that I can verify the reconstruction and object position visually in RViz.

### How It Works
1. Automatically finds the most recent processed PLY file  
2. Translates the cloud forward into the robot’s reachable region (e.g., +0.4 m, +0.2 m)  
3. Converts the XYZ data into a `sensor_msgs/PointCloud2` message  
4. Publishes at 2 Hz to `/sfm/pointcloud`

### Why This Matters
Visualising the point cloud in RViz is part of the coursework requirements and is essential for verifying:
- The reconstruction is correct  
- The object lies inside the workspace  
- The centroid matches the intended grasping location  

The publisher also keeps the cloud aligned with the robot’s `base_link` frame.

---

## 3. Robot Grasping and Motion Planning  
(`object_grasping_demo_updated.py`)

### Purpose
This script implements a complete grasping sequence for the UR5e, including:
- Moving to a pre-grasp posture  
- Approaching the detected object position  
- Moving to a placement point  
- Returning to the `home` posture  

### How It Works

---

### (A) Pre-grasp Posture (Joint-Space Planning)

I first used RViz’s interactive markers to manually tune a stable and safe posture.  
I then stored the corresponding joint values:

- shoulder_pan_joint  
- shoulder_lift_joint  
- elbow_joint  
- wrist_1_joint  
- wrist_2_joint  
- wrist_3_joint  

In code, I construct a `RobotState` and create a joint constraint, commanding MoveIt to move the UR5e into this predictable, collision-free starting pose.

### Why Joint Space
Joint-space planning is highly stable and does not rely on IK.  
It guarantees that the robot begins the task from a valid configuration.

---

### (B) Computing the Target Point (From SfM Centroid)

I compute the centroid of the processed SfM point cloud:

```python
centroid = np.mean(points, axis=0)
```

Then I apply the same workspace translation that the publisher uses.
This produces the final grasping point in the robot’s `base_link` coordinate frame.

---
### (C) Cartesian Movements

For the actual grasping motion, I use Cartesian end-effector goals:

* Move vertically above the object
* Move to a fixed placement position

MoveItPy handles IK and trajectory planning automatically via:

```python
arm.set_goal_state(pose_stamped_msg=pose, pose_link="tool0")
```

### Why Cartesian Space

This produces intuitive movement, ensures the approach is vertical, and allows fine control of the end-effector path.

---

### (D) Return to Home

At the end of the sequence, I return the robot to the SRDF-defined `home` posture:

```python
arm.set_goal_state(configuration_name="home")
```

This resets the robot safely for the next run.

---

## 4. Combined Planning Strategy

My pipeline uses both **joint-space** and **Cartesian-space** planning:

### Joint Space

* Used for pre-grasp and home
* Always solvable, stable, not dependent on IK

### Cartesian Space

* Used for approaching the object and placing it
* Natural, precise, and human-interpretable motions

### Why Combine Both

This hybrid strategy ensures:

* High stability
* Intuitive trajectories
* Reduced failure rate
* Clearer demonstration for assessment

It also aligns well with real-world manipulation practices.

---

# Coursework Requirements Coverage

This implementation covers all required aspects:

* 3D reconstruction (SfM)
* Data preprocessing and filtering
* Sparse point cloud visualization
* Integration with ROS2 and MoveIt2
* Planning and execution of a complete manipulation pipeline
* Final demonstration video with RViz + robot execution

## Tested Environment

All commands and experiments were executed **inside an Ubuntu virtual machine running on macOS (Apple Silicon)**.

The project was tested with the following environment:

- OS: Ubuntu 24.04.3 LTS (noble)
- Kernel / Architecture: `Linux 6.8.0-87-generic`, `aarch64` (ARM64)
- ROS 2: Jazzy (environment variables: `ROS_VERSION=2`, `ROS_DISTRO=jazzy`)
- Python: 3.12.3
- Build tool: `colcon` (used with `colcon build`, `colcon info`)

You can quickly verify that your environment by running:

```bash
# OS version
lsb_release -a

# Kernel and architecture
uname -a

# Python version
python3 --version

# ROS 2 distro
printenv | grep -E 'ROS_DISTRO|ROS_VERSION'

# Check that the workspace packages are visible to colcon
cd ~/ros2_ws
colcon info


