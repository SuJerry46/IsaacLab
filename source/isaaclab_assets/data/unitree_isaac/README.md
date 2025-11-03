# Unitree Isaac Sim Package

This repository contains a package for simulating Unitree robots in NVIDIA Isaac Sim.

## Start Unitree G1 Extension

1. Open the Unitree G1 USD file. You have two options:
   - **Standard version:** `unitree_isaac/usd/g1_joint.usd`
   - **With Inspire Hand:** `unitree_isaac/usd/g1_joint_inspire_hand/g1_joint_inspire_hand_stage.usd`
2. In the menu, select **Windows -> Extensions**.
3. Click the dropdown in the top right and select the **Settings** option.
4. Click the green **+** button in the **Extension Search Path**.
5. Type in the full URL `/path_to/unitree_isaac/extension`.
6. Click **THIRD PARTY** and enable **UNITREE.EXTENSION.G1**.

## Articulation Control 
### G1 Motor Control and Camera Feed

This package provides two control interfaces for G1 motor control and camera feed:

1. **FastAPI Interface**: A web-based API for controlling the robot and accessing camera streams.
2. **ROS2 Interface**: A ROS2-based interface for integrating with robotic applications and accessing motor and camera data.

### Provided Interfaces

The package offers the following data interfaces:

- **Joint Current State**: Access the current state of the robot's joints, including position, velocity, and effort.
- **Target Joint State**: Specify and monitor the desired target state for the robot's joints.
- **RGB Image Feed**: Retrieve real-time RGB camera images from the robot's perspective.
- **Depth Image Feed**: Access depth images for understanding the robot's environment in 3D.

### ROS2 Interface Details

The ROS2 interface provides the following topics for communication:

- **Subscribed Topics**:
   - **`isaac_sim/command_joint_states`** (`sensor_msgs/JointState`): Subscribes to target joint states for controlling the robot.

- **Published Topics**:
   - **`isaac_sim/joint_states`** (`sensor_msgs/JointState`): Publishes the current joint states of the robot.
   - **`isaac_cam/head/rgb`** (`sensor_msgs/Image`): Publishes real-time RGB images from the head camera.
   - **`isaac_cam/head/depth`** (`sensor_msgs/Image`): Publishes depth images from the head camera.
   - **`isaac_cam/right_hand/rgb`** (`sensor_msgs/Image`): Publishes real-time RGB images from the right hand camera.
   - **`isaac_cam/right_hand/depth`** (`sensor_msgs/Image`): Publishes depth images from the right hand camera.
   - **`isaac_cam/left_hand/rgb`** (`sensor_msgs/Image`): Publishes real-time RGB images from the left hand camera.
   - **`isaac_cam/left_hand/depth`** (`sensor_msgs/Image`): Publishes depth images from the left hand camera.

### FastAPI WebSocket Interface

The FastAPI interface provides WebSocket endpoints for real-time communication with the robot. Below are the available WebSocket endpoints and their data formats:
#### WebSocket Server Configuration

- **Host**: `localhost`
- **Port**: `8787`
- **Example**: `ws://localhost:8787/isaac/ws/joints/action`

### Client Example

For an example of how to communicate with the WebSocket endpoints, refer to the client script located at:

   `extension/unitree.extension.g1/unitree_extension_g1_python/modules/fastapi/socket_test.py`

   1. **`/isaac/ws/joints/action`**: Send target joint positions to control the robot.

      **JSON Format**:
      ```json
      {
          "name": ["joint1", "joint2", "joint3"],
          "position": [1.0, 0.5, -0.5],
          "velocity": [0.0, 0.0, 0.0],
          "effort": [0.0, 0.0, 0.0],
          // "velocity": None,
          // "effort": None,
      }
      ```
      <details>
      <summary>Click to expand JSON example</summary>

      ```json
      {
         "name": [
            "left_hip_pitch_joint",
            "right_hip_pitch_joint",
            "waist_yaw_joint",
            "left_hip_roll_joint",
            "right_hip_roll_joint",
            "waist_roll_joint",
            "left_hip_yaw_joint",
            "right_hip_yaw_joint",
            "waist_pitch_joint",
            "left_knee_joint",
            "right_knee_joint",
            "left_shoulder_pitch_joint",
            "right_shoulder_pitch_joint",
            "left_ankle_pitch_joint",
            "right_ankle_pitch_joint",
            "left_shoulder_roll_joint",
            "right_shoulder_roll_joint",
            "left_ankle_roll_joint",
            "right_ankle_roll_joint",
            "left_shoulder_yaw_joint",
            "right_shoulder_yaw_joint",
            "left_elbow_joint",
            "right_elbow_joint",
            "left_wrist_roll_joint",
            "right_wrist_roll_joint",
            "left_wrist_pitch_joint",
            "right_wrist_pitch_joint",
            "left_wrist_yaw_joint",
            "right_wrist_yaw_joint",
            "L_index_proximal_joint",
            "L_middle_proximal_joint",
            "L_pinky_proximal_joint",
            "L_ring_proximal_joint",
            "L_thumb_proximal_yaw_joint",
            "R_index_proximal_joint",
            "R_middle_proximal_joint",
            "R_pinky_proximal_joint",
            "R_ring_proximal_joint",
            "R_thumb_proximal_yaw_joint",
            "L_index_intermediate_joint",
            "L_middle_intermediate_joint",
            "L_pinky_intermediate_joint",
            "L_ring_intermediate_joint",
            "L_thumb_proximal_pitch_joint",
            "R_index_intermediate_joint",
            "R_middle_intermediate_joint",
            "R_pinky_intermediate_joint",
            "R_ring_intermediate_joint",
            "R_thumb_proximal_pitch_joint",
            "L_thumb_intermediate_joint",
            "R_thumb_intermediate_joint",
            "L_thumb_distal_joint",
            "R_thumb_distal_joint"
         ],
         "position": [
            0.0007804749184288085,
            0.0007804736378602684,
            -0.00011832222662633285,
            1.670047095103655e-05,
            -1.6700041669537313e-05,
            0.0007804546621628106,
            -3.440240163854469e-07,
            3.446313598942652e-07,
            0.0022809975780546665,
            2.5935507437679917e-05,
            2.5936342353816144e-05,
            -0.6979290843009949,
            -0.031067820265889168,
            5.302013505570358e-06,
            5.301443252392346e-06,
            0.8581474423408508,
            -1.3888615369796753,
            4.8086390052048955e-06,
            -4.80880544273532e-06,
            -0.9335179328918457,
            0.7499045729637146,
            -0.7162644267082214,
            -0.7104516625404358,
            -1.1000251770019531,
            0.923369824886322,
            0.20002500712871552,
            -0.3823974132537842,
            1.3620049953460693,
            -0.9236316084861755,
            -1.5722089301561937e-05,
            -2.4331970962521154e-06,
            6.145739007479278e-06,
            9.160394256468862e-06,
            0.0004803100600838661,
            0.0010244321310892701,
            -9.345768376078922e-06,
            -3.408647535252385e-05,
            -9.384474651596975e-06,
            0.0010255284141749144,
            -0.26033976674079895,
            -0.1477358341217041,
            -0.046479806303977966,
            -0.1064087525010109,
            0.004673686344176531,
            -0.3331564664840698,
            -0.2157028615474701,
            -0.10289596021175385,
            -0.15516890585422516,
            0.0034224039409309626,
            0.9600016474723816,
            0.9599988460540771,
            1.0495456457138062,
            0.9945629835128784
         ]
         "velocity": None,
         "effort": None
      }
      ```

      </details>

   2. **`/isaac/ws/joints/current`**: Retrieve the current joint positions of the robot.

      **JSON Format**:
      ```json
      {
          "name": ["joint1", "joint2", "joint3"],
          "position": [1.2, 0.6, -0.4],
          "velocity": [0.0, 0.0, 0.0],
          "effort": [0.0, 0.0, 0.0],
      }
      ```

   3. **`/isaac/ws/images/head`**: Access real-time image data from the robot's head cameras.
   4. **`/isaac/ws/images/left_hand`**: Access real-time image data from the robot's left hand cameras.
   5. **`/isaac/ws/images/right_hand`**: Access real-time image data from the robot's right hand cameras.

      **JSON Format**:
      ```json
      {
          "rgb_image": "base64_encoded_rgb_image_data",
          "depth_image": "base64_encoded_depth_image_data"
      }
      ```

   These WebSocket endpoints enable efficient and real-time communication for controlling the robot and accessing its sensor data.


## Navigation Isaac environment

Follow the steps below to set up and run the simulation.

### Run unitree isaac sim

1. Open Isaac Sim:
   ```bash
   cd /isaac-sim
   ./runapp.sh
   ```

2. Load the simulation file:
   - Open the file `g1_static.usd` in Isaac Sim.

### Topics Provided

- **/odom**: Provides odometry data for tracking the robot's position and movement.
- **/point_cloud**: Publishes point cloud data for perception environment
- **/cmd_vel**: Allows velocity control of the robot using linear and angular velocity commands.

### Usage

Once the simulation is loaded, you can interact with the Unitree robot model in the Isaac Sim environment. Use the provided tools and scripts to test and develop your applications.