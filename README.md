# sim-to-real project

This project implements a vision-guided deep reinforcement learning framework to control a humanoid robot arm (THORMANG3) for balancing a ball on a tray in the real world. It uses an ONNX-trained PPO policy, visual tracking via OpenCV or YOLOv8, and ROS-based actuation through joint state publishing.

---

## Key Components

###  `infer_net.py`
Main control loop for sim-to-real inference:
- Loads trained PPO policy from ONNX
- Receives observations from camera and joint state
- Computes actions
- Publishes joint position commands to THORMANG3

###  `orange_ball_tracker.py`
HSV-based vision module:
- Detects the tray and orange ball via color segmentation
- Applies perspective transform
- Computes ball position and velocity on the tray in meters

###  `joint_state_observer.py`
ROS-based observer:
- Subscribes to `/robotis/present_joint_states`
- Collects joint position and velocity data
- Assembles part of the state vector

###  `publish_joint_state.py`
ROS publisher:
- Publishes joint positions to `/robotis/set_joint_states`
- Sends 4-DOF control actions to the real robot
- Includes torque control

###  `yolo_v8_inference_node.py`
Alternative object detection node:
- Uses ArUco markers to detect tray corners
- Applies YOLOv8 to detect the orange ball
- Computes and publishes ball position and velocity

---

##  Project Structure

sim2real/
├── infer_net.py # Main RL inference and control loop
├── orange_ball_tracker.py # HSV vision tracking
├── joint_state_observer.py # ROS joint state subscription
├── publish_joint_state.py # Joint command publisher
├── yolo_v8_inference_node.py # Optional YOLOv8 + ArUco tracking node
├── README.md


---

## How to Run

1. **Prepare the robot:**
   - Make sure THORMANG3 is powered and ready
   - Launch ROS and camera nodes

2. **Run vision module (choose one):**

   HSV-based:
   ```bash
   python3 orange_ball_tracker.py Or YOLOv8-based:


+------------------------+         +--------------------+
|   Camera               | ---->   |  Ball Tracker       |
|  		         |         |  (HSV or YOLOv8)    |
+------------------------+         +--------------------+
                                            |
                                            v
+-----------------------+         +-------------------------+
| Joint State Observer  | ---->   |   RL Inference (ONNX)   |
| /robotis/present...   |         |   PPO from Isaac Gym    |
+-----------------------+         +-------------------------+
                                            |
                                            v
+-----------------------+         +-------------------------+
| PublishJointState     | <----   |   Scaled Actions        |
| /robotis/set...       |         |   + Torque Control      |
+-----------------------+         +-------------------------+





