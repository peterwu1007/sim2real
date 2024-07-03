#!/home/mpc/anaconda3/envs/yolo/bin/python

from init_ros_node import init_ros_node
import rospy  # 确保导入 rospy
import numpy as np
import onnx
import onnxruntime as ort
from joint_state_observer import JointStateObserver
from yolo_v8_inference_node import YOLOv8InferenceNode  
import time

class Test:
    def __init__(self):
        init_ros_node('test_node')
        onnx_model = onnx.load("/home/mpc/catkin_ws/src/peter/src/exports/test.onnx")
        onnx.checker.check_model(onnx_model)
        self.loaded_model = ort.InferenceSession("/home/mpc/catkin_ws/src/peter/src/exports/test.onnx")

        self.yolov8_node = YOLOv8InferenceNode()
        self.joint_state_observer = JointStateObserver()

        self.turned_on = False

    def build_obs(self, dof_positions, dof_velocities, ball_positions, ball_linvels):
        obs = np.concatenate((dof_positions, dof_velocities, ball_positions, ball_linvels), axis=0)
        if len(obs) != 18:
            raise ValueError(f"Observation length is {len(obs)}, expected 18.")
        obs = np.reshape(obs, (1, len(obs)))
        return obs

    def infer(self, obs):
        return self.loaded_model.run(None, {"obs": obs.astype(np.float32)})[0]

    def run(self):
        rospy.loginfo("Waiting for data...")
        time.sleep(2)  

        while not self.joint_state_observer.get_observations()['position']:
            rospy.loginfo("Waiting for joint state data...")
            time.sleep(0.1)

        while not rospy.is_shutdown():
            ball_positions, ball_linvels = self.yolov8_node.get_ball_position_and_velocity()
            dof_positions = self.joint_state_observer.get_observations()['position']
            dof_velocities = self.joint_state_observer.get_observations()['velocity']

            # print(f"Ball positions: {ball_positions}, length: {len(ball_positions)}")
            # print(f"Ball linear velocities: {ball_linvels)}, length: {len(ball_linvels)}")
            # print(f"DOF positions: {dof_positions}, length: {len(dof_positions)}")
            # print(f"DOF velocities: {dof_velocities}, length: {len(dof_velocities)}")

            try:
                obs = self.build_obs(dof_positions, dof_velocities, ball_positions, ball_linvels)
                actions = self.infer(obs)
                actions_array = np.array(actions)

                print("Actions:", actions_array)
            except ValueError as e:
                print(e)

            time.sleep(0.1)  # 设定循环间隔时间，避免过高的CPU使用率

if __name__ == '__main__':
    try:
        test_instance = Test()
        test_instance.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Test node terminated.")
    except Exception as e:
        print(e)
