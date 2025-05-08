
from init_ros_node import init_ros_node
import cv2
import rospy
import numpy as np
import onnx
import onnxruntime as ort
from joint_state_observer import JointStateObserver
import time
import csv
from publish_joint_state import Publisher
from orange_ball_tracker import OrangeBallTracker

class Test:
    def __init__(self):
        init_ros_node('infer_node')
        onnx_model = onnx.load("/home/mpc/catkin_ws/src/peter/src/exports/test3.onnx")
        onnx.checker.check_model(onnx_model)
        self.loaded_model = ort.InferenceSession("/home/mpc/catkin_ws/src/peter/src/exports/test3.onnx")

        self.joint_state_observer = JointStateObserver()
        self.tracker = OrangeBallTracker(
            camera_index=0,
            calibration_file="/home/mpc/catkin_ws/src/peter/src/C920hd_dist.p",
            lower_color_paper=np.array([110, 36, 128]),
            upper_color_paper=np.array([157, 255, 255]),
            # lower_color_ball=np.array([10, 150, 150]),
            # upper_color_ball=np.array([25, 255, 255]),
            lower_color_ball=np.array([102, 172, 41]),
        upper_color_ball=np.array([113, 241, 230]),
            real_board_width=0.25,
            real_board_height=0.25
        )

        self.joint_state_publisher = Publisher()
        self.joint_limits  = [
            {"lower_limit": -0.3, "upper_limit": 0.1},
            {"lower_limit": -1.57, "upper_limit": -1.27},
            {"lower_limit": -0.2, "upper_limit": 0.2},
            {"lower_limit": -0.2, "upper_limit": 0.2},
            {"lower_limit": 3.14, "upper_limit": 3.14},
            {"lower_limit": 0.0, "upper_limit": 0.0},
            {"lower_limit": -0.3, "upper_limit": -0.3},
        ]

        self.filter_alpha = 0.8
        self.prev_actions = None
        self.ball_history = []

    def adjust_joint_positions(self, dof_positions):
        dof_positions[4] = dof_positions[4] * 0.5
        dof_positions[1] = dof_positions[1] + 1.57
        return dof_positions

    def map_range(self, values):
        mapped_values = []
        for v, limits in zip(values, self.joint_limits):
            target_min, target_max = limits["lower_limit"], limits["upper_limit"]
            mapped_value = (v + 1) * 0.5 * (target_max - target_min) + target_min
            mapped_values.append(mapped_value)
        return mapped_values

    def build_obs(self, dof_positions, dof_velocities, ball_positions, ball_linvels):
        obs = np.concatenate((dof_positions, dof_velocities, ball_positions, ball_linvels, [0, 0, 0]), axis=0)
        if len(obs) != 18:
            raise ValueError(f"Observation length is {len(obs)}, expected 18.")
        return np.reshape(obs, (1, len(obs)))

    def infer(self, obs):
        return self.loaded_model.run(None, {"obs": obs.astype(np.float32)})[0]

    def run(self):
        rospy.loginfo("Waiting for data...")
        time.sleep(2)

        while not self.joint_state_observer.get_observations()['position']:
            rospy.loginfo("Waiting for joint state data...")
            time.sleep(0.01)

        start_time = None  
        loop_count = 0
        total_loop_time = 0


        csv_file = open("time_step_obs_action.csv", mode="w", newline="")
        csv_writer = csv.writer(csv_file)
        csv_writer.writerow(["relative_time", "loop_duration", "inference_duration", "obs", "raw_action", "scaled_action"])

        try:
            while not rospy.is_shutdown():
                loop_start_time = time.time()  

                frame_undistort, mask_paper, warped_board, mask_ball = self.tracker.update()
                ball_positions, ball_linvels = self.tracker.get_position_and_velocity()

    
                if ball_positions is None or ball_linvels is None:
                    continue

                if start_time is None:
                    start_time = time.time()  

                ball_positions = [-value for value in ball_positions]
                ball_linvels = [-value for value in ball_linvels]
                self.ball_history.append(ball_positions)
                if len(self.ball_history) > 100:
                    self.ball_history.pop(0)

                dof_positions = self.joint_state_observer.get_observations()['position']
                dof_velocities = self.joint_state_observer.get_observations()['velocity'][:4]

                dof_positions = self.adjust_joint_positions(list(dof_positions))

                try:
                    obs = self.build_obs(dof_positions, dof_velocities, ball_positions, ball_linvels)
                    print("observation", obs)

                    infer_start_time = time.time()  
                    actions = self.infer(obs)
                    infer_end_time = time.time()  

                    raw_action = actions.flatten()
                    print("action before scale:", raw_action)

                    actions = self.map_range(raw_action.tolist())

                    if self.prev_actions is None:
                        filtered_actions = actions
                    else:
                        filtered_actions = [
                            self.filter_alpha * a + (1 - self.filter_alpha) * pa 
                            for a, pa in zip(actions, self.prev_actions)
                        ]
                    self.prev_actions = filtered_actions

                    self.joint_state_publisher.publish_actions(filtered_actions)
                    print("action after scale and filtering:", filtered_actions)

                    loop_end_time = time.time()  
                    loop_duration = loop_end_time - loop_start_time 
                    inference_duration = infer_end_time - infer_start_time  
                    total_loop_time += loop_duration
                    loop_count += 1


                    relative_time = loop_end_time - start_time

                    print(f"Loop execution time: {loop_duration:.4f} sec | Inference time: {inference_duration:.4f} sec")


                    csv_writer.writerow([
                        relative_time,
                        loop_duration,
                        inference_duration,
                        obs.flatten().tolist(),
                        raw_action.tolist(),
                        filtered_actions
                    ])

                except ValueError as e:
                    print(e)

                time.sleep(0.01)  
            
        except rospy.ROSInterruptException:
            rospy.loginfo("Test node terminated.")
        
        finally:
            csv_file.close()  
            if start_time is not None:
                end_time = time.time() 
                total_runtime = end_time - start_time 
                avg_loop_time = total_loop_time / loop_count if loop_count > 0 else 0 
                
                print(f"Total runtime: {total_runtime:.2f} sec")
                print(f"Average loop execution time: {avg_loop_time:.4f} sec")
            else:
                print("No valid observations received, no runtime recorded.")

if __name__ == '__main__':
    try:
        test_instance = Test()
        test_instance.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Test node terminated.")
    except Exception as e:
        print(e)
