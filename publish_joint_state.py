#!/home/mpc/anaconda3/envs/yolo/bin/python

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from init_ros_node import init_ros_node
from pioneer import Motor
class Publisher:
    def __init__(self):
        init_ros_node('set_joint_states_publisher')
    
        self.pub = rospy.Publisher('/robotis/set_joint_states', JointState, queue_size=1)
        self.rate = rospy.Rate(10)  # 10 Hz


        self.joint_state = JointState()
        self.joint_state.header = Header()
        self.joint_state.header.seq = 0
        self.joint_state.header.stamp = rospy.Time.now()
        self.joint_state.header.frame_id = ''
        robot_name = "Thormang3_Wolf"  
        self.motor = Motor(robot_name)


        self.joint_torque_name = ['r_arm_sh_p1', 'r_arm_sh_r', 'r_arm_sh_p2', 
                    'r_arm_el_y', 'r_arm_wr_r', 'r_arm_wr_y', 'r_arm_wr_p']
        self.torque = 0.5  


        
        self.joint_state.name = [
            'r_arm_sh_p1', 'r_arm_sh_r', 'r_arm_sh_p2', 
            'r_arm_el_y', 'r_arm_wr_r', 
            'r_arm_wr_y', 'r_arm_wr_p'
        ]

    # def map_range(self, values, source_min=-1, source_max=1):

    #     if source_max - source_min == 0:
    #         raise ValueError("Source range cannot be zero.")
        

    #     if not isinstance(values, list) or len(values) != len(self.joint_limits):
    #         raise ValueError("Values must be a list with the same length as joint_limits.")
        

    #     mapped_values = []
    #     for v, limits in zip(values, self.joint_limits):
    #         target_min = limits["lower_limit"]
    #         target_max = limits["upper_limit"]

    #         # mapped_value = (target_max - target_min) / (source_max - source_min) * v + (target_max - target_min) / (source_max - source_min)
    #         mapped_value = (v+1)*0.5 * (target_max-target_min)+target_min
    #         mapped_values.append(mapped_value)
        
    #     return mapped_values
        
    # def adjust_publish_positions(self, actions):
   


    #     actions[4] = actions[4] * 2 + 1.57
    #     actions[1] = actions[1] - 1.57


    #     return actions

    def publish_actions(self, actions):


        # adjusted_actions = self.adjust_publish_positions(actions)
        
        self.joint_state.header.stamp = rospy.Time.now()  
        self.motor.set_joint_torque(self.joint_torque_name, self.torque)

        self.joint_state.position = actions  # actions 是 numpy array，需轉換為 list

        self.joint_state.velocity = [0.0] * len(self.joint_state.name)
        self.joint_state.effort = [0.0] * len(self.joint_state.name)


        self.pub.publish(self.joint_state)
        self.rate.sleep()
