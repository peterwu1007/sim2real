#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from infer_net import Test

class JointStatePublisher:
    def __init__(self):

        rospy.init_node('set_joint_states_publisher', anonymous=True)
  
        self.pub = rospy.Publisher('/robotis/set_joint_states', JointState, queue_size=10)
      
        self.rate = rospy.Rate(10)  # 10 Hz
        self.action = Test.run()
        
        self.joint_state = JointState()
        self.joint_state.header = Header()
        self.joint_state.header.seq = 0
        self.joint_state.header.stamp = rospy.Time.now()
        self.joint_state.header.frame_id = ''

        
        self.joint_state.name = [
            'r_arm_sh_p1', 'r_arm_sh_r', 'r_arm_sh_p2', 
            'r_arm_el_y', 'r_arm_wr_r', 
            'r_arm_wr_y', 'r_arm_wr_p'
        ]



    def publish_joint_state(self):
        
        while not rospy.is_shutdown():
            self.joint_state.header.stamp = rospy.Time.now()  
            self.joint_state.position = self.action
            # self.joint_state.velocity = self.get_velocities()  
            self.joint_state.effort = [0.0] * len(self.joint_state.name)  
            self.pub.publish(self.joint_state)
            self.rate.sleep()

if __name__ == '__main__':
    try:
        joint_state_publisher = JointStatePublisher()
        joint_state_publisher.publish_joint_state()
    except rospy.ROSInterruptException:
        pass
