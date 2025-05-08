#!/home/mpc/anaconda3/envs/yolo/bin/python

from init_ros_node import init_ros_node
import rospy
from sensor_msgs.msg import JointState

class JointStateObserver:
    def __init__(self):
        init_ros_node('joint_state_observer_node')
        

        self.required_joints = ['r_arm_sh_p1', 'r_arm_sh_r', 'r_arm_sh_p2', 'r_arm_el_y']

        self.goal_joint_names = ['r_arm_wr_r', 'r_arm_wr_y', 'r_arm_wr_p']
        

        self.observations = {
            "position": [None] * 7,  
            "velocity": [None] * 7,  
            "name": self.required_joints + self.goal_joint_names  
        }


        rospy.Subscriber("/robotis/present_joint_states", JointState, self.joint_state_callback)
        rospy.Subscriber("/robotis/goal_joint_states", JointState, self.goal_joint_state_callback)
        rospy.loginfo("JointStateObserver initialized and subscribed to /robotis/present_joint_states and /goal_joint_state")

    def joint_state_callback(self, msg):
        try:

            joint_data = {name: {'position': 0.0, 'velocity': 0.0} for name in self.required_joints}
            
            for i, name in enumerate(msg.name):
                if name in joint_data:
                    joint_data[name] = {
                        'position': msg.position[i],
                        'velocity': msg.velocity[i]
                    }
            

            for idx, name in enumerate(self.required_joints):
                self.observations['position'][idx] = joint_data[name]['position']
                self.observations['velocity'][idx] = joint_data[name]['velocity']

        except Exception as e:
            rospy.logerr("Error in joint_state_callback: {}".format(e))

    def goal_joint_state_callback(self, msg):
        try:

            goal_joint_data = {name: {'position': 0.0, 'velocity': 0.0} for name in self.goal_joint_names}
            
            for i, name in enumerate(msg.name):
                if name in goal_joint_data:
                    goal_joint_data[name] = {
                        'position': msg.position[i],
                        'velocity': msg.velocity[i]
                    }


            for idx, name in enumerate(self.goal_joint_names):
                if name in goal_joint_data:
                    self.observations['position'][idx + len(self.required_joints)] = goal_joint_data[name]['position']
                    self.observations['velocity'][idx + len(self.required_joints)] = goal_joint_data[name]['velocity']
                else:

                    self.observations['position'][idx + len(self.required_joints)] = 0.0
                    self.observations['velocity'][idx + len(self.required_joints)] = 0.0

        except Exception as e:
            rospy.logerr("Error in goal_joint_state_callback: {}".format(e))

    def get_observations(self):

        for i in range(len(self.observations['position'])):
            if self.observations['position'][i] is None:
                self.observations['position'][i] = 0.0
            if self.observations['velocity'][i] is None:
                self.observations['velocity'][i] = 0.0
        return self.observations

# if __name__ == '__main__':
#     try:
#         observer = JointStateObserver()
#         rospy.spin()
#     except rospy.ROSInterruptException:
#         pass
