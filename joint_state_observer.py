#!/home/mpc/anaconda3/envs/yolo/bin/python

from init_ros_node import init_ros_node
import rospy
from sensor_msgs.msg import JointState

class JointStateObserver:
    def __init__(self):
        init_ros_node('joint_state_observer_node')
        self.required_joints = [
            'r_arm_sh_p1', 'r_arm_sh_r', 'r_arm_sh_p2', 
            'r_arm_el_y', 'r_arm_wr_r', 
            'r_arm_wr_y', 'r_arm_wr_p'
        ]

        self.observations = {
            "position": [],
            "velocity": [],
            "name": self.required_joints
        }

        rospy.Subscriber("/robotis/goal_joint_states", JointState, self.joint_state_callback)
        rospy.loginfo("JointStateObserver initialized and subscribed to /robotis/goal_joint_states")

    def joint_state_callback(self, msg):
        try:
            joint_data = {name: {'position': 0.0, 'velocity': 0.0} for name in self.required_joints}
            
            for i, name in enumerate(msg.name):
                if name in joint_data:
                    joint_data[name] = {
                        'position': msg.position[i],
                        'velocity': msg.velocity[i]
                    }

            self.observations['position'] = [joint_data[name]['position'] for name in self.required_joints]
            self.observations['velocity'] = [joint_data[name]['velocity'] for name in self.required_joints]

            # rospy.loginfo("Position: {}".format(self.observations['position']))
            # rospy.loginfo("Velocity: {}".format(self.observations['velocity']))
            # rospy.loginfo("Name: {}".format(self.required_joints))
        except Exception as e:
            rospy.logerr("Error in joint_state_callback: {}".format(e))

    def get_observations(self):
        return self.observations

if __name__ == '__main__':
    try:
        observer = JointStateObserver()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
