# init_ros_node.py
import rospy

def init_ros_node(node_name):
    if not rospy.core.is_initialized():
        rospy.init_node(node_name, anonymous=True)
