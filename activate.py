#!/usr/bin/env python
from __future__ import print_function
from std_msgs.msg import String
from thormang3_manipulation_module_msgs.msg import KinematicsPose, KinematicsArrayPose
import numpy as np
import math
import rospy
from sensor_msgs.msg import JointState
from robotis_controller_msgs.msg import SyncWriteItem
from thormang3_manipulation_module_msgs.srv import GetKinematicsPose, GetKinematicsPoseRequest, GetKinematicsPoseResponse
import threading
import time
class Kinematics:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('kinematics_control', anonymous=True)
        
        
        self.bounding_box = {
                'x_min': 0.19,
                'x_max': 0.43,
                'y_min': -0.33,
                'y_max': -0.17,
                'z_min': 0.65,
                'z_max': 0.76
            }
        

        # Define constants and variables
        self.pi = 3.1415
        self.min = 0
        self.max = 10
        self.xp = [self.min, self.max]
        self.fp = [-self.pi, self.pi]
        
        self.pub_rate = rospy.Rate(10)
        self.thread_rate = rospy.Rate(30)
        self.module_name = None
        self.status_msg = None
        self.thread1_flag = False
        self.mutex = threading.Lock()
        self.joint_position = {}
        
        # Define publishers
        self.module_control_pub = rospy.Publisher('/robotis/enable_ctrl_module', String, queue_size=10)
        self.send_ini_pose_msg_pub = rospy.Publisher('/robotis/manipulation/ini_pose_msg', String, queue_size=10)
        self.send_ik_msg_pub = rospy.Publisher('/robotis/manipulation/kinematics_pose_msg', KinematicsPose, queue_size=5)
        self.send_ik_arr_msg_pub = rospy.Publisher('/robotis/manipulation/kinematics_pose_arr_msg', KinematicsArrayPose, queue_size=5)
        self.set_joint_pub = rospy.Publisher('/robotis/set_joint_states', JointState, queue_size=10)
        self.sync_write_pub = rospy.Publisher('/robotis/sync_write_item', SyncWriteItem, queue_size=10)
        
        self.right_arm_joints = ['r_arm_sh_p1', 'r_arm_sh_p2', 'r_arm_sh_r', 'r_arm_el_y', 'r_arm_wr_p', 'r_arm_wr_r', 'r_arm_wr_y']
        

        # Service client for getting kinematics pose
        rospy.wait_for_service('/robotis/manipulation/get_kinematics_pose')
        self.get_kinematics_pose_client = rospy.ServiceProxy('/robotis/manipulation/get_kinematics_pose', GetKinematicsPose)
        # Start checking boundaries in a separate thread
        # self.check_thread = threading.Thread(target=self.check_boundaries)
        # self.check_thread.start()
        

        # Initialize operations
        rospy.sleep(1)
        self.module_control_pub.publish(String("manipulation_module"))
        rospy.sleep(1)
        self.send_ini_pose_msg_pub.publish(String("ini_pose"))
        rospy.sleep(1)
        self.send_ini_pose_msg_pub.publish(String("manipulation_module"))
        
        #bounding box
        # self.check_boundaries()

        # Set joint states
        # self.set_joint_states("all_hand",True)
        # Get the kinematics pose of the right arm
        # pose = self.get_kinematics_pose("right_arm")
        # if pose:
        #     rospy.loginfo("Kinematics pose: {}".format(pose))
        # else:
        #     rospy.logerr("Failed to get kinematics pose")
    
    def check_boundaries(self):
        rate = rospy.Rate(1)  # Check boundaries at 1 Hz
        while not rospy.is_shutdown():
            pose = self.get_kinematics_pose("right_arm")  # Replace with your actual group name
            if pose:
                x = pose['x']
                y = pose['y']
                z = pose['z']
                rospy.loginfo("Current Position -> x: %f, y: %f, z: %f", x, y, z)
                
                if (x < self.bounding_box['x_min'] or x > self.bounding_box['x_max'] or
                    y < self.bounding_box['y_min'] or y > self.bounding_box['y_max'] or
                    z < self.bounding_box['z_min'] or z > self.bounding_box['z_max']):
                    
                    self.set_joint_states("all", False)
                    rospy.loginfo("Out of range.")
            rate.sleep()

    

    def set_joint_states(self, joint_name, torque):
        '''
        Enable/Disable Torque
        '''
        sync_write           = SyncWriteItem()
        sync_write.item_name = "torque_enable"


        if True :   # Thormang3 Gogoro #Im lazy me too
            if joint_name[0] == "all":
                joint_name = [  "r_arm_sh_p1", "l_arm_sh_p1", "r_arm_sh_r",  "l_arm_sh_r",  "r_arm_sh_p2", "l_arm_sh_p2", "r_arm_el_y",   "l_arm_el_y",
                                "r_arm_wr_r",  "l_arm_wr_r",  "r_arm_wr_y",  "l_arm_wr_y",  "r_arm_wr_p",  "l_arm_wr_p",  "r_leg_hip_y",  "l_leg_hip_y",
                                "r_leg_hip_r", "l_leg_hip_r", "r_leg_hip_p", "l_leg_hip_p", "r_leg_kn_p",  "l_leg_kn_p",  "r_leg_an_p",   "l_leg_an_p",
                                "r_leg_an_r",  "l_leg_an_r",  "torso_y",     "head_y",      "head_p",      "l_arm_grip",  "r_arm_grip" ]

            elif joint_name == "left_arm":
                joint_name = [ "l_arm_el_y", "l_arm_sh_p1", "l_arm_sh_p2", "l_arm_sh_r", "l_arm_wr_r", "l_arm_wr_y", "l_arm_wr_p", "l_arm_grip" ]

            elif joint_name == "right_arm":
                joint_name = [ "r_arm_el_y", "r_arm_sh_p1", "r_arm_sh_p2", "r_arm_sh_r", "r_arm_wr_r", "r_arm_wr_y", "r_arm_wr_p", "r_arm_grip" ]

            elif joint_name == "left_leg":
                joint_name = [ "l_leg_hip_y", "l_leg_hip_r", "l_leg_hip_p", "l_leg_kn_p", "l_leg_an_p", "l_leg_an_r" ]

            elif joint_name == "right_leg":
                joint_name = [ "r_leg_hip_y", "r_leg_hip_r", "r_leg_hip_p", "r_leg_kn_p", "r_leg_an_p", "r_leg_an_r" ]
            elif joint_name == "all_hand":
                        
                joint_name = ["r_arm_sh_p1", "r_arm_sh_p2", "r_arm_sh_r","r_arm_el_y",          "r_arm_wr_p","r_arm_wr_r","r_arm_wr_y","l_arm_sh_p1", "l_arm_sh_p2", "l_arm_sh_r", "l_arm_wr_p","l_arm_wr_r","l_arm_wr_y","l_arm_el_y" ]
            else:
                print("TEST UGO DEBUG")
                sync_write.joint_name = joint_name


        sync_write.joint_name = joint_name
        sync_write.value      = [ torque for _ in range(len(sync_write.joint_name )) ]
        print([ np.radians(self.joint_position.get(_,0)) for _ in joint_name ])
        
        #rospy.sleep(10)
        # turn off torque
        if not torque: 
            self.publisher_(self.sync_write_pub, sync_write)
        # turn on torque
        else:
            joint           = JointState()
            joint.name      = joint_name

            self.mutex.acquire()
            joint.position  = [ np.radians(self.joint_position.get(_,0)) for _ in joint_name ] # read present position
            self.mutex.release()

            joint.velocity  = [ 0 for _ in joint_name ]
            joint.effort    = [ 0 for _ in joint_name ]
            # torque          = 2 # 0 default
            # joint.effort    = [      np.interp(torque, [0, 100], [0, 13.700920687]) if joint=="head_p" or joint=="head_y" \
            #                     else np.interp(torque, [0, 100], [0, 32.090233596]) for joint in joint_name ]

            self.publisher_(self.set_joint_pub, joint)        # set present position

            self.publisher_(self.sync_write_pub, sync_write)  # turn on torque

    def get_kinematics_pose(self, group_name):
        rospy.wait_for_service('/robotis/manipulation/get_kinematics_pose')
        try:
            rospy.loginfo("Calling service /robotis/manipulation/get_kinematics_pose with group_name: {}".format(group_name))
            request = GetKinematicsPoseRequest()
            request.group_name = group_name
            resp = self.get_kinematics_pose_client(request)
            if not resp:
                rospy.logerr("Received empty response from kinematics service.")
                return None

            euler_rad = self.quaternion_to_euler(resp.group_pose.orientation.x, resp.group_pose.orientation.y,
                                                 resp.group_pose.orientation.z, resp.group_pose.orientation.w)
            euler_deg = np.degrees(euler_rad)
            return {'x': resp.group_pose.position.x,
                    'y': resp.group_pose.position.y,
                    'z': resp.group_pose.position.z,              
                    'roll': euler_deg[0],
                    'pitch': euler_deg[1],
                    'yaw': euler_deg[2]}
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: {}".format(e))
            return None

    def joint_status_callback(self, data):
        self.joint_status = data
        rospy.loginfo(rospy.get_caller_id() + "I heard %s", data)
        self.publisher_(self.set_joint_pub, data, latch=False)
        rospy.sleep(1)

    def publisher_(self, topic, msg, latch=False):
        if latch:
            for _ in range(4):
                topic.publish(msg)
                self.pub_rate.sleep()
        else:
            topic.publish(msg)
    def run(self):
        # Loop to wait for user input
        torque_enabled = False
        while not rospy.is_shutdown():
            try:
                    time.sleep(5)
                    torque_enabled = not torque_enabled
                    self.set_joint_states("all_hand", torque_enabled)
                    state = "enabled" if torque_enabled else "disabled"
                    rospy.loginfo("Torque {} for all hand joints.".format(state))  
            except (EOFError, KeyboardInterrupt):
                break
    def quaternion_to_euler(self, x, y, z, w):
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(t0, t1)
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch = math.asin(t2)
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(t3, t4)
        return [roll, pitch, yaw]

if __name__ == '__main__':
    kin = Kinematics()
    # kin.run()
    rospy.spin()  # Keep the node running
