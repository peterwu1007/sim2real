#!/home/mpc/anaconda3/envs/yolo/bin/python

from init_ros_node import init_ros_node
import rospy
from sensor_msgs.msg import Image
from ultralytics import YOLO
import cv2
import sys
import numpy as np

sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')

def imgmsg_to_cv2(img_msg):
    if img_msg.encoding != "bgr8":
        rospy.logerr("This Coral detect node has been hardcoded to the 'bgr8' encoding. Come change the code if you're actually trying to implement a new camera")
    dtype = np.dtype("uint8")
    dtype = dtype.newbyteorder('>' if img_msg.is_bigendian else '<')
    image_opencv = np.ndarray(shape=(img_msg.height, img_msg.width, 3), dtype=dtype, buffer=img_msg.data)
    if img_msg.is_bigendian == (sys.byteorder == 'little'):
        image_opencv = image_opencv.byteswap().newbyteorder()
    return image_opencv

def cv2_to_imgmsg(cv_image):
    img_msg = Image()
    img_msg.height = cv_image.shape[0]
    img_msg.width = cv_image.shape[1]
    img_msg.encoding = "bgr8"
    img_msg.is_bigendian = 0
    img_msg.data = cv_image.tobytes()
    img_msg.step = len(img_msg.data) // img_msg.height
    return img_msg

class YOLOv8InferenceNode:
    def __init__(self):
        init_ros_node('yolov8_inference_node')
        self.image_sub = rospy.Subscriber('/camera/image_raw', Image, self.image_callback)
        self.image_pub = rospy.Publisher('/yolov8/detections', Image, queue_size=10)
        self.model = YOLO('/home/mpc/catkin_ws/src/peter/src/Yolov8_ros/yolov8_ros/ball/train8/weights/best.pt')
        self.prev_position = None
        self.target_height = 640
        self.target_width = 320
        self.ball_position = [0, 0]
        self.velocity = [0, 0]

    def image_callback(self, data):
        frame = imgmsg_to_cv2(data)
        frame = cv2.resize(frame, (self.target_width, self.target_height))
        results = self.model(frame)
        for result in results:
            boxes = result.boxes
            for box in boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
                conf = box.conf[0]
                cls = int(box.cls[0])

                if cls == 0:
                    self.ball_position = [(x1 + x2) // 2, (y1 + y2) // 2]
                    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.circle(frame, tuple(self.ball_position), 5, (0, 255, 0), -1)

                    if self.prev_position is not None:
                        self.velocity[0] = self.ball_position[0] - self.prev_position[0]
                        self.velocity[1] = self.ball_position[1] - self.prev_position[1]

                    self.prev_position = self.ball_position

                    if any(self.velocity):
                        velocity_text = 'Velocity: ({}, {})'.format(self.velocity[0], self.velocity[1])
                        position_text = 'Position: ({}, {})'.format(self.ball_position[0], self.ball_position[1])
                        cv2.putText(frame, velocity_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)              
                        cv2.putText(frame, position_text, (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                        print(velocity_text)
                        print(position_text)
        self.image_pub.publish(cv2_to_imgmsg(frame))
    
    def get_ball_position_and_velocity(self):
        return np.array(self.ball_position), np.array(self.velocity)

if __name__ == '__main__':
    try:
        yolov8_node = YOLOv8InferenceNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
