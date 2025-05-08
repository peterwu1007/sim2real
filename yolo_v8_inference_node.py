#!/home/mpc/anaconda3/envs/yolo/bin/python

from init_ros_node import init_ros_node
import rospy
from sensor_msgs.msg import Image
from ultralytics import YOLO
import cv2
import sys
import numpy as np
import time
sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')

def getMarkerCoordinates(markers, ids, point=0):
    if markers is None or ids is None:
        print("Markers or IDs are None.")
        return [], None
    marker_array = []
    for marker in markers:
        marker_array.append([int(marker[0][point][0]), int(marker[0][point][1])])
    ids = [id[0] for id in ids]  # Flatten the list of lists
    print(marker_array)
    return marker_array, ids

def imgmsg_to_cv2(img_msg):
    if img_msg.encoding != "bgr8":
        rospy.logerr("This node has been hardcoded to the 'bgr8' encoding. Change the code if you're using a different camera.")
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
  

def draw_corners(img, corners):
    if corners is None:
        return
    for corner in corners:
        if corner is not None:
            cv2.circle(img, (corner[0], corner[1]), 10, (0, 255, 0), thickness=-1)


def draw_numbers(img, corners, ids):
    if corners is None or ids is None:
        return
    font = cv2.FONT_HERSHEY_SIMPLEX
    thickness = 4
    for number, corner in enumerate(corners):
        if corner is not None:
            cv2.putText(img, str(ids[number]), (corner[0] + 10, corner[1] + 10), font, 2, (0, 0, 0), thickness)


def show_spec(img, corners):
    if corners is None:
        return
    font = cv2.FONT_HERSHEY_SIMPLEX
    thickness = 1
    amountOfCorners = len(corners)
    spec_string = f"{amountOfCorners} markers found."
    cv2.putText(img, spec_string, (15, 15), font, 0.5, (0, 0, 250), thickness)


def draw_field(img, corners, ids):
    if corners is None or ids is None or len(corners) != 4:
        return img, False
    markers_sorted = [None] * 4
    for sorted_corner_id in [1, 2, 3, 4]:
        if sorted_corner_id in ids:
            index = ids.index(sorted_corner_id)
            markers_sorted[sorted_corner_id - 1] = corners[index]
    
    if None in markers_sorted:
        return img, False

    contours = np.array(markers_sorted, dtype=np.int32)
    overlay = img.copy()
    cv2.fillPoly(overlay, pts=[contours], color=(255, 215, 0))
    alpha = 0.4
    img_new = cv2.addWeighted(overlay, alpha, img, 1 - alpha, 0)
    return img_new, True




class YOLOv8InferenceNode:
    def __init__(self):
        init_ros_node('yolov8_inference_node')
        self.image_sub = rospy.Subscriber('/camera/image_raw', Image, self.image_callback)
        self.image_pub = rospy.Publisher('/yolov8/detections', Image, queue_size=10)
        self.model_path = '/home/mpc/catkin_ws/src/peter/src/Yolov8_ros/yolov8_ros/weights/best.pt'
        
        # Load YOLO model with debug info
        try:
            self.model = YOLO(self.model_path)
            rospy.loginfo("YOLO model loaded successfully.")
        except Exception as e:
            rospy.logerr(f"Error loading YOLO model: {e}")
            raise

        self.prev_position = None
       

        self.ball_position = [0, 0]
        self.velocity = [0, 0]
        self.marker_location_hold = True
        self.current_square_points = [None] * 4
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_100)
        self.parameters = cv2.aruco.DetectorParameters()

    def image_callback(self, data):
        frame = imgmsg_to_cv2(data)
        h, w, _ = frame.shape
        width = 720
        height = 960  
        # frame = cv2.resize(frame, (width, height), interpolation=cv2.INTER_NEAREST)
        frame = cv2.resize(frame, (width, height), interpolation=cv2.INTER_CUBIC)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        marker_location_hold = True
        current_square_points = [None] * 4
        start_time = time.time()
        current_time = time.time()
        delay = 0
        read_states_list = []
        all_states = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13}

        aruco_detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.parameters)
        corners, ids, rejected = aruco_detector.detectMarkers(gray)

        frame_clean = frame.copy()

        # print(f"corners: {corners}")
        # print(f"ids: {ids}")

        left_corners, corner_ids = getMarkerCoordinates(corners, ids, 0)
        # print(f"left_corners: {left_corners}")
        # print(f"corner_ids: {corner_ids}")

        # if marker_location_hold:
        #     if corner_ids is not None:
        #         count = 0
        #         for id in corner_ids:
        #             if id > 4:
        #                 break
        #             if count < len(left_corners) and left_corners[count] is not None:
        #                 current_square_points[id - 1] = left_corners[count]
        #             count += 1
        #     left_corners = current_square_points
        #     corner_ids = [1, 2, 3, 4]

        # if (start_time + delay * 1) < current_time and (start_time + delay * 2) > current_time:
        #     cv2.aruco.drawDetectedMarkers(frame, corners)
        # if (start_time + delay * 2) < current_time:
        #     if left_corners is not None and all(corner is not None for corner in left_corners):
        #         draw_corners(frame, left_corners)
        #     else:
        #         print("Warning: left_corners is None or contains None elements")
        # if (start_time + delay * 3) < current_time:
        #     if left_corners is not None and corner_ids is not None:
        #         draw_numbers(frame, left_corners, corner_ids)
        #     else:
        #         print("Warning: left_corners or corner_ids is None or contains None elements")
        # if (start_time + delay * 4) < current_time:
        #     show_spec(frame, left_corners)

        # if left_corners is not None and corner_ids is not None:
        #     frame_with_square, squareFound = draw_field(frame, left_corners, corner_ids)
        # else:
        #     frame_with_square, squareFound = frame, False



        # if len(corner_ids) == 4:
        #     frame_with_square, squareFound = self.draw_field(frame, left_corners, corner_ids)
        # else:
        #     frame_with_square, squareFound = frame, False

        if ids is not None and len(ids) == 4:
            all_corners = np.concatenate(corners)
            pts = all_corners.reshape(16, 2)
            ordered_pts = self.order_points(pts)
            cropped_image = self.four_point_transform(frame, ordered_pts)


            cropped_image = cv2.resize(cropped_image, None, fx=0.5, fy=0.5, interpolation=cv2.INTER_NEAREST)

            try:
                results = self.model(cropped_image)
                rospy.loginfo("YOLO inference executed successfully.")

                crop_height, crop_width = cropped_image.shape[:2]
                real_board_width = 0.25 
                real_board_height = 0.25  

                for result in results:
                    for box in result.boxes:
                        x1n, y1n, x2n, y2n = box.xyxyn[0].tolist()
                        x1 = int(x1n * crop_width)
                        y1 = int(y1n * crop_height)
                        x2 = int(x2n * crop_width)
                        y2 = int(y2n * crop_height)
                        cls = int(box.cls[0].item())

                        if cls == 0:  
                            ball_x = (x1 + x2) // 2
                            ball_y = (y1 + y2) // 2
                            
                       
                            ball_x = crop_width - ball_x
                            ball_y = crop_height - ball_y
                            
                       
                            real_ball_x = ball_x * (real_board_width / crop_width)
                            real_ball_y = ball_y * (real_board_height / crop_height)

                            self.ball_position = [real_ball_x, real_ball_y]
                            
                            if self.prev_position:
                                self.velocity = [
                                    self.ball_position[0] - self.prev_position[0],
                                    self.ball_position[1] - self.prev_position[1]
                                ]
                            self.prev_position = self.ball_position

                          
                            rospy.loginfo(f"Ball Position (meters): {self.ball_position}")
                            rospy.loginfo(f"Ball Velocity (meters/frame): {self.velocity}")

                            cv2.rectangle(cropped_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                            cv2.circle(cropped_image, (ball_x, ball_y), 5, (0, 255, 0), -1)

                            if any(self.velocity):
                                velocity_text = f'Velocity: ({self.velocity[0]:.4f}, {self.velocity[1]:.4f}) m/frame'
                                position_text = f'Position: ({self.ball_position[0]:.4f}, {self.ball_position[1]:.4f}) m'
                                cv2.putText(cropped_image, velocity_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                                cv2.putText(cropped_image, position_text, (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                        # elif cls == 1:
                        #     cv2.rectangle(cropped_image, (x1, y1), (x2, y2), (255, 0, 0), 2)
                        #     cv2.putText(cropped_image, 'Board', (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
                
                # self.image_pub.publish(cv2_to_imgmsg(cropped_image))
            except Exception as e:
                rospy.logerr(f"Error during YOLO inference: {e}")

            # self.image_pub.publish(cv2_to_imgmsg(cropped_image))

    def get_marker_coordinates(self, markers, ids, point=0):
        if markers is None or ids is None:
            print("Markers or IDs are None.")
            return [], None
        marker_array = []
        for marker in markers:
            marker_array.append([int(marker[0][point][0]), int(marker[0][point][1])])
        ids = [id[0] for id in ids]  # Flatten the list of lists
        return marker_array, ids


    def draw_field(self, img, corners, ids):
        if corners is None or ids is None or len(corners) != 4:
            return img, False
        markers_sorted = [None] * 4
        for sorted_corner_id in [1, 2, 3, 4]:
            if sorted_corner_id in ids:
                index = ids.index(sorted_corner_id)
                markers_sorted[sorted_corner_id - 1] = corners[index]
        
        if None in markers_sorted:
            return img, False

        contours = np.array(markers_sorted, dtype=np.int32)
        overlay = img.copy()
        cv2.fillPoly(overlay, pts=[contours], color=(255, 215, 0))
        alpha = 0.4
        img_new = cv2.addWeighted(overlay, alpha, img, 1 - alpha, 0)
        return img_new, True

    def order_points(self, pts):
        rect = np.zeros((4, 2), dtype="float32")
        s = pts.sum(axis=1)
        rect[0] = pts[np.argmin(s)]
        rect[2] = pts[np.argmax(s)]
        diff = np.diff(pts, axis=1)
        rect[1] = pts[np.argmin(diff)]
        rect[3] = pts[np.argmax(diff)]
        return rect

    def four_point_transform(self, image, pts):
        rect = self.order_points(pts)
        (tl, tr, br, bl) = rect
        widthA = np.sqrt(((br[0] - bl[0]) ** 2) + ((br[1] - bl[1]) ** 2))
        widthB = np.sqrt(((tr[0] - tl[0]) ** 2) + ((tr[1] - tl[1]) ** 2))
        maxWidth = max(int(widthA), int(widthB))
        heightA = np.sqrt(((tr[0] - br[0]) ** 2) + ((tr[1] - br[1]) ** 2))
        heightB = np.sqrt(((tl[0] - bl[0]) ** 2) + ((tl[1] - bl[1]) ** 2))
        maxHeight = max(int(heightA), int(heightB))
        # print("hieght",maxHeight)
        # print("width",maxWidth)
        dst = np.array([
            [0, 0],
            [maxWidth - 1, 0],
            [maxWidth - 1, maxHeight - 1],
            [0, maxHeight - 1]], dtype="float32")
        M = cv2.getPerspectiveTransform(rect, dst)
        warped = cv2.warpPerspective(image, M, (maxWidth, maxHeight))
        return warped

    def get_ball_position_and_velocity(self):
        return np.array(self.ball_position), np.array(self.velocity)

if __name__ == '__main__':
    try:
        yolov8_node = YOLOv8InferenceNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass   
