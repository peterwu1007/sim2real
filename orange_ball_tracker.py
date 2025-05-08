import cv2
import numpy as np
import pickle
import math
import time
class OrangeBallTracker:
    def __init__(
        self,
        camera_index=0,
        calibration_file=None,
        lower_color_paper=np.array([116, 30, 163]),
        upper_color_paper=np.array([137, 113, 255]),
        lower_color_ball=np.array([10, 150, 150]),
        upper_color_ball=np.array([25, 255, 255]),
        real_board_width=0.25,
        real_board_height=0.25
    ):
        """
        初始化:
        :param camera_index: 攝影機編號 (或影片路徑)
        :param calibration_file: 相機校正檔路徑 (若無可傳 None)
        :param lower_color_paper, upper_color_paper: 偵測四角紙的 HSV 範圍
        :param lower_color_ball, upper_color_ball: 偵測球的 HSV 範圍
        :param real_board_width, real_board_height: 板子的真實寬高 (公尺)
        """
        self.cap = cv2.VideoCapture(camera_index)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
        if not self.cap.isOpened():
            raise IOError("無法打開攝影機，請確認 camera_index 或路徑。")

        # 如果有校正檔，讀取並存起來
        self.has_calibration = False
        if calibration_file is not None:
            try:
                with open(calibration_file, 'rb') as f:
                    calibration_data = pickle.load(f)
                self.camera_matrix = calibration_data['mtx']
                self.dist_coeffs = calibration_data['dist']
                self.has_calibration = True
            except Exception as e:
                print("讀取校正檔失敗，將略過去畸變:", e)
                self.has_calibration = False

        # HSV 範圍 (紙)
        self.lower_color_paper = lower_color_paper
        self.upper_color_paper = upper_color_paper

        # HSV 範圍 (球)
        self.lower_color_ball = lower_color_ball
        self.upper_color_ball = upper_color_ball

        # 板子真實尺寸 (公尺)
        self.real_board_width = real_board_width
        self.real_board_height = real_board_height

        # 前一幀球位置 (用於計算速度)
        self.prev_position = None
        self.prev_time = None
        # 最新的球位置與速度 (對外提供)
        self.current_position = None
        self.current_velocity = None

    def order_points(self, pts):
        """
        將四個點座標排序為: [左上, 右上, 右下, 左下]
        """
        rect = np.zeros((4, 2), dtype="float32")
        s = pts.sum(axis=1)
        diff = np.diff(pts, axis=1)

        rect[0] = pts[np.argmin(s)]    # x+y 最小 -> 左上
        rect[2] = pts[np.argmax(s)]    # x+y 最大 -> 右下
        rect[1] = pts[np.argmin(diff)] # x-y 最小 -> 右上
        rect[3] = pts[np.argmax(diff)] # x-y 最大 -> 左下
        return rect

    def four_point_transform(self, image, pts):
        """
        給定影像及四個頂點座標，做透視轉換並回傳 crop 後影像
        """
        rect = self.order_points(pts)
        (tl, tr, br, bl) = rect

        # 計算新的影像寬高
        widthA = math.sqrt(((br[0] - bl[0]) ** 2) + ((br[1] - bl[1]) ** 2))
        widthB = math.sqrt(((tr[0] - tl[0]) ** 2) + ((tr[1] - tl[1]) ** 2))
        maxWidth = max(int(widthA), int(widthB))

        heightA = math.sqrt(((tr[0] - br[0]) ** 2) + ((tr[1] - br[1]) ** 2))
        heightB = math.sqrt(((tl[0] - bl[0]) ** 2) + ((tl[1] - bl[1]) ** 2))
        maxHeight = max(int(heightA), int(heightB))

        # 定義透視後影像的四個角
        dst = np.array([
            [0, 0],
            [maxWidth - 1, 0],
            [maxWidth - 1, maxHeight - 1],
            [0, maxHeight - 1]], dtype="float32")

        # 取得透視轉換矩陣
        M = cv2.getPerspectiveTransform(rect, dst)
        warped = cv2.warpPerspective(image, M, (maxWidth, maxHeight))
        return warped

    def update(self):
        """
        讀取一幀 (frame)，偵測紙與球，更新 self.current_position & self.current_velocity
        :return:
            frame_undistort: 去畸變後的原圖
            mask_paper: 偵測紙的遮罩
            warped_board: 透視轉換後的影像 (若沒找到 4 點則 None)
            mask_ball: 球的遮罩 (若沒找到 4 點則 None)
        """
        ret, frame = self.cap.read()
        if not ret:
            return None, None, None, None  # 讀不到畫面

        if self.has_calibration:
            h, w = frame.shape[:2]
            new_camera_matrix, _ = cv2.getOptimalNewCameraMatrix(
                self.camera_matrix, self.dist_coeffs, (w, h), 1, (w, h)
            )
            frame_undistort = cv2.undistort(frame, self.camera_matrix, self.dist_coeffs, None, new_camera_matrix)
        else:
            frame_undistort = frame.copy()

        # -- 1. 偵測紙 (找 4 個角) --
        hsv = cv2.cvtColor(frame_undistort, cv2.COLOR_BGR2HSV)
        mask_paper = cv2.inRange(hsv, self.lower_color_paper, self.upper_color_paper)

        # 形態學操作 (開運算 + 閉運算)
        mask_paper = cv2.morphologyEx(mask_paper, cv2.MORPH_OPEN, np.ones((3,3), np.uint8))
        mask_paper = cv2.morphologyEx(mask_paper, cv2.MORPH_CLOSE, np.ones((3,3), np.uint8))

        # 找輪廓
        contours_paper, _ = cv2.findContours(mask_paper, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        valid_contours = [c for c in contours_paper if cv2.contourArea(c) > 50]
        valid_contours.sort(key=lambda c: cv2.contourArea(c), reverse=True)
        top4_contours = valid_contours[:4]

        # 找到紙的四角
        corner_points = []
        for c in top4_contours:
            M_ = cv2.moments(c)
            if M_["m00"] == 0:
                continue
            cx = int(M_["m10"] / M_["m00"])
            cy = int(M_["m01"] / M_["m00"])
            corner_points.append((cx, cy))

        warped_board = None
        mask_ball = None

        if len(corner_points) == 4:
            corners_float = np.array(corner_points, dtype="float32")
            corners_ordered = self.order_points(corners_float)
            warped_board = self.four_point_transform(frame_undistort, corners_ordered)

            hsv_ball = cv2.cvtColor(warped_board, cv2.COLOR_BGR2HSV)
            mask_ball = cv2.inRange(hsv_ball, self.lower_color_ball, self.upper_color_ball)
            mask_ball = cv2.morphologyEx(mask_ball, cv2.MORPH_OPEN, np.ones((5,5), np.uint8))
            mask_ball = cv2.morphologyEx(mask_ball, cv2.MORPH_CLOSE, np.ones((5,5), np.uint8))
            current_time = time.time()

            contours_ball, _ = cv2.findContours(mask_ball, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if contours_ball:
                largest_ball = max(contours_ball, key=cv2.contourArea)
                (x_ball, y_ball), radius = cv2.minEnclosingCircle(largest_ball)

                if radius > 1:
                    # 球心像素座標
                    pixel_x = int(x_ball)
                    pixel_y = int(y_ball)

                    # warped_board 的大小
                    crop_height, crop_width = warped_board.shape[:2]
                    # 翻轉
                    pixel_x = crop_width - pixel_x
                    pixel_y = crop_height - pixel_y

                    # 轉換為真實世界長度 (公尺)
                    real_x = pixel_x * (self.real_board_width / crop_width)
                    real_y = pixel_y * (self.real_board_height / crop_height)

                    current_time = time.time()
                    if self.prev_position is not None and self.prev_time is not None:
                        dt = current_time - self.prev_time
                        if dt > 0:
                            velocity_x = (real_x - self.prev_position[0]) / dt
                            velocity_y = (real_y - self.prev_position[1]) / dt
                        else:
                            velocity_x, velocity_y = 0, 0
                    else:
                        velocity_x, velocity_y = 0, 0
                    self.prev_time = current_time
                    self.prev_position = (real_x, real_y)
                    # 記得將目前位置/速度更新 (若想要負號翻轉，可在此調整)
                    self.current_position = (real_x, real_y)
                    self.current_velocity = (velocity_x, velocity_y)
                else:
                    # 找到球但半徑太小，視為無效
                    self.current_position = None
                    self.current_velocity = None
            else:
                # 沒找到球
                self.current_position = None
                self.current_velocity = None
        else:
            # 沒找到 4 個角，無法做透視
            self.current_position = None
            self.current_velocity = None

        return frame_undistort, mask_paper, warped_board, mask_ball

    def get_position_and_velocity(self):
        """
        回傳當前偵測到的球位置與速度 (None 表示未偵測到)
        """
        return self.current_position, self.current_velocity

    def release(self):
        """
        釋放攝影機資源
        """
        if self.cap.isOpened():
            self.cap.release()
        cv2.destroyAllWindows()
