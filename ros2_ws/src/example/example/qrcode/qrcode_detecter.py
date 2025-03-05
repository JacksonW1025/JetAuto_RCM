#!/usr/bin/env python3
# encoding: utf-8
import os
import cv2  
import queue
import rclpy
import threading
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import Image

class QRCodeDetectNode(Node):
    def __init__(self, name):
        rclpy.init()
        super().__init__(name)
        self.running = True
        self.model_path = os.path.join(os.path.abspath(os.path.split(os.path.realpath(__file__))[0]), 'model/detector.tflite')
        self.image_queue = queue.Queue(maxsize=2)
        self.image_sub = self.create_subscription(Image, '/depth_cam/rgb/image_raw', self.image_callback, 1)
        self.qcd = cv2.QRCodeDetector()
        threading.Thread(target=self.main, daemon=True).start()  

    def image_callback(self, ros_image):
        rgb_image = np.ndarray(shape=(ros_image.height, ros_image.width, 3), dtype=np.uint8,
                               buffer=ros_image.data)  # 原始 RGB 画面
        bgr_image = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2BGR)
        if self.image_queue.full():
            # 如果队列已满，丢弃最旧的图像
            self.image_queue.get()
            # 将图像放入队列
        self.image_queue.put(bgr_image)

    def main(self):
        while self.running:
            try:
                image = self.image_queue.get(block=True, timeout=1)
            except queue.Empty:
                if not self.running:
                    break
                else:
                    continue
            ret_qr, decoded_info, points, _ = self.qcd.detectAndDecodeMulti(image)
            if ret_qr:
                for s, p in zip(decoded_info, points):
                    if s:
                        print(s)
                        color = (0, 255, 0)
                    else:
                        color = (0, 0, 255)
                    image = cv2.polylines(image, [p.astype(int)], True, color, 8)
            cv2.imshow('image', image)
            key = cv2.waitKey(1)
            if key == ord('q') or key == 27:  # 按q或者esc退出
              break

def main():
    node = QRCodeDetectNode('qrcode_detect')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.destroy_node()
        rclpy.shutdown()
        print('shutdown')
    finally:
        print('shutdown finish')

if __name__ == "__main__":
    main()
