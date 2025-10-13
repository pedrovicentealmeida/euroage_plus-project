#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8
from PyQt6.QtWidgets import QApplication, QLabel, QVBoxLayout, QWidget
from PyQt6.QtGui import QPixmap
from PyQt6.QtCore import QTimer
import sys
import os
import threading 
from PyQt6.QtCore import QObject, pyqtSignal
from PyQt6.QtCore import QPropertyAnimation
from ament_index_python.packages import get_package_share_directory

class VisemeRenderer(Node, QObject):
    viseme_signal = pyqtSignal(int)  # Signal to update the GUI with a new viseme ID

    def __init__(self):
        Node.__init__(self, 'viseme_renderer')
        QObject.__init__(self)

        self.ignore_microsoft = False

        self.subscriber = self.create_subscription(UInt8, 'local_mouth_shape', self.subscriber_callback, 10)
        self.subscriber2 = self.create_subscription(UInt8,'mouth_shape',self.subscriber_callback2,10)
        self.get_logger().info("Subscription to 'mouth_shape' topic created.")

        # Get the root directory of the package
        package_share_directory = get_package_share_directory('viseme_renderer') # If you are using a workspace folder
        package_root = os.path.abspath(os.path.join(package_share_directory,'..','..','..','..','src','viseme_renderer'))
        self.faces_path = os.path.join(package_root, 'src', 'faces')
        self.get_logger().info(f'faces_path-->"{self.faces_path}"')

        self.app = QApplication(sys.argv)
        self.window = QWidget()
        self.window.setWindowTitle("Face Renderer")
        self.layout = QVBoxLayout()

        initial_image_path = os.path.join(self.faces_path, "viseme-0.png")
        print(f"Initial image path: {initial_image_path}")
        self.image_label = QLabel()
        self.pixmap = QPixmap(initial_image_path)
        if self.pixmap.isNull():
            self.get_logger().error(f"Failed to load image: {initial_image_path}")
        

        self.image_label.setPixmap(self.pixmap)
        self.image_label.setScaledContents(True)
        self.layout.addWidget(self.image_label)

        self.window.setLayout(self.layout)
        self.window.resize(800, 450)

        self.viseme_map = {
            0:  "viseme-0.png",
            1:  "viseme-1.png",
            2:  "viseme-2.png",
            3:  "viseme-3.png",
            4:  "viseme-4.png",
            5:  "viseme-5.png",
            6:  "viseme-6.png",
            7:  "viseme-7.png",
            8:  "viseme-8.png",
            9:  "viseme-9.png",
            10: "viseme-10.png",
            11: "viseme-11.png",
            12: "viseme-12.png",
            13: "viseme-13.png",
            14: "viseme-14.png",
            15: "viseme-15.png",
            16: "viseme-16.png",
            17: "viseme-17.png",
            18: "viseme-18.png", 
            19: "viseme-19.png",
            20: "viseme-20.png",
            21: "viseme-21.png",
            22: "viseme-22.png"
        }

        self.viseme_signal.connect(self.update_face)  # Connect the signal to the update_face method
        self.window.show()

    def subscriber_callback(self, msg):
        self.get_logger().info(f'Viseme Received Local Mouth Shape: "{msg.data}"')
        if self.ignore_microsoft == False:
            self.viseme_signal.emit(msg.data)  # Emit the signal with the viseme ID
    
    def subscriber_callback2(self, msg):
    #    self.get_logger().info(f'Viseme Received Mouth Shape: "{msg.data}"')
        if msg.data == 10: #IF IT IS THE AWAITING FACE
            self.ignore_microsoft = True
            awaiting_viseme = 22
            self.viseme_signal.emit(awaiting_viseme)  # Emit the signal with the viseme ID

        self.ignore_microsoft = False

    def update_face(self, viseme_id):
        filename = self.viseme_map.get(viseme_id)

        if filename is None:
            self.get_logger().error(f"Viseme ID {viseme_id} is not mapped to any image.")
            return

        file_path = os.path.join(self.faces_path, filename)
        self.pixmap = QPixmap(file_path)
        if self.pixmap.isNull():
            self.get_logger().error(f"Failed to load image: {file_path}")
            return

        self.image_label.setPixmap(self.pixmap)

        QApplication.processEvents()  # Force UI update


def main(args=None):
    rclpy.init(args=args)
    node = VisemeRenderer()

    def ros_spin():
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)

    ros_thread = threading.Thread(target=ros_spin, daemon=True)
    ros_thread.start()

    node.get_logger().info("ROS 2 node running in a separate thread.")

    try:
        node.app.exec()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
