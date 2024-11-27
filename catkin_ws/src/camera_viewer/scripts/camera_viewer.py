#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np
from threading import Lock

class CameraViewer:
    def __init__(self):
        rospy.init_node('camera_viewer')
        
        # Buffer for latest image
        self.latest_image = None
        self.image_lock = Lock()
        
        # Subscribe with a larger queue for smoother streaming
        self.sub = rospy.Subscriber(
            '/usb_cam/image_raw/compressed',
            CompressedImage,
            self.image_callback,
            queue_size=10,
            buff_size=2**24  # Larger buffer for network packets
        )
        
        rospy.loginfo("Camera viewer started. Press 'q' to exit.")

    def image_callback(self, msg):
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            with self.image_lock:
                self.latest_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        except Exception as e:
            rospy.logerr(f"Error processing image: {e}")

    def run(self):
        rate = rospy.Rate(30)  # 30 Hz display rate
        
        while not rospy.is_shutdown():
            with self.image_lock:
                if self.latest_image is not None:
                    cv2.imshow('Camera View', self.latest_image)
                    
            # Check for 'q' key to quit
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
                
            rate.sleep()
        
        cv2.destroyAllWindows()

if __name__ == '__main__':
    viewer = CameraViewer()
    viewer.run()