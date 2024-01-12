#!/usr/bin/env python3
import pyrealsense2 as rs
import signal, time, numpy as np
import sys, cv2, rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from std_msgs.msg import Header
from sensor_msgs.msg import Image

# Realsense Node:
class Realsense(Node):
    def __init__(self, fps= 60):
        super().__init__('realsense')

        self.isOk= True

        # Configure depth and color streams
        self.pipeline = rs.pipeline()
        self.config = rs.config()

        # Get device product line for setting a supporting resolution
        pipeline_wrapper = rs.pipeline_wrapper(self.pipeline)
        pipeline_profile = self.config.resolve(pipeline_wrapper)
        self.device = pipeline_profile.get_device()
        self.device_product_line = str(self.device.get_info(rs.camera_info.product_line))

        self.config.enable_stream(rs.stream.color, 848, 480, rs.format.bgr8, 60)
        self.config.enable_stream(rs.stream.depth, 848, 480, rs.format.z16, 60)

        self.image_publisher = self.create_publisher(Image,"image_raw",10)
        self.depth_publisher = self.create_publisher(Image,"image_raw",10)


        # Start streaming
        self.pipeline.start(self.config)

    def read_imgs(self):


        print( f"Connect: {self.device_product_line}" )
        found_rgb = True
        for s in self.device.sensors:
            print( "Name:" + s.get_info(rs.camera_info.name) )
            if s.get_info(rs.camera_info.name) == 'RGB Camera':
                found_rgb = True

        if not (found_rgb):
            print("Depth camera equired !!!")
            exit(0)

        freq= 60

        sys.stdout.write("-")


        # Wait for a coherent tuple of frames: depth, color and accel
        frames = self.pipeline.wait_for_frames()

        color_frame = frames.first(rs.stream.color)
        depth_frame = frames.first(rs.stream.depth)
        
        
        # Convert images to numpy arrays
        self.depth_image = np.asanyarray(depth_frame.get_data())
        self.color_image = np.asanyarray(color_frame.get_data())

        # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(self.depth_image, alpha=0.03), cv2.COLORMAP_JET)

        depth_colormap_dim = depth_colormap.shape
        color_colormap_dim = self.color_image.shape

        sys.stdout.write( f"\r- {color_colormap_dim} - {depth_colormap_dim} - ({round(freq)} fps)" )

        # Show images
        images = np.hstack((self.color_image, depth_colormap))

        # Show images
        cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
        cv2.imshow('RealSense', images)
        cv2.waitKey(1)
        pass

    def publish_imgs(self):


        self.bridge=CvBridge()

        # Utilisation de colormap sur l'image depth de la Realsense (image convertie en 8-bit par pixel)
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(self.depth_image, alpha=0.03), cv2.COLORMAP_JET)

        self.bridge=CvBridge()


        msg_image = self.bridge.cv2_to_imgmsg(self.color_image,"bgr8")
        msg_image.header.stamp = self.get_clock().now().to_msg()
        msg_image.header.frame_id = "image"
        self.image_publisher.publish(msg_image)

        msg_depth = self.bridge.cv2_to_imgmsg(depth_colormap,"bgr8")
        msg_depth.header.stamp = msg_image.header.stamp
        msg_depth.header.frame_id = "depth"
        self.depth_publisher.publish(msg_depth)
        pass

    def signalInteruption(signum, frame):       
        print( "\nCtrl-c pressed" )
        self.isOk= False



# Node processes:
def main (args=None):
    rclpy.init(args=args)
    rsNode= Realsense()
    while rsNode.isOk:
        rsNode.read_imgs()
        rsNode.publish_imgs()
        rclpy.spin_once(rsNode, timeout_sec=0.001)
    # Stop streaming
    signal.signal(signal.SIGINT, rsNode.signalInteruption)
    print("Ending...")
    rsNode.pipeline.stop()
    # Clean end
    rsNode.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()