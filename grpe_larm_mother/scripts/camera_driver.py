#!/usr/bin/env python3
import pyrealsense2 as rs
import signal, time, numpy as np
import sys, cv2, rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from std_msgs.msg import Header
from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_msgs.msg import Float64
from geometry_msgs.msg import Point

import math


colorizer = rs.colorizer()
# Realsense Node:
class Realsense(Node):
    def __init__(self, fps= 60):
        super().__init__('realsense')

        self.isOk= True

        # Configure depth and color streams
        self.pipeline = rs.pipeline()
        self.config = rs.config()

        # frames = self.pipeline.wait_for_frames()
        # frames.poll_frames()
        # self.depth_frame = frames.get_depth_frame()

        # Get device product line for setting a supporting resolution
        pipeline_wrapper = rs.pipeline_wrapper(self.pipeline)
        pipeline_profile = self.config.resolve(pipeline_wrapper)
        self.device = pipeline_profile.get_device()
        self.device_product_line = str(self.device.get_info(rs.camera_info.product_line))

        self.config.enable_stream(rs.stream.color, 848, 480, rs.format.bgr8, 60)
        self.config.enable_stream(rs.stream.depth, 848, 480, rs.format.z16, 60)

        self.image_publisher = self.create_publisher(Image,"image_raw_image",10)
        self.depth_publisher = self.create_publisher(Image,"image_raw_depth",10)

        self.trouver = self.create_publisher(String, 'Objet_trouve', 10)
        # self.depth_object = self.create_publisher(Float64, 'distance_object',10)
        self.coord_xy_obj = self.create_publisher(Point,'coordonnee_objet_ref_robot',10)


        # self.rsNode_2 = DepthCalculator()

        align_to = rs.stream.depth
        self.align = rs.align(align_to)




        # Start strsNode_2reaming
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
        depth_stream = frames.first(rs.stream.depth)

        aligned_frames =  self.align.process(frames)
        self.depth_frame = aligned_frames.get_depth_frame()
        self.aligned_color_frame = aligned_frames.get_color_frame()
        
        
        # Convert images to numpy arrays
        self.depth_image = np.asanyarray(self.depth_frame.get_data())
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

    def gestion_image(self):

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
        conversion = self.bridge.imgmsg_to_cv2(img_msg=msg_image,desired_encoding='passthrough')


        color=65

        lo=np.array([color-15, 100, 50])
        hi=np.array([color+15, 255,255])

        color_info=(0, 0, 255)

        cv2.namedWindow('Camera')
        hsv_px = [47,142,120]

        kernel = np.ones((3, 3), np.uint8)

        frame=conversion
        image=cv2.cvtColor(conversion, cv2.COLOR_BGR2HSV)
        mask=cv2.inRange(image, lo, hi)
        mask=cv2.erode(mask, kernel, iterations=1)
        mask=cv2.dilate(mask, kernel, iterations=1)
        image2=cv2.bitwise_and(frame, frame, mask= mask)
        cv2.putText(frame, "Couleur: {:d}".format(color), (10, 30), cv2.FONT_HERSHEY_DUPLEX, 1, color_info, 1, cv2.LINE_AA)

        # Affichage des composantes HSV sous la souris sur l'image
        pixel_hsv = " ".join(str(values) for values in hsv_px)
        font = cv2.FONT_HERSHEY_SIMPLEX
        cv2.putText(frame, "px HSV: "+pixel_hsv, (10, 260),
                font, 1, (255, 255, 255), 1, cv2.LINE_AA)

        # Flouttage de l'image
        image=cv2.blur(image, (7, 7))
        # Erosion d'un mask
        mask=cv2.erode(mask, None, iterations=4)
        # dilatation d'un mask
        mask=cv2.dilate(mask, None, iterations=4)

        msg = String()
        msg.data = ' Bouteille trouvée '
        elements=cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
        if len(elements) > 0:
            c=max(elements, key=cv2.contourArea)
            ((x, y), rayon)=cv2.minEnclosingCircle(c)
            elements.pop(c)
            c1 = max (elements, key=cv2.contourArea)
            ((x1, y1), rayon1)=cv2.minEnclosingCircle(c1)
            if (rayon<60 and rayon>40) and (rayon1 <60 and rayon1 > 40):

                color_intrin = self.aligned_color_frame.profile.as_video_stream_profile().intrinsics

                # objet 1
                depth = self.depth_frame.get_distance(int(x), int(y))
                dx ,dy, dz = rs.rs2_deproject_pixel_to_point(color_intrin, [x,y], depth)
                distance = math.sqrt(((dx)**2) + ((dy)**2) + ((dz)**2))

                #objet 2
                depth1 = self.depth_frame.get_distance(int(x1), int(y1))
                dx1 ,dy1, dz1 = rs.rs2_deproject_pixel_to_point(color_intrin, [x1,y1], depth1)
                distance1 = math.sqrt(((dx1)**2) + ((dy1)**2) + ((dz1)**2))

                if distance >1.0:



                    #Calcul des coordonnées de l'object par rapport a la camera
                    coord_obj = Point()
                    coord_obj.x = dz
                    coord_obj.y = -dx

                    print(coord_obj)
                    self.coord_xy_obj.publish(coord_obj)

                if distance1 >1.0:

                    #Calcul des coordonnées de l'object par rapport a la camera
                    coord_obj1 = Point()
                    coord_obj1.x = dz1
                    coord_obj1.y = -dx1

                    print(coord_obj1)
                    self.coord_xy_obj.publish(coord_obj1)


                cv2.circle(image2, (int(x), int(y)), int(rayon), color_info, 2)
                cv2.circle(frame, (int(x), int(y)), 5, color_info, 10)
                cv2.line(frame, (int(x), int(y)), (int(x)+150, int(y)), color_info, 2)
                cv2.putText(frame, "Objet !!!", (int(x)+10, int(y) -10), cv2.FONT_HERSHEY_DUPLEX, 1, color_info, 1, cv2.LINE_AA)
                self.trouver.publish(msg)
        cv2.imshow('Camera', frame)
        cv2.imshow('image2', image2) # si nécessaire décommanter les lignes
        cv2.imshow('Mask', mask)


        cv2.waitKey(10)
        pass

    def px_coo_to_angle(self, px_coo, device):
    #data specifiques pour chaque cameras
        if device == "camera":
            hfov = 69
            vfov = 42
            hpx = 848
            vpx = 480
        if device == "depth":
            hfov = 87
            vfov = 58
            hpx = 848
            vpx = 480

        # Un angle c'est un Point car tamer
        # angle.x est l'angle longitudinal qui nous intéresse calculé par rapport au pixel de camera
        # angle.y est l'angle de la hauteur 
        if hfov is not None:
            angle = Point()
            angle.x = (px_coo.x - hpx/2)/hpx*hfov/2 # calcul : coordonée 0 mise au centre pour avoir +- largeur image en coo
            angle.y = (px_coo.y - vpx/2)/vpx*vfov/2 # coo ratoinalisée par la largeur totale multipliée par angle fov/2 pour chaque côté
            angle.z = 0.0 # Zéro
            return angle
        return None #apas marché car device pas reconnu
    
    # def calcul_coord_object(self, distance)

    def signalInteruption(self, signum, frame):       
        print( "\nCtrl-c pressed" )
        self.isOk= False



# Node processes:
def main (args=None):
    rclpy.init(args=args)
    rsNode= Realsense()


    while rsNode.isOk:
        rsNode.read_imgs()
        rsNode.gestion_image()
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