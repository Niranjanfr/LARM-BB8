#!/usr/bin/env python
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
import math
import numpy as np
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from geometry_msgs.msg import Point

class MarkerPublisher(Node):
    def __init__(self):
        super().__init__('marker_publisher')
        self.publisher = self.create_publisher(MarkerArray, 'marker_array_topic', 10)
        
        # Créer un MarkerArray
        self.marker_array = MarkerArray()

        #Créate a subscriber qui récupère la position du robot dans la carte

        self.odom_data = 0

        self.subs_position_robot = self.create_subscription(
            Odometry, '/odom',
            self.listener_callback, 10)
        
        #create subscriber pour les coordonées de la bouteille

        self.subs_coord_nuke = self.create_subscription(
            Point, '/coordonnee_objet_ref_robot',
            self.get_objt_coord, 10
        )
        self.nuke_coord = 0
        
        # # Ajouter des marqueurs avec des coordonnées spécifiques
        # self.add_marker(1, 0.0, 0.0, 0.0)
        # self.add_marker(2, 1.0, 1.0, 1.0)
        # self.add_marker(3, -1.0, -1.0, 1.0)

        # Publier le MarkerArray
        self.publish_markers()

    def listener_callback(self, msgs):

        self.get_logger().info('I receive: "%s"' %
                               str(self.odom_data))
        
        self.odom_data = msgs

    def position_robot(self):
        position = self.odom_data.pose.pose.position
        orientation = self.odom_data.pose.pose.orientation
        (posx, posy, posz) = (position.x, position.y, position.z)
        (qx, qy, qz, qw) = (orientation.x, orientation.y, orientation.z, orientation.w)

        print("robot : ", posx, posy, posz)
        return posx,posy,qz
    
    def get_objt_coord(self, msg):

        self.nuke_coord = msg
        print (" Objt coordonnée = " ,self.nuke_coord )


    def transform_coordinates(self):

        #coordonnées robot
        x_r, y_r, theta_r = self.position_robot()

        #coordonnées nuke_cola
        x_obj, y_obj = self.nuke_coord

        # Transformation matrix from robot frame to world frame
        transformation_matrix = np.array([
            [np.cos(theta_r), -np.sin(theta_r), x_r],
            [np.sin(theta_r), np.cos(theta_r), y_r],
            [0, 0, 1]
        ])

        # Homogeneous coordinates of the object in robot frame
        object_homogeneous = np.array([x_obj, y_obj, 1])

        # Transform object coordinates to world frame
        transformed_coordinates = np.dot(transformation_matrix, object_homogeneous)[:2]

        return tuple(transformed_coordinates)

        
    


    def add_marker(self):
        marker = Marker()
        marker.header.frame_id = "base_link"  # Le frame_id dans lequel les coordonnées sont définies
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'my_namespace'

        x, y, z = self.transform_coordinates()
        id = 0
        for m in self.marker_array.markers:
            m.id = id
            id += 1

        marker.id = id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = z
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0

        marker_identique = False
        for m in self.marker_array.markers:
            if m.pose.position.x == marker.pose.position.x  and m.pose.position.y == marker.pose.position.y: 
                 marker_identique = True
            else: 
                self.marker_array.markers.append(marker)

    def publish_markers(self):
        self.publisher.publish(self.marker_array)
        self.get_logger().info('MarkerArray published')

def main(args=None):
    rclpy.init(args=args)
    marker_publisher = MarkerPublisher()
    rclpy.spin(marker_publisher)
    marker_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

# topic = 'visualization_marker_array'
# publisher = rospy.Publisher(topic, MarkerArray)

# rospy.init_node('register')

# markerArray = MarkerArray()

# count = 0
# MARKERS_MAX = 100

# while not rospy.is_shutdown():

#     marker = Marker()
#     marker.header.frame_id = "/base_link"
#     marker.type = marker.SPHERE
#     marker.action = marker.ADD
#     marker.scale.x = 0.2
#     marker.scale.y = 0.2
#     marker.scale.z = 0.2
#     marker.color.a = 1.0
#     marker.color.r = 1.0
#     marker.color.g = 1.0
#     marker.color.b = 0.0
#     marker.pose.orientation.w = 1.0
#     marker.pose.position.x = math.cos(count / 50.0) #position de la bouteille 
#     marker.pose.position.y = math.cos(count / 40.0) 
#     marker.pose.position.z = math.cos(count / 30.0) 

    # We add the new marker to the MarkerArray, removing the oldest marker from it when necessary
    # if(count > MARKERS_MAX):
    #     markerArray.markers.pop()

    # markerArray.markers.append(marker)

    # # Renumber the marker IDs to ensure uniqueness
    # id = 0
    # for m in markerArray.markers:
    #     m.id = id
    #     id += 1

    # # Publish the MarkerArray
    # publisher.publish(markerArray)

    # count += 1

    # rospy.sleep(0.01)