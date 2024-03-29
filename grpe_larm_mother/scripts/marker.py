#!/usr/bin/env python3
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
import math as math
import numpy as np
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from geometry_msgs.msg import Point
import signal

class MarkerPublisher(Node):
    def __init__(self):
        super().__init__('marker_publisher')
        self.publisher = self.create_publisher(MarkerArray, 'marker_array_topic', 10)
        
        # Créer un MarkerArray
        self.marker_array = MarkerArray()
        self.isOk = True

        #Créate a subscriber qui récupère la position du robot dans la carte

        self.subs_position_robot = self.create_subscription(
            Odometry, '/odom',
            self.listener_callback, 10)
        
        #create subscriber pour les coordonées de la bouteille

        self.subs_coord_nuke = self.create_subscription(
            Point, '/coordonnee_objet_ref_robot',
            self.get_objt_coord, 10
        )
        self.nuke_coord =Point()
        self.detection = False

        # Publier le MarkerArray
        self.publish_markers()

    def listener_callback(self, msgs):
        self.odom_data = msgs

    # extrait la position et l'orientation du robot 
    def position_robot(self):
        position = self.odom_data.pose.pose.position
        print(position)
        orientation = self.odom_data.pose.pose.orientation
        print(orientation)
        (posx, posy, posz) = (position.x, position.y, position.z)
        (qx, qy, qz, qw) = (orientation.x, orientation.y, orientation.z, orientation.w)
        return posx,posy,qz
    
    def get_objt_coord(self, msg):
        self.nuke_coord = msg
        if self.nuke_coord == msg : 
            self.detection = True

    # Transforme les coordonées de la bouteille dans le référentiel de la map 
    def transform_coordinates(self):
        #coordonnées robot
        x_r, y_r, theta_r = self.position_robot()

        #coordonnées nuke_cola
        x_obj= self.nuke_coord.x
        y_obj = self.nuke_coord.y


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

    # méthode qui crée et ajoute un marqueur dans un Markerarray
    def add_marker(self):
        print("add_marker")
    
        marker = Marker()
        marker.header.frame_id = "map"  # Le frame_id dans lequel les coordonnées sont définies
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'my_namespace'

        x, y = self.transform_coordinates()
        id = 0
        for m in self.marker_array.markers:
            m.id = id
            id += 1

        marker.id = id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
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

        self.marker_array.markers.append(marker)

        
    def mark(self):
        if self.detection == True  and self.odom_data != None: # Dans le soucis de synchroniser les informations
            self.add_marker()                                  # reçues nous avons crée une fonction mark qui appelle l'ajout du marqueur
            self.publish_markers()                             # et la publication du marker_array que quand le noeud reçoit les infos issues
            self.detection = False                             # de la position du robot à travers l'odom et détecte la position des objets

    # publication de marker_array
    def publish_markers(self):
        self.publisher.publish(self.marker_array)
        self.get_logger().info('MarkerArray published')

    # arrêt du noeud 
    def signalInteruption(self):       
        print( "\nCtrl-c pressed" )
        self.isOk= False

def main(args=None):
    rclpy.init(args=args)
    marker_publisher = MarkerPublisher()

    while marker_publisher.isOk:
        marker_publisher.mark()
        rclpy.spin_once (marker_publisher)

    #stop streaming
    signal.signal(signal.SIGINT, marker_publisher.signalInteruption)
    marker_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
