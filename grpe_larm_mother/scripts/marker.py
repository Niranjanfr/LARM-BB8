#!/usr/bin/env python
 
import roslib; roslib.load_manifest('visualization_marker_tutorials')
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
import rospy
import math
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import Odometry
from std_msgs.msg import String

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
        
        # Ajouter des marqueurs avec des coordonnées spécifiques
        self.add_marker(1, 0.0, 0.0, 0.0)
        self.add_marker(2, 1.0, 1.0, 1.0)
        self.add_marker(3, -1.0, -1.0, 1.0)

        # Publier le MarkerArray
        self.publish_markers()

    def listener_callback(self, msgs):

        self.get_logger().info('I receive: "%s"' %
                               str(self.odom_data))
        
        self.odom_data = msgs
        
    def position_robot(self):
        position = self.odom_data.pose.pose.position
        (posx, posy, posz) = (position.x, position.y, position.z)

        return posx,posy,posz


    def add_marker(self, id, x, y, z):
        marker = Marker()
        marker.header.frame_id = "base_link"  # Le frame_id dans lequel les coordonnées sont définies
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'my_namespace'
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