#!python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math
from geometry_msgs.msg import Point32
import sensor_msgs_py.point_cloud2
from std_msgs.msg import Header


rosNode=None
pointcloud = None

def scan_callback(scanMsg):
    global rosNode
    global pointcloud
    rosNode.get_logger().info( f"scan:\n{scanMsg}" )
    return sensor_msgs_py.point_cloud2.create_cloud_xyz32(Header(frame_id='laser'), scanpoint(scanMsg))
    

def scanpoint(scanMsg):
    obstacles= []
    angle= scanMsg.angle_min
    for aDistance in scanMsg.ranges :
        if 0.1 < aDistance and aDistance < 5.0 :
            aPoint= [
                math.cos(angle) * aDistance,
                math.sin(angle) * aDistance,
                0
            ]
            obstacles.append(aPoint)
        angle+= scanMsg.angle_increment

    return obstacles

    # sample= [ [ round(p.x, 2), round(p.y, 2) ] for p in  obstacles[10:20] ]
    
    # rosNode.get_logger().info( f" obs({len(obstacles)}) ...{sample}..." )



    #print(len(obstacles))

    #pointcloud = sensor_msgs_py.point_cloud2.create_cloud_xyz32(Header(frame_id='laser'), obstacles)
    #for point in sensor_msgs_py.point_cloud2.read_points(pointcloud):
    #    print(point)
        


        

     

def main():
    global rosNode
    print('Move move move !')
    # Initialize a Publisher:
    myNode= Node('point_node') # Create a Node, with a name         
    point_publisher = myNode.create_publisher(Point32, '/scan_points', 10)

    # publish a msg
    velo = scan_callback(scanMsg)
    velo.linear.x= 0.2   # meter per second
    velo.angular.z= 0.5 # radian per second
    velocity_publisher.publish(velo)

    rclpy.init()
    rosNode= Node('scan_interpreter')
    rosNode.create_subscription( LaserScan, 'scan', scan_callback, 10)

    

    while True :
        rclpy.spin_once( rosNode )
    rosNode.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__' :
    main()