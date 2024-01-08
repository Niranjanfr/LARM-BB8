#!python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math
from geometry_msgs.msg import Point32
import sensor_msgs_py.point_cloud2
from std_msgs.msg import Header
from std_msgs.msg import String
from geometry_msgs.msg import Twist



# rosNode=None

class MSubscriber(Node):

    def __init__(self, node_name):
        super().__init__(node_name)
        self.subscription= self.create_subscription(
            LaserScan, 'scan',
            self.scan_callback, 10)
        self.velocityPub = self.create_publisher(Twist, '/cmd_vel', 10)

    def scan_callback(self, scanMsg):
        #global rosNode
        obstacles = []
        rosNode.get_logger().info( f"scan:\n{scanMsg}" )
        
        angle= scanMsg.angle_min
        for aDistance in scanMsg.ranges :
            if 0.1 < aDistance and aDistance < 5.0 :
                aPoint= Point32()
                aPoint.x= (float)(math.cos(angle) * aDistance)
                aPoint.y= (float)(math.sin( angle ) * aDistance)
                aPoint.z= (float)(0)  
                obstacles.append(aPoint)
            angle+= scanMsg.angle_increment

        #DÉTECTION DES OBSTACLES
                
        x_ref = 0.1
        y_ref = 0.1
        largeur_rectangle = 0.6
        longeur_rectangle = 0.7
        obstacle_droite = False
        obstacle_gauche = False
        for i in range (len(obstacles)):
            #obstacles droite
            if ( (x_ref <= obstacles[i].x and obstacles[i].x <= x_ref + longeur_rectangle)
                and  (y_ref <= obstacles[i].y and obstacles[i].y <= y_ref + (largeur_rectangle/2.0)) 
            ):
                obstacle_droite = True
            #obstacles gauche
            elif ( (x_ref <= obstacles[i].x and obstacles[i].x <= x_ref + longeur_rectangle) 
                  and  ((y_ref- (largeur_rectangle/2) <= obstacles[i].y and obstacles[i].y < y_ref))
            ):     
               obstacle_gauche = True
        
        

        sample= [ [ round(p.x, 2), round(p.y, 2) ] for p in  obstacles[10:20] ]
        
        rosNode.get_logger().info( f" obs({len(obstacles)}) ...{sample}..." )

        #publication du message de vélocité

        self.publishe(obstacle_gauche,obstacle_droite)

    def publishe(self, obstacle_gauche, obstacle_droite):
        if obstacle_gauche :
            velo = Twist()
            velo.linear.x= 0.2   # meter per second
            velo.angular.z= -0.5 # radian per second
            self.velocityPub.publish(velo) 

        elif obstacle_droite: 
            velo = Twist()
            velo.linear.x= 0.2   # meter per second
            velo.angular.z= 0.5 # radian per second
            self.velocityPub.publish(velo)

        elif obstacle_gauche and obstacle_droite: 
            velo = Twist()
            velo.linear.x= 0.2   # meter per second
            velo.angular.z= 0.7 # radian per second
            self.velocityPub.publish(velo)

        else : 
            velo = Twist()
            velo.linear.x= 0.2   # meter per second
            velo.angular.z= 0.0 # radian per second
            self.velocityPub.publish(velo)


    
# class MPublisher:

#     # def __init__(self):
#     #     self.i = 0
    
#     # def timer_callback(self):
#     #     msg = String()
#     #     msg.data = 'Hello World: %d' % self.i
#     #     self._node.publisher_.publish(msg)
#     #     self._node.get_logger().info('Publishing: "%s"' % msg.data)
#     #     self.i += 1

#     def process(self):
#         rclpy.init()
#         self._node= Node()
#         # Create a publisher
#         self._publisher= self._node.create_publisher(Twist, 'topic', 10)
#         # Create a timer at 0.5 hertz, with a callback
#         # self._timer = self._node.create_timer(0.5, self.timer_callback)
#         # Go
#         rclpy.spin(self._node)
#         # Clean stop
#         self._node.minimal_publisher.destroy_node()
#         rclpy.shutdown()



#self.velocity_publisher_.publish(msg)
     

def main():
    global rosNode
    print('Move move move !')

    rclpy.init()
    rosNode= MSubscriber("move_node")

    rclpy.spin( rosNode )
    rosNode.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__' :
    main()

