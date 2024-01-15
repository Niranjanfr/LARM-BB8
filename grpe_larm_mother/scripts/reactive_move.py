#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math
from geometry_msgs.msg import Point32
import sensor_msgs_py.point_cloud2
from std_msgs.msg import Header
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import random




class MSubscriber(Node):

    def __init__(self, node_name):
        super().__init__(node_name)
        self.subscription= self.create_subscription(
            LaserScan, 'scan',
            self.scan_callback, 10)
        self.velocityPub = self.create_publisher(Twist, '/multi/cmd_nav', 10)

        self.VL = 0.0
        self.VA = 1.0

    def scan_callback(self, scanMsg):
        obstacles = []
        
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
                
        x_ref = 0.0
        y_ref = 0.0
        largeur_rectangle = 0.5
        longeur_rectangle = 0.8
        self.obstacle_droite = False
        self.obstacle_gauche = False
        self.continue_tourner_gauche = False
        self.continue_tourner_droite = False
        for i in range (len(obstacles)):
            #obstacles droite
            if ( (x_ref <= obstacles[i].x <= x_ref + longeur_rectangle)
                and  (y_ref <= obstacles[i].y <= y_ref + (largeur_rectangle/2.0)) 
            ):
                self.obstacle_droite = True
                break
            elif ( (x_ref <= obstacles[i].x <= x_ref + longeur_rectangle) 
                  and  ((y_ref - (largeur_rectangle/2.0) <= obstacles[i].y < y_ref))
            ):     
               self.obstacle_gauche = True
               break
        
        

        sample= [ [ round (p.x, 2), round(p.y, 2) ] for p in  obstacles[10:20] ]
        

        #publication du message de vélocité

        self.publishe(self.obstacle_gauche,self.obstacle_droite)

    def publishe(self, obstacle_gauche, obstacle_droite):
        
        R_VA=random.randrange(10, 14, 1)

        if (self.VL == 0.0):
            if not obstacle_gauche and not obstacle_droite:
                self.VL = 0.25
                self.VA = 0.0
        else:
            if obstacle_droite :
                self.VL = 0.0
                self.VA = -R_VA/10
            elif obstacle_gauche:
                self.VL = 0.0
                self.VA = R_VA/10


        velo = Twist()
        velo.linear.x= self.VL   # meter per second
        velo.angular.z= self.VA # radian per second
        self.velocityPub.publish(velo)
     

def main():
    print('Move move move !')

    rclpy.init()
    rosNode= MSubscriber("move_node")

    rclpy.spin( rosNode )
    rosNode.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__' :
    main()
