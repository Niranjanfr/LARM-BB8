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




# rosNode=None

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
        #global rosNode
        obstacles = []
        # self.get_logger().info( f"scan:\n{scanMsg}" )
        
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
        longeur_rectangle = 0.9
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
                self.continue_tourner_gauche = True
                print("obstacle_droite")
                break
            #obstacles gauche
            elif ( (x_ref <= obstacles[i].x <= x_ref + longeur_rectangle) 
                  and  ((y_ref - (largeur_rectangle/2.0) <= obstacles[i].y < y_ref))
            ):     
               self.obstacle_gauche = True
               self.continue_tourner_droite = True
               print("obstacle_gauche")
               break
            # elif ((x_ref <= obstacles[i].x <= x_ref + longeur_rectangle)
            #     and  (y_ref <= obstacles[i].y <= y_ref + (largeur_rectangle/2.0))
            #     and (x_ref <= obstacles[i].x <= x_ref + longeur_rectangle) 
            #       and  ((y_ref - (largeur_rectangle/2.0) <= obstacles[i].y < y_ref) )
            # ):
            #     self.
        
        

        sample= [ [ round (p.x, 2), round(p.y, 2) ] for p in  obstacles[10:20] ]
        
        # self.get_logger().info( f" obs({len(obstacles)}) ...{sample}..." )

        #publication du message de vélocité

        self.publishe(self.obstacle_gauche,self.obstacle_droite)

    # def test_obstacle(self):

    #     if self.continue_tourner_gauche:
    #         velo = Twist()
    #         velo.linear.x= 0.0   # meter per second
    #         velo.angular.z= 1.7 # radian per second
    #         self.velocityPub.publish(velo)
    #         self.scan_callback()

    #     elif self.continue_tourner_droite:
    #         velo = Twist()
    #         velo.linear.x= 0.0   # meter per second
    #         velo.angular.z= -1.7 # radian per second
    #         self.velocityPub.publish(velo)
    #         self.scan_callback()
    # tourner_gauche = False
    # tourner_droite = False

    def publishe(self, obstacle_gauche, obstacle_droite):
        
        # global tourner_droite
        # global tourner_gauche
        
        R_VA=random.randrange(10, 17, 1)

        if (self.VL == 0.0):
            if not obstacle_gauche and not obstacle_droite:
                self.VL = 0.2
                self.VA = 0.0
        else:
            if obstacle_droite :
                self.VL = 0.0
                self.VA = R_VA/10
                # tourner_gauche = True
                # tourner_droite = False
            elif obstacle_gauche:
                self.VL = 0.0
                self.VA = -R_VA/10
                # tourner_droite = True
                # tourner_gauche = False

        print(self.VL, self.VA)
        velo = Twist()
        velo.linear.x= self.VL   # meter per second
        velo.angular.z= self.VA # radian per second
        self.velocityPub.publish(velo)


        # if obstacle_gauche :
        #     # ang_speed = random.random()
        #     # velo = Twist()
        #     # velo.angular.z= ang_speed # radian per second
        #     # self.velocityPub.publish(velo)
        #     # tourner_droite= True

        # # elif obstacle_droite and tourner_droite :
        # #     velo = Twist()
        # #     velo.angular.z= 1.0 # radian per second
        # #     self.velocityPub.publish(velo)
        # #     tourner_droite= False

        #     while obstacle_gauche or obstacle_droite : 
        #         velo = Twist()
        #         velo.angular.z= 1.0 # radian per second
        #         self.velocityPub.publish(velo)
        #         if not obstacle_gauche or not obstacle_droite:
        #             break

        #     # if tourner_droite and obstacle_droite:
        #     #     velo = Twist()
        #     #     velo.angular.z= 1.0 # radian per second
        #     #     self.velocityPub.publish(velo)
        #     #     tourner_droite = True

        #     # tourner_droite = False
            
        #     # if obstacle_droite == False and obstacle_gauche == False :
        #     #     while self.obstacle_droite==False:
        #     #         self.avancer_arc_cercle(0.5)
        #     #         self.scan_callback()




        # elif obstacle_droite : 
        #     # ang_speed = random.random()
        #     # velo = Twist()
        #     # velo.angular.z= -ang_speed # radian per second
        #     # self.velocityPub.publish(velo)
        #     # tourner_gauche = True

        #     while obstacle_droite or obstacle_gauche : 
        #         velo = Twist()
        #         velo.angular.z= -1.0 # radian per second
        #         self.velocityPub.publish(velo)
        #         if not obstacle_gauche or not obstacle_droite:
        #             break

        # # elif tourner_gauche and obstacle_gauche:
        # #     velo = Twist()
        # #     velo.angular.z= -1.0 # radian per second
        # #     self.velocityPub.publish(velo)
        # #     tourner_gauche = False

        #     # tourner_gauche = False


        #     # if obstacle_droite == False and obstacle_gauche == False :
        #     #     while self.obstacle_gauche==False:
        #     #         self.avancer_arc_cercle(-0.5)
        #     #         self.scan_callback()          
            

        # elif obstacle_gauche and obstacle_droite:
        #     while obstacle_droite and obstacle_gauche : 
        #         velo = Twist()
        #         velo.angular.z= -1.0 # radian per second
        #         self.velocityPub.publish(velo)
        #         if not obstacle_gauche :
        #             break


        # else: 
        #     self.avancer()



    # def avancer(self):

    #     velo = Twist()
    #     velo.linear.x= 0.2   # meter per second
    #     velo.angular.z= 0.0 # radian per second
    #     self.velocityPub.publish(velo)
    
    # def avancer_arc_cercle(self,rotation):

    #     velo = Twist()
    #     velo.linear.x= 0.2   # meter per second
    #     velo.angular.z= rotation # radian per second
    #     self.velocityPub.publish(velo)


    
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
#         rclpy.spin(self._node)des code
#         # Clean stop
#         self._node.minimal_publisher.destroy_node()
#         rclpy.shutdown()



#self.velocity_publisher_.publish(msg)
     

def main():
    print('Move move move !')

    rclpy.init()
    rosNode= MSubscriber("move_node")

    rclpy.spin( rosNode )
    rosNode.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__' :
    main()
