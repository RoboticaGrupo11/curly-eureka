#!/usr/bin/env python3
import rospkg
import rospy
import time

from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from pytictoc import TicToc
from threading import Thread
from mi_bot_11.srv import replay_tour_srv

class Punto4:

    def __init__(self) -> None:
        #Publishers
        self.cmdPublisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # Service server
        self.replayTourService = rospy.Service('turtle_bot_player/replay_tour_srv', replay_tour_srv, self.callback_replay_tour_srv)

        # Constants
        self.VEL_LINEAR = 10.0
        self.VEL_ANGULAR = 10.0
        self.RATE = rospy.Rate(10)
      
     
    
    def callback_replay_tour_srv(self, req):
        """
        Callback for the replay tour service: Calls the replay tour function in a new thread.
        """
        self.PATH = req.route+'/'   
        
        #Constants
        self.PATH_TOURS =  self.PATH+'tours.txt'
        
        thread = Thread(target=self.move)
        thread.start()
        return 'approved'

    def move(self):

        file=open(self.PATH_TOURS,'r').read().split(",")[1:]
        for key in file:
            twistMessage = Twist()
            if key == 'w':
                twistMessage.linear.x = 2.0
            elif key == 'q':
                twistMessage.angular.z = 10.0
            elif key == 's':
                twistMessage.linear.x = -2.0
            elif key == 'e':
                twistMessage.angular.z = -10.0
            self.cmdPublisher.publish(twistMessage)
            self.writeFile(self.PATH+'temp.txt', ','+key)
            self.RATE.sleep()
    
    def writeFile(self, path, msg):
        """
        Open a file, write the message that is received by parameter and close the file.
        """
        file = open(path, 'a')
        file.write(msg)
        file.close()


if __name__ == '__main__':
    rospy.init_node('turtle_bot_player')
    punto4 = Punto4()
    rospy.spin()
    


