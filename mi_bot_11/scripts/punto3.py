#!/usr/bin/env python3
import rospy
import os
from pynput import keyboard as kb
from mi_bot_11.srv import save_route_srv
from std_msgs.msg import Float32, Empty
from geometry_msgs.msg import Twist
from threading import Thread


class Punto3:

    # -----------------------------------------------------INIT--------------------------------------------------------------

    def __init__(self) -> None:
        #Publishers
        self.cmdPublisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
        rospy.wait_for_service('/save_route')
        self.saveRouteClient = rospy.ServiceProxy('/save_route', save_route_srv)

        #Constants
        self.PATH = self.saveRouteClient.call().path
        os.makedirs(os.path.dirname(self.PATH+'/'), exist_ok=True)
        
        self.PATH_TOURS = self.PATH+'/tours.txt'
        
        self.RATE = rospy.Rate(10)
        
        #Attributes
        self.keyPressed = ""

        thread = Thread(target=self.saveInformation)
        thread.start()

 

    def pressKey(self, key):
        """
        Callback for kb.LIstener: It is called when a key is pressed and depending on the key that is pressed
        a Twist message is published in the cmd_vel topic. Invoke the function 'saveInformation'.
        """
        if key == kb.KeyCode.from_char('w'):
            self.keyPressed = 'w'
        elif key == kb.KeyCode.from_char('q'):
            self.keyPressed = 'q'
        elif key == kb.KeyCode.from_char('s'):
            self.keyPressed = 's'
        elif key == kb.KeyCode.from_char('e'):
            self.keyPressed = 'e'
        else:
            self.keyPressed = ''
    
    def releaseKey(self, key):
        """
        Callback for kb.Listener: It is called when a key is released and a Twist message with all the parameters
        in 0 is published.
        """
        self.keyPressed = ''

    def saveInformation(self):
        """
        Save in a .txt file the sequence of keys that are pressed.
        """
        while not rospy.is_shutdown():
            twistMessage = Twist()
            if self.keyPressed == 'w':
                twistMessage.linear.x = 2.0
            elif self.keyPressed == 'q':
                twistMessage.angular.z = 10.0
            elif self.keyPressed == 's':
                twistMessage.linear.x = -2.0
            elif self.keyPressed == 'e':
                twistMessage.angular.z = -10.0
            self.cmdPublisher.publish(twistMessage)
            self.writeFile(self.PATH_TOURS, ','+self.keyPressed)
            self.RATE.sleep()
        

    def writeFile(self, path, msg):
        """
        Open a file, write the message that is received by parameter and close the file.
        """
        file = open(path, 'a')
        file.write(msg)
        file.close()

if __name__ == '__main__':
    rospy.init_node('turtle_bot_teleop')
    punto3 = Punto3()
    with kb.Listener(punto3.pressKey, punto3.releaseKey) as escuchador:
        escuchador.join()
    rospy.spin()
