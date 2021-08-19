#!/usr/bin/env python
import roslib
import rospy

from searcher.srv import *
from velma_common import*
from rcprg_ros_utils import exitError

class HeadMover(object):

    def __init__(self,velma):

        rospy.loginfo("Initiating Head Mover...")
        self.velma=velma
        self.q_down = (0.0,0.6)
        self.q_left = (1.0,0.7)
        self.q_right = (-1.0,0.7)

    def startMoving(self):
        rospy.loginfo("Moving head around...")
        self.moveToGivenConf(self.q_down,3.0)
        #self.moveToGivenConf(self.q_left,2.0)
        #self.moveToGivenConf(self.q_right,4.5)
        #self.moveToGivenConf(self.q_down,2.5)
        
    def moveToGivenConf(self,config,time):
        rospy.loginfo("Moving to "+str(config))
        self.velma.moveHead(config,time,start_time=0.5)
        if self.velma.waitForHead() !=0:
            exitError(8)
        rospy.sleep(0.5)
        if not isHeadConfigurationClose(self.velma.getHeadCurrentConfiguration(),config,0.1):
            exitError(9)        
        rospy.sleep(0.1)


def lookaround_callback(req):
    head_mover = HeadMover(velma)
    head_mover.startMoving()
    return "Head movement completed"
	
if __name__ == "__main__":

    rospy.init_node('lookaround_node',anonymous=True)
    velma=VelmaInterface()
    s = rospy.Service('look_around', Lookup, lookaround_callback)
    rospy.spin()    