#!/usr/bin/env python

import roslib
import rospy
import PyKDL
import math
import utils
import tf_conversions
import tf2_ros

from velma_common import*
from searcher.srv import *
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist,PoseStamped,Pose,TransformStamped
from gazebo_msgs.msg import ModelStates
from robot_moves import RobotMover
from camera_kinect_rgb import Kinect

from tf.transformations import quaternion_from_euler


class FinitStateMachine(object):

    def __init__(self,numberOfStates,velma):

        rospy.loginfo("Initiating Finite State Machine...")
        self.ifIinitialized = False
        self.actual_state = None
        self.numberOfStates= numberOfStates
        self.velma=velma
        self.last_state = None
        (self.initialPosx,self.initialPosy) = (None,None) 

    def __str__(self): 
        return 'Finite State has (%d) states' % self.numberOfStates

    def initiate(self):
        self.s0()
    def s0(self): 
        rospy.loginfo("In state 0 [INITIALIZE ROBOT]") 
        utils.diag(self.velma)
        self.initialAng = utils.getTorsoAngle(self.velma)
        print("INIT [TORSO ANGLE]: ",self.initialAng)
        (self.initialPosx,self.initialPosy)= utils.getInitialDistFromTable(self.velma,1)
        print("INIT [X]: ",self.initialPosx)
        print("INIT [Y]: ",self.initialPosy)
        self.ifInitialized = True
        rospy.sleep(3.0)
        if self.ifInitialized:
            self.s1()
            self.last_state = 0

    def s1(self):
        rospy.loginfo("In state 1 [GO TO TABLE 1]") 
        utils.safePos(velma,0) 
        goToTable1()
        self.last_state = 1
        self.s5()

    def s2(self):
        rospy.loginfo("In state 2 [GO TO TABLE 2]") 
        utils.safePos(velma,1) 
        goToTable2()
        self.last_state = 2
        self.s5()

    def s3(self):
        rospy.loginfo("In state 3 [GO TO CABINET]")  
        utils.safePos(velma,0)
        goToCabinet()
        self.last_state = 3
        self.s4()

    def s4(self):
        rospy.loginfo("In state 4 [OPEN CABINET]")  
        openCabinet()
        self.last_state = 4
        self.s5()

    def s5(self):
        rospy.loginfo("In state 5 [LOOK DOWN]") 
        lookAround()
        if self.last_state == 1:
            end = "end"
            #self.s2()
        elif self.last_state == 2:
            end = "end"
           # self.s3()
        else:
            rospy.loginfo("END OF ROUTE")
        self.last_state=5

def beerClient(mode):
    rospy.wait_for_service('beer_getter')
    try:
        beer_getter = rospy.ServiceProxy('beer_getter',Beer)
        resp = beer_getter(mode)
        return resp.result
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)   

def lookupClient():
    rospy.wait_for_service('look_around')
    try:
        look_around = rospy.ServiceProxy('look_around',Lookup)
        resp = look_around()
        return resp.result
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)   

def openingClient():
    rospy.wait_for_service('opening')
    try:
        opening = rospy.ServiceProxy('opening',Opening)
        resp = opening()
        return resp.result
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e) 


def putbackBeer():
    beerClient(1)

def goToCabinet():

    T_M_C = velma.getTf("map","cafe_fix")
    rm.driveSides(T_M_C.p[0])


def goToTable1():
    T_M_T1 = velma.getTf("map","table1_fix")
    #rm.rotateToDest(T_M_T1.p[0],T_M_T1.p[1])
    rm.driveFront(T_M_T1.p[0]-1.1)

def goToTable2():
    T_M_T2 = velma.getTf("map","table2_fix")
    T_M_B = velma.getTf("map","B")
    rospy.loginfo("Distance in x is %f"%math.fabs(T_M_T2.p[0]-T_M_B.p[0]))
    rm.driveFront(T_M_T2.p[0])
    rm.rotateToDest(T_M_T2.p[0],T_M_T2.p[1])
    print(utils.getTorsoAngle(velma))
    (initialPosx,initialPosy)= utils.getInitialDistFromTable(velma,2)
    print("INIT [X]: ",initialPosx)
    print("INIT [Y]: ",initialPosy)

def openCabinet():
    resp=openingClient()
    
def lookAround():
    utils.makeSafePos(velma,1) #pozycja rece w dol
    lookupClient()
    #kinect.showLastImg()

def usage():
    print("Inappropriate usage, should be : finite_state_machine 0..1")
  
if __name__ == "__main__":

    rospy.init_node('fsm_node',anonymous=True)
    velma=VelmaInterface()
    if len(sys.argv)==1:
        usage()
        sys.exit()
    if sys.argv[1]=='0':
        rm = RobotMover(velma)
        kinect = Kinect()
        fsm = FinitStateMachine(5,velma)
        print(fsm)
        fsm.initiate()
    if sys.argv[1]=='1':
        kinect = Kinect()
        kinect.showLastImg()
        print("second opt")
    if sys.argv[1]=='2':
        T_B_Be= velma.getTf("head_kinect_rgb_optical_frame","B")
        (roll,pitch,yaw)= T_B_Be.M.GetRPY()
        quaternion = quaternion_from_euler(roll, pitch, yaw)
        print(quaternion)

    rospy.spin()
