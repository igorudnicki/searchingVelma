#!/usr/bin/env python
import roslib
import rospy
import PyKDL
import math
import utils

from searcher.srv import *
from velma_common import*
from rcprg_ros_utils import exitError
from geometry_msgs.msg import Twist,PoseStamped,Pose,Point,Quaternion

class BeerManipulator(object):

	def __init__(self,velma):

		rospy.loginfo("Initiating Beer Manipulator...")
		self.velma=velma
		self.beer_height = 0.15
		self.away_from_beer = 0.35
		self.back_with_beer = 0.15

	def isGrabbed(self):
		r_hand= self.velma.getHandRightCurrentConfiguration()
		if r_hand[1]>1.55 and r_hand[4]>1.55 and r_hand[6]>1.55:
			rospy.loginfo("Grabbing unsuccessful")
			return 0
		else:
			rospy.loginfo("Grabbing successful")
			return 1

	def moveRightWrist(self,x,y,z,roll,pitch,yaw):
		rospy.loginfo("Moving right wrist...")
		T_B_Trd = PyKDL.Frame(PyKDL.Rotation.RPY( roll , pitch , yaw), PyKDL.Vector( x , y , z ))
		if not self.velma.moveCartImpRight([T_B_Trd], [2.5], None, None, None, None, PyKDL.Wrench(PyKDL.Vector(5,5,5), PyKDL.Vector(5,5,5)), start_time=0.5):
			exitError(8)
		if self.velma.waitForEffectorRight() != 0:
			exitError(9)

	def changeTool(self):
		rospy.loginfo("Moving tool and equilibrium pose from 'wrist' to 'grip' frame...")
		T_B_Wr = self.velma.getTf("B", "Wr")
		T_Wr_Gr = self.velma.getTf("Wr", "Gr")
		if not self.velma.moveCartImpRight([T_B_Wr*T_Wr_Gr], [0.1], [T_Wr_Gr], [0.1], None, None, PyKDL.Wrench(PyKDL.Vector(5,5,5), PyKDL.Vector(5,5,5)), start_time=0.5):
			exitError(18)
		if self.velma.waitForEffectorRight() != 0:
			exitError(19)

	def changeToolBack(self):
		rospy.loginfo("Moving tool and equilibrium pose from 'grip' to 'wrist' frame...")
		T_B_Gr = self.velma.getTf("B", "Gr")
		T_Gr_Wr = self.velma.getTf("Gr", "Wr")
		if not self.velma.moveCartImpRight([T_B_Gr*T_Gr_Wr], [0.1], [T_Gr_Wr], [0.1], None, None, PyKDL.Wrench(PyKDL.Vector(5,5,5), PyKDL.Vector(5,5,5)), start_time=0.5):
			exitError(18)
		if self.velma.waitForEffectorRight() != 0:
			exitError(19)

	def handTowardsBeer(self):
		rospy.loginfo("Rotating towards beer...")
		utils.modecart(self.velma)
		T_B_Wristr= self.velma.getTf("B","Tr")
		(rollWristr,pitchWristr,yawWristr)= T_B_Wristr.M.GetRPY()
		yawWristr= yawWristr - math.pi/1.5
		self.moveRightWrist(T_B_Wristr.p[0],T_B_Wristr.p[1],T_B_Wristr.p[2],rollWristr,pitchWristr,yawWristr)

	def handDirection(self):
		T_Wr_B= self.velma.getTf("B","Tr")
		T_B_Be= self.velma.getTf("B","beer_fix")
		(roll,pitch,yaw)= T_B_Be.M.GetRPY()
		T_B_B = self.velma.getTf("B","beer_fix")
		self.moveRightWrist(T_B_B.p[0]-self.away_from_beer,T_B_B.p[1],T_B_B.p[2]+self.beer_height/2,0,0,yaw)

	def changedWristMove(self):
		self.changeTool()
		T_Wr_B= self.velma.getTf("B","Tr")
		(roll,pitch,yaw)= T_Wr_B.M.GetRPY()
		T_B_Be = self.velma.getTf("B","beer_fix")
		self.moveRightWrist(0.97*T_B_Be.p[0],T_B_Be.p[1],T_Wr_B.p[2],roll,pitch,yaw)	

	def handBack(self):
		T_Wr_B= self.velma.getTf("B","Tr")
		(roll,pitch,yaw)= T_Wr_B.M.GetRPY()
		self.moveRightWrist(T_Wr_B.p[0]-self.back_with_beer,T_Wr_B.p[1]-self.back_with_beer,T_Wr_B.p[2]+self.back_with_beer,roll,pitch,yaw)		

	def handPutback(self):
		T_Wr_B= self.velma.getTf("B","Tr")
		T_B_T2= self.velma.getTf("B","table2_fix")
		(roll,pitch,yaw)= T_B_T2.M.GetRPY()
		pitch = pitch+math.pi/2
		roll = roll +math.pi/2
		self.moveRightWrist(T_Wr_B.p[0]+self.back_with_beer,T_Wr_B.p[1]+self.back_with_beer,T_Wr_B.p[2]-self.back_with_beer,roll,pitch,yaw)

	def closeRightHand(self,closing_factor):
		rospy.loginfo("Closing right hand fingers..")
		angle=closing_factor*math.pi
		conf= [angle,angle,angle,0]
		self.velma.moveHandRight(conf,[1,1,1,1],[2000,2000,2000,2000],1000,hold=True)
		if self.velma.waitForHandRight()!=0:
			exitError(6)

	def releaseRightHand(self):
		rospy.loginfo("Opening right hand fingers..")
		conf= [0,0,0,0]
		self.velma.moveHandRight(conf,[1,1,1,1],[2000,2000,2000,2000],1000,hold=True)
		if self.velma.waitForHandRight()!=0:
			exitError(6)	


def beer_callback(req):

	if req.mode==0: 
		bm.handTowardsBeer() 
		bm.handDirection()
		bm.changedWristMove()
		bm.closeRightHand(0.6)
		if not bm.isGrabbed():
			return "Failure grabbing"
		bm.handBack()
		return "Mode 0 completed"
	elif req.mode==1:                #odlozenie sloika i cofniecie reki
		bm.handPutback()
		bm.releaseRightHand()
		bm.handBack()
		return "Mode 1 completed"
	elif req.mode==2:
		bm.handTowardsBeer()
		return "Mode 2 completed"


if __name__ == "__main__":

	rospy.init_node('beer_node',anonymous=True)
	velma=VelmaInterface()
	bm = BeerManipulator(velma)
	s = rospy.Service('beer_getter', Beer, beer_callback)
	rospy.spin()
