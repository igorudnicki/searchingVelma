#!/usr/bin/env python
#Project2 Igor Rudnicki, Marcel Kalinski
import roslib
import rospy
import PyKDL
import math

from velma_common import*
from rcprg_ros_utils import exitError

q_map_starting = {'torso_0_joint':0.0,'right_arm_0_joint':-0.4, 'right_arm_1_joint':-2.0,
	'right_arm_2_joint':1.25, 'right_arm_3_joint':1.8, 'right_arm_4_joint':0, 'right_arm_5_joint':-1.5,
	'right_arm_6_joint':0, 'left_arm_0_joint':0.4, 'left_arm_1_joint':2.0, 'left_arm_2_joint':-1.25,
	'left_arm_3_joint':-1.8, 'left_arm_4_joint':0, 'left_arm_5_joint':1.5, 'left_arm_6_joint':0} 

q_default = {'torso_0_joint':0, 'right_arm_0_joint':-0.3, 'right_arm_1_joint':-1.8,
    'right_arm_2_joint':1.25, 'right_arm_3_joint':0.85, 'right_arm_4_joint':0, 'right_arm_5_joint':-0.5,
    'right_arm_6_joint':0, 'left_arm_0_joint':0.3, 'left_arm_1_joint':1.8, 'left_arm_2_joint':-1.25,
    'left_arm_3_joint':-0.85, 'left_arm_4_joint':0, 'left_arm_5_joint':0.5, 'left_arm_6_joint':0 }

def moveLeftWrist(velma,x,y,z,roll,pitch,yaw):
	print("Moving left wrist...")
	T_B_Tld = PyKDL.Frame(PyKDL.Rotation.RPY( roll , pitch , yaw), PyKDL.Vector( x , y , z ))
	if not velma.moveCartImpLeft([T_B_Tld], [3.0], None, None, None, None, PyKDL.Wrench(PyKDL.Vector(5,5,5), PyKDL.Vector(5,5,5)), start_time=0.5):
		exitError(8)
	if velma.waitForEffectorLeft() != 0:
		exitError(9)
	rospy.sleep(0.5)

def moveRightWrist(velma,x,y,z,roll,pitch,yaw):
	print("Moving right wrist...")
	T_B_Trd = PyKDL.Frame(PyKDL.Rotation.RPY( roll , pitch , yaw), PyKDL.Vector( x , y , z ))
	if not velma.moveCartImpRight([T_B_Trd], [3.0], None, None, None, None, PyKDL.Wrench(PyKDL.Vector(5,5,5), PyKDL.Vector(5,5,5)), start_time=0.5):
		exitError(8)
	if velma.waitForEffectorRight() != 0:
		exitError(9)
	rospy.sleep(0.5)
def makeSafePos(velma,option=0):

	if option==0:
		q_map=q_map_starting
	else:
		q_map=q_default

	velma.moveJoint(q_map, time=5.0, start_time=0.2, stamp=None, position_tol=10.0/180.0 * math.pi)
	velma.waitForJoint()
	rospy.sleep(0.5)
	jstate= velma.getLastJointState()		#sprawdzenie czy jest poprawna pozycja poczatkowa
	if not isConfigurationClose(q_map,jstate[1],tolerance=0.4):
		print("Starting position should be",q_map)
		exitError(10)
	else:
		print("Starting position correct")
def modecart(velma):
	print("Switch to cart_imp mode")
	if not velma.moveCartImpRightCurrentPos(start_time=0.2):
		exitError(8)
	rospy.sleep(0.5)
	diag=velma.getCoreCsDiag()
	if not diag.inStateCartImp():
		print("Switch to cart_imp failed")
		exitError(3)

def safePos(velma,ifWrists):

	default_conf = (0.0,0.0)

	if isHeadConfigurationClose(velma.getHeadCurrentConfiguration(),default_conf,0.1):
		makeSafePos(velma,0)
	else:
		velma.moveHead(default_conf,3.0,start_time=0.5)
		if velma.waitForHead() !=0:
			exitError(8)
		rospy.sleep(0.5)
		if not isHeadConfigurationClose(velma.getHeadCurrentConfiguration(),default_conf,0.1):
			exitError(9)      
		makeSafePos(velma,0)

	if ifWrists == 1:
		modecart(velma)
		T_B_Wristr= velma.getTf("B","Tr")
		(rollWristr,pitchWristr,yawWristr)= T_B_Wristr.M.GetRPY()
		yawWristr += math.pi/6

		T_B_Wristl= velma.getTf("B","Tl")
		(rollWristl,pitchWristl,yawWristl)= T_B_Wristl.M.GetRPY()
		yawWristl -= math.pi/6

		moveLeftWrist(velma,T_B_Wristl.p[0],T_B_Wristl.p[1],T_B_Wristl.p[2],rollWristl,pitchWristl,yawWristl)
		moveRightWrist(velma,T_B_Wristr.p[0],T_B_Wristr.p[1],T_B_Wristr.p[2],rollWristr,pitchWristr,yawWristr)

def diag(velma):
	if not velma.waitForInit(timeout_s=10.0):
		print("Could not initialize VelmaInterface\n")
	else:
		print("Correctly initialized")
	print("Enabling motors...")
	if velma.enableMotors() != 0:
		exitError(2)

	rospy.sleep(0.5)
	diag = velma.getCoreCsDiag()
	if not diag.motorsReady():
		print("Motors must be homed and ready to use")
		exitError(1)
	rospy.sleep(0.5)

def checkIfCabinetOpened(velma):
	T_Ca_Rd = velma.getTf("cabinet_fix","right_door_fix")
	x= T_Ca_Rd.p[0]
	y= T_Ca_Rd.p[1]
	if math.sqrt(x**2+y**2)>0.25:
		return True
	else: 
		return False
def getHandGrabbingInitPos(velma):
	T_B_Tr = velma.getTf("B","Tr")
	(roll,pitch,yaw)= T_B_Tr.M.GetRPY()
	print(T_B_Tr)
	return T_B_Tr

def getInitialDistFromTable(velma,table):
	if table==1:
		T_B_T1 = velma.getTf("B","table1_fix")	
		return T_B_T1.p[0],T_B_T1.p[1]
	elif table==2:
		T_B_T2 = velma.getTf("B","table2_fix")	
		return T_B_T2.p[0],T_B_T2.p[1]
def getTorsoAngle(velma):
	jstate= velma.getLastJointState()
	ang = jstate[1]['torso_0_joint']
	return ang