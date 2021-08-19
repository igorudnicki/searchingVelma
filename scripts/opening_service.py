#!/usr/bin/env python
import roslib
import rospy
import PyKDL
import math
import utils

from searcher.srv import *
from velma_common import*
from rcprg_ros_utils import exitError
from geometry_msgs.msg import Twist,PoseStamped,Pose,Quaternion,Point
 
cabinet_width = 0.6
cabinet_height= 0.7


def deg2rad(deg):
    return float(deg)/180.0*math.pi

def hookGrip(angle=100):
    print("Configuring grip...")
    qdest= [deg2rad(angle), deg2rad(angle), deg2rad(angle), deg2rad(180)]
    velma.moveHandRight(qdest, [1, 1, 1, 1], [4000,4000,4000,4000], 1000, hold=True)
    if velma.waitForHandRight() != 0:
        exitError(6)

def getObjectLoc(object_name):
    T_B_O = velma.getTf("B",object_name)
    T_B_O_angle = math.atan2(T_B_O.p[1],T_B_O.p[0])
    return T_B_O,T_B_O_angle 

def setImpRight(x,y,z,xT,yT,zT):
    print "Setting impendance"
    if not velma.moveCartImpRight(None, None, None, None, [PyKDL.Wrench(PyKDL.Vector(x,y,z), PyKDL.Vector(xT,yT,zT))], [2], PyKDL.Wrench(PyKDL.Vector(5,5,5), PyKDL.Vector(5,5,5))):
        exitError(16)
    if velma.waitForEffectorRight() != 0:
        exitError(17)

def changeTool():

    print "Moving tool and equilibrium pose from 'wrist' to 'grip' frame..."
    T_B_Wr = velma.getTf("B", "Wr")
    T_Wr_Gr = velma.getTf("Wr", "Gr")
    if not velma.moveCartImpRight([T_B_Wr*T_Wr_Gr], [0.1], [T_Wr_Gr], [0.1], None, None, PyKDL.Wrench(PyKDL.Vector(5,5,5), PyKDL.Vector(5,5,5)), start_time=0.5):
        exitError(18)
    if velma.waitForEffectorRight() != 0:
        exitError(19)

def moveRightWrist(x,y,z,roll,pitch,yaw):
    print("Moving right wrist...")
    T_B_Trd = PyKDL.Frame(PyKDL.Rotation.RPY( roll , pitch , yaw), PyKDL.Vector( x , y , z ))
    if not velma.moveCartImpRight([T_B_Trd], [3.5], None, None, None, None, PyKDL.Wrench(PyKDL.Vector(5,5,5), PyKDL.Vector(5,5,5)), start_time=0.5):
        exitError(8)
    if velma.waitForEffectorRight() != 0:
        exitError(9)
    rospy.sleep(0.5)

def moveRightWristSlowTol(x,y,z,roll,pitch,yaw, tol):
    print("Moving right wrist...")
    T_B_Trd = PyKDL.Frame(PyKDL.Rotation.RPY( roll , pitch , yaw), PyKDL.Vector( x , y , z ))
    if not velma.moveCartImpRight([T_B_Trd], [3.0], None, None, None, None, PyKDL.Wrench(PyKDL.Vector(5,5,5), PyKDL.Vector(5,5,5)), start_time=0.5,path_tol=PyKDL.Twist(PyKDL.Vector(tol,tol,tol), PyKDL.Vector(tol,tol,tol))):
        exitError(8)
    if velma.waitForEffectorRight() != 0:
        exitError(9)
    rospy.sleep(0.5)
def handTowardsCabinet(velma):

    print("moving towards cabinet")
    utils.modecart(velma)
    T_B_Wristr= velma.getTf("B","Tr")
    (rollWristr,pitchWristr,yawWristr)= T_B_Wristr.M.GetRPY()
    yawWristr= yawWristr - math.pi/2

    moveRightWrist(T_B_Wristr.p[0],T_B_Wristr.p[1],T_B_Wristr.p[2],rollWristr,pitchWristr,yawWristr)

def movementToHandle(velma,T_B_C):

    T_B_Rh,T_B_Rh_angle = getObjectLoc("right_handle_fix")
    T_B_Rd,T_B_Rd_angle = getObjectLoc("right_door_fix")


    (rollCab,pitchCab,yawCab)= T_B_C.M.GetRPY()

    pointrx= (T_B_Rd.p[0]+T_B_Rh.p[0])*0.5
    pointry= (T_B_Rd.p[1]+T_B_Rh.p[1])*0.5
    pointr_ang = math.atan2(pointry,pointrx)
    T_C_B = velma.getTf("cabinet_fix","B")

    x=0.5
    y=0
    z=0
    pos = PyKDL.Vector(x,y,z)
    point = T_C_B.Inverse() * pos
    point[2] = T_B_Rh.p[2]+0.1
    p = (point[0],point[1],point[2])       #ruch przed klamke


    yawCab= yawCab+math.pi
    utils.modecart(velma)
    hookGrip()


    moveRightWristSlowTol(p[0],p[1],p[2],0,0,yawCab,0.15)
    changeTool()

    setImpRight(1000,300,300,1000,1000,1000)
    T_B_Wrist = velma.getTf("B","Tr")
    (rollWrist,pitchWrist,yawWrist)= T_B_Wrist.M.GetRPY() #DO DOPRACOWANIA
    moveRightWristSlowTol(0.97*pointrx,0.97*pointry,T_B_Rh.p[2],rollWrist,pitchWrist,yawWrist, 0.15)
    #dotknal szafki

    setImpRight(1000,1000,1000,1000,1000,1000)
    T_C_Rh = velma.getTf("cabinet_fix","right_handle_fix")
    x= T_C_Rh.p[0] + 0.06
    y= 0.01
    z= T_C_Rh.p[2]
    pos = PyKDL.Vector(x,y,z)
    point = T_C_B.Inverse() * pos
    point[2]=T_B_Rh.p[2]
    moveRightWrist(point[0],point[1],point[2],rollWrist,pitchWrist,yawWrist)


def getGlobalCirclePoint():

    T_B_Rh,T_B_Rh_angle = getObjectLoc("right_handle_fix")
    z = T_B_Rh.p[2]
    #rownanie okregu dla szafki po prawej stronie
    radius = 0.26
    x0 = 0.178
    y0 = 0.283
    rad = -math.pi/2
    plist=[]
    while rad <= math.pi/8:
        x= x0 + radius*math.cos(rad)
        y= y0 + radius*math.sin(rad) 
        T_C_B = velma.getTf("cabinet_fix","B")
        pos = PyKDL.Vector(x,y,z)
        point = T_C_B.Inverse() * pos
        point[2] = T_B_Rh.p[2]
        p = (point[0],point[1],point[2])
        plist.append(p)
        if rad == 0:
            rad += math.pi/16
        else:
            rad += math.pi/8
    return plist

def movementToOpen(point_list):

    setImpRight(1000,100,100,1000,1000,1000)

    T_B_Wrist = velma.getTf("B","Tr")
    (rollWrist,pitchWrist,yawWrist)= T_B_Wrist.M.GetRPY()
    finger_angle = 100
    i =0

    for point in point_list[1:]:

        if i==0:
            yawWrist += math.pi/14
            finger_angle += 12
            hookGrip(finger_angle)
            moveRightWristSlowTol(point[0], point[1], point[2], rollWrist, pitchWrist, yawWrist, 0.1)
            yawWrist += math.pi/11
        elif i>=1 and i<=4:
            moveRightWristSlowTol(point[0], point[1], point[2], rollWrist, pitchWrist, yawWrist, 0.1)
            yawWrist += math.pi/11
        else:
            moveRightWristSlowTol(point[0], point[1], point[2], rollWrist, pitchWrist, yawWrist, 0.1)               
        i=i+1

    T_Rd_Rh = velma.getTf("right_door_fix","right_handle_fix")
    x=T_Rd_Rh.p[0] + 0.05
    y=T_Rd_Rh.p[1] + 0.12
    z=T_Rd_Rh.p[2]
    T_B_Rh,T_B_Rh_angle = getObjectLoc("right_handle_fix")
    T_C_B = velma.getTf("right_door_fix","B")
    pos = PyKDL.Vector(x,y,z)
    point = T_C_B.Inverse() * pos
    point[2]=T_B_Rh.p[2]
    p = (point[0],point[1],point[2])
    moveRightWrist(p[0], p[1], p[2], rollWrist, pitchWrist, yawWrist)
    
def releaseRightHand():
    print("Opening right hand fingers..")
    conf= [0,0,0,0]
    velma.moveHandRight(conf,[1,1,1,1],[2000,2000,2000,2000],1000,hold=True)
    if velma.waitForHandRight()!=0:
        exitError(6)

def opening_callback(req):

    T_B_C,T_B_C_angle = getObjectLoc("cabinet_fix")
    #handTowardsCabinet(velma)
    movementToHandle(velma,T_B_C)
    point_list = getGlobalCirclePoint()
    movementToOpen(point_list)
    #releaseRightHand()
    utils.safePos(velma,1) 
    return "Opening done"

if __name__ == "__main__":

    rospy.init_node('opening_node')
    velma=VelmaInterface()
    s = rospy.Service('opening', Opening, opening_callback)
    rospy.spin()    
