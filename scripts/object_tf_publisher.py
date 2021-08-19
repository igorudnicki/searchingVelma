#!/usr/bin/env python

import roslib
import rospy
import PyKDL
import math
import tf_conversions
import tf2_ros

from velma_common import*
from geometry_msgs.msg import Twist,PoseStamped,Pose,TransformStamped
from gazebo_msgs.msg import ModelStates


class Position(object):

    def __init__(self, x, y,z,theta):
        self.x = x
        self.y = y
        self.z = z
        self.theta = theta 

class TransformPublisher(object):

    def __init__(self):
        self.gazeboPos = Position(0,0,0,0)
        self.broadcaster = tf2_ros.TransformBroadcaster()
        self.tt = geometry_msgs.msg.TransformStamped()
        self.index = 0
        self.runOnce = True
        self.listener()

    def getGazeboNav(self,data):

        if (self.runOnce == True):
            self.index = self.getVelmaIndex(data)
            self.runOnce = False

        self.gazeboPos.x = data.pose[self.index].position.x
        self.gazeboPos.y = data.pose[self.index].position.y
        self.gazeboPos.z = data.pose[self.index].position.z
                
        rotation = PyKDL.Rotation.Quaternion(data.pose[self.index].orientation.x, data.pose[self.index].orientation.y, data.pose[self.index].orientation.z, data.pose[self.index].orientation.w)
        [roll,pitch,yaw] = rotation.GetRot()
        self.gazeboPos.theta= yaw;

    def getVelmaIndex(self,data):

        for model in data.name:
            if model == "velma":
                return self.index
            self.index = self.index+1

    def getTfNew(self,new_name,transform,diff):
        self.tt.header.stamp = rospy.Time.now()
        self.tt.header.frame_id = "map"
        self.tt.child_frame_id = new_name
        self.tt.transform.translation.x = transform.p[0]+diff[0]
        self.tt.transform.translation.y = transform.p[1]+diff[1]
        self.tt.transform.translation.z = transform.p[2]+diff[2]
        if new_name=="cabinet_fix" or new_name=="right_door_fix" or new_name=="right_handle_fix":
            q = tf_conversions.transformations.quaternion_from_euler(0, 0, -1.5708)
        else:
            q = tf_conversions.transformations.quaternion_from_euler(0, 0, 0)
        self.tt.transform.rotation.x = q[0]
        self.tt.transform.rotation.y = q[1]
        self.tt.transform.rotation.z = q[2]
        self.tt.transform.rotation.w = q[3]
        return self.tt
    def listener(self):
        self.model_sub = rospy.Subscriber('/gazebo/model_states',ModelStates, self.getGazeboNav) 

if __name__ == "__main__":

    rospy.init_node('object_tf_gazebo',anonymous=False)   #stworzenie node'a i inicjacja velmy
    velma=VelmaInterface()
    tp = TransformPublisher()
    while not rospy.is_shutdown():

        T_B_Be= velma.getTf("map","torso_base")
        T_M_T1= velma.getTf("map","table")
        T_M_T2= velma.getTf("map","table2")
        T_M_Be= velma.getTf("map","beer")

        T_M_Ct= velma.getTf("map","cafe_table")
        T_M_Ca= velma.getTf("map","cabinet")
        T_M_Rd= velma.getTf("map","right_door")
        T_M_Rh= velma.getTf("map","right_handle")

        diff = (T_B_Be.p[0] - tp.gazeboPos.x,T_B_Be.p[1] - tp.gazeboPos.y,T_B_Be.p[2] - tp.gazeboPos.z)

        tp.broadcaster.sendTransform(tp.getTfNew("table1_fix",T_M_T1,diff))
        tp.broadcaster.sendTransform(tp.getTfNew("table2_fix",T_M_T2,diff))
        tp.broadcaster.sendTransform(tp.getTfNew("beer_fix",T_M_Be,diff))

        tp.broadcaster.sendTransform(tp.getTfNew("cafe_fix",T_M_Ct,diff))
        tp.broadcaster.sendTransform(tp.getTfNew("cabinet_fix",T_M_Ca,diff))
        tp.broadcaster.sendTransform(tp.getTfNew("right_handle_fix",T_M_Rd,diff))
        tp.broadcaster.sendTransform(tp.getTfNew("right_door_fix",T_M_Rh,diff))
        
    rospy.spin()