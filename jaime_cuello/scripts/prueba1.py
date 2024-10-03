from __future__ import print_function
import time
import sys
import rospy
from dynamixel_workbench_msgs.srv import *
from dynamixel_workbench_msgs.msg import *
from std_msgs.msg import Float64MultiArray
import numpy as np

def goal_position(ID , pos, modo,vel):
    
    if modo==0:
        #rospy.wait_for_service('/dynamixel_command')
        try:
            #Se manda el servicio para mover el motor
            dynamixel_command = rospy.ServiceProxy('/dynamixel_workbench/dynamixel_command' , DynamixelCommand)
            print ("ServiceProxy success ...")
                
            resp= dynamixel_command( '',ID,'Goal_Position',pos)
            return resp
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
    if modo==1:
        goal_vel(ID,vel)
        #rospy.wait_for_service('/dynamixel_command')
        try:
            #Se manda el servicio para mover el motor
            dynamixel_command = rospy.ServiceProxy('/dynamixel_workbench/dynamixel_command' , DynamixelCommand)
            print ("ServiceProxy success ...")
                
            resp= dynamixel_command( '',ID,'Goal_Position',pos)
            return resp
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

    if modo==2:
        if vel>=0:
            goal_vel(ID,vel)
            pos_p=[28000]
            #rospy.wait_for_service('/dynamixel_command')
        try:
            #Se manda el servicio para mover el motor
            dynamixel_command = rospy.ServiceProxy('/dynamixel_workbench/dynamixel_command' , DynamixelCommand)
            print ("ServiceProxy success ...")
                
            resp= dynamixel_command( '',ID,'Goal_Position',pos_p)
            return resp
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

        if vel<0:
            goal_vel(ID,vel)
            pos_n=[-28000]
        #rospy.wait_for_service('/dynamixel_command')
        try:
            #Se manda el servicio para mover el motor
            dynamixel_command = rospy.ServiceProxy('/dynamixel_workbench/dynamixel_command' , DynamixelCommand)
            print ("ServiceProxy success ...")
                
            resp= dynamixel_command( '',ID,'Goal_Position',pos_n)
            return resp
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)


def goal_vel(ID , vel):
    try:
        #Se manda el servicio para mover el motor
        dynamixel_command = rospy.ServiceProxy('/dynamixel_workbench/dynamixel_command' , DynamixelCommand)
        #print ("ServiceProxy success ...")
            
        resp= dynamixel_command( '',ID,'Moving_Speed',vel)
        return resp
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)
        

n=0
while n<5:
    y=int(input("vel= "))
    goal_vel(2,y)