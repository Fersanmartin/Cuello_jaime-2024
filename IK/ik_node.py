import numpy as np
import time
import rospy
from std_msgs.msg import Float64MultiArray, MultiArrayDimension, Int32
from dynamixel_workbench_msgs.msg import *
import NR_cuello as nr
from sensor_msgs.msg import JointState
from std_msgs.msg import Header


goal_pos = None
conf = 0
dynamixel_position = None
joints = None
q0 = None

##Callback recibe goal_pos data
def callback(data):
    global goal_pos
    goal_pos = data.data

#Callback recibe present position dynamixel joint 1 and 2
def dynamixel_read(data):
    global dynamixel_position
    dynamixel_position[0]=data.dynamixel_state[0].present_position
    dynamixel_position[1]=data.dynamixel_state[1].present_position

#Callback recibe de present position joints in simulation
def joint_read(data):
    global joints
    joints = [data.position[0],data.position[2]]
    

def conf_read(data):
    global conf
    conf= data.data

#offsets of dynamixels in angle=0º
offset = [195,90.5]

#Init de ros node of script
rospy.init_node('IK', anonymous=True)

#assign None value to before_goal_pos
before_goal_pos=None

#Current waypoint
n=0

#Recibe the goal position 
rospy.Subscriber('Goal_pos', Float64MultiArray, callback)
#Recibe the present joint states of simulation
rospy.Subscriber('joint_states', JointState, joint_read,queue_size=1)
#Recibe the present position of dynamixel angles
rospy.Subscriber("/dynamixel_workbench/dynamixel_state", DynamixelStateList, dynamixel_read,queue_size=1)
#Recibe de confirmation data
rospy.Subscriber('/confirmation', Int32, conf_read)

while not rospy.is_shutdown():
    #define q0
    #Real Case
    if dynamixel_position != None:
        q0 = [(dynamixel_position[0]*300/1024) - offset[0] ,(dynamixel_position[1]*300/1024) - offset[1]]
    #Simulation Case
    if joints != None:
        q0=joints
    #If recibe a goal_pos
    if goal_pos != None and q0 != None:
        #if goal_pos is diferent of the before
        if goal_pos != before_goal_pos:
            #calculate the trayectori
            points=nr.puntos_array(q0,goal_pos[0],goal_pos[1])
            print(points)
            way_points=[]
            q=q0
            #calculate the waypoint
            for i in points:
                q=nr.IK_NR(i,q)
                way_points.append(q)
            # Reset current waypoint
            n=0

            print(way_points)
            #Assign the current goal_pos to before_goal_pos
            before_goal_pos=goal_pos

        #update the current way_point
        if (conf==1) and (n!=len(way_points)-1):
            n +=1
            #return conf to 0 
            conf=0

        

        #current goal angles
        goal_angles=way_points[n]

        #pub the goal_angles
        pub = rospy.Publisher('/joint_node', Float64MultiArray, queue_size=1)
        msg = Float64MultiArray()
        msg.data = [goal_angles[0],goal_angles[0],goal_angles[1],goal_angles[1],0,0]
        pub.publish(msg)