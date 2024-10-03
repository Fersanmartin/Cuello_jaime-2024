#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from __future__ import print_function
import rospy
from dynamixel_workbench_msgs.srv import *
from dynamixel_workbench_msgs.msg import *
from std_msgs.msg import Float64MultiArray
import numpy as np

tiempo=0
last_received_time = None
class Master():
    def __init__(self):

        self.home=None
        self.goal=None

    #callback posicion
    def callback_goal(self,data):
        offset=[375,281]
        goal=data.data
        self.goal=[int((goal[0]*(1024/300))+offset[0]), int((goal[1]*(1024/300))+offset[1])]
        print(self.goal)

    #Callback tren superior
    #Callback para tren superior
    def callback_vel(self,data):
        x=data.data[1]
        current_pos = ((1*x)*(1023/90)*0.5)
        #d = data.dynamixel_state[3].present_velocity
        kp = 2
        kd = 0.1
           
        new_speed = kp*(current_pos) + 50
        if x<0:
            new_speed=(abs(new_speed/2)) + 1024 
        else:
            new_speed=(abs(new_speed/2))
        print(new_speed)        
        goal_vel(4,int(new_speed))
            


    #Callback tren inferior
            
    def callback_prueba(self,data):
        conf=0
        print(data.dynamixel_state[4].present_position,data.dynamixel_state[5].present_position)
        if self.goal != None:
            for i in [1,2]:
                error = -1*(self.goal[i-1] - data.dynamixel_state[i+3].present_position)
                vel = data.dynamixel_state[i-1].present_velocity
                
                if vel>1024:
                    vel=-1*(vel-1024)

                p,d =[20,0]    

                PI =  np.clip( int(p*error + d*vel),-100,100)
                if abs(PI)<60:
                    PI=0


                if PI<0:
                    PI = -1*PI +1024
                
                goal_vel(i,PI)
                if abs(error)<5:
                    conf+=1
                else:
                    conf=0
            pub = rospy.Publisher('confirmacion', Float64MultiArray, queue_size=1)
            data = Float64MultiArray()
            if conf>=2:
                data.data = [1]
            else:
                data.data = [0]
            pub.publish(data)

    def homie(self,data):
        self.home=data.data
        
            
    def callback_face(self,data):
        if data.data[0]==-1:
            self.x=None
            self.y=None
        else:
            self.x=(data.data[0]-320)*0.2

            self.y=(data.data[1]-240)*0.2

    def Main(self):

        #Se inicia nodo 
        rospy.init_node('Dynamixel', anonymous=True)

        #rospy.Subscriber("face_node", Float64MultiArray, self.callback_face)

        rospy.Subscriber("gyro", Float64MultiArray, self.callback_vel,queue_size=1)

        rospy.Subscriber("Newton_R", Float64MultiArray, self.callback_goal, queue_size=1)

        rospy.Subscriber("/dynamixel_workbench/dynamixel_state", DynamixelStateList, self.callback_prueba,queue_size=1)
        
        rospy.spin() 
        



###################################################################3

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


def goal_vel(ID , vel):
    try:
        #Se manda el servicio para mover el motor
        dynamixel_command = rospy.ServiceProxy('/dynamixel_workbench/dynamixel_command' , DynamixelCommand)
        #print ("ServiceProxy success ...")
            
        resp= dynamixel_command( '',ID,'Moving_Speed',vel)
        return resp
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def Torque(ID , torque):
    try:
        #Se manda el servicio para mover el motor
        dynamixel_command = rospy.ServiceProxy('/dynamixel_workbench/dynamixel_command' , DynamixelCommand)
        #print ("ServiceProxy success ...")
            
        resp= dynamixel_command( '',ID,'Torque_Enable',torque)
        return resp
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

#####################################################################################






############################################################################################





if __name__ == "__main__":
    # Inicializa la clase IMU
    imu = Master()

    # Llama a la función listener para iniciar la suscripción al tópico "gyroscope"
    imu.Main()

        
          