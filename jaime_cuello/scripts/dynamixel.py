#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function
import time
import sys
import rospy
from dynamixel_workbench_msgs.srv import *
from dynamixel_workbench_msgs.msg import *
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Joy

pos=[]

last_received_time = None
class Master():
    def __init__(self):
        self.IMU=list()
        
        self.x=0
        self.y=0

        self.home=[0,0,0,0,0]

        self.pos=[0,0,0,0,0]
        self.inicio=0
        

    #Callback para arduino serial
    def callback_IMU(self, data):
        self.x, self.y =data.data[1], data.data[0]


    #Callback para tren superior
    def callback_vel(self,data):

        pid_goal = 0
        current_pos = ((1*self.x)*(1023/90)*0.5)
        d = data.dynamixel_state[1].present_velocity
        kp = 3
        kd = 0.1
        
        new_speed = kp*(pid_goal-current_pos) + kd*d
        if self.x>0:
               new_speed=(abs(new_speed/2))+ 1024 
        else:
              new_speed=(abs(new_speed/2))
        
        

        goal_vel(4, new_speed)


    def callback_multivuelta(self,data):
        global pos

        offset1= self.home[0]
        offset2= self.home[1]

        
    
        primer_joint=data.axes[1]
        segundo_joint=data.axes[4]
        
        
        vel1=int(segundo_joint*200)
        vel2=int(primer_joint*200)
        print(vel1,vel2)
    
        if offset1-self.pos[0]>=0:
            if vel1>0:
                vel1=0

        #if vel1<0:
            #goal_position(1,25000,modo=1,vel=-vel1)
        #else:
            #goal_position(1,-25000,modo=1,vel=vel1)
    #Función calback lectura posicion 
    def callback_read(self,data):
        
        self.pos[0]=(data.dynamixel_state[0].present_position)
        self.pos[1]=(data.dynamixel_state[1].present_position)
        self.pos[2]=(data.dynamixel_state[4].present_position)
        # for i in range(3):
        #     self.pos[i]=(data.dynamixel_state[i].present_position)


    def homie(self,data):
        self.home=data.data
        print(self.home)
        print("offset is : ")
        print(9000-self.home[0])
        print(-1000-self.home[1])
        goal_position(3,1000,modo=0,vel=200)
        goal_vel(3,200)

    def Main(self):
        
        #Se inicia nodo 
        rospy.init_node('Dynamixel', anonymous=True)
        

        rospy.Subscriber("homie_node", Float64MultiArray, self.homie)
        
        

        #rospy.Subscriber("gyro", Float64MultiArray, self.callback_IMU)
        
        rospy.Subscriber("/dynamixel_workbench/dynamixel_state", DynamixelStateList, self.callback_read,queue)
        
        rospy.Subscriber("joy", Joy, self.callback_multivuelta)
        
        

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



#####################################################################################






############################################################################################





if __name__ == "__main__":
    # Inicializa la clase IMU
    imu = Master()

    # Llama a la función listener para iniciar la suscripción al tópico "gyroscope"
    imu.Main()

        
          