#!/usr/bin/env python3


from __future__ import print_function
import time
import sys
import rospy
from dynamixel_workbench_msgs.srv import *
from dynamixel_workbench_msgs.msg import *
from std_msgs.msg import Float64MultiArray
import numpy as np
import pandas as pd
import pickle
from sensor_msgs.msg import Joy

tiempo=0
last_received_time = None
class Master():
    def __init__(self):
        self.IMU=list()
        
        self.x=0
        self.y=0

        self.home=None

        self.pos=[0,0,0,0,0]
        self.inicio=0
        
        self.joy=[0,0]

        self.angulo=0

        self.cambio=0
    #Callback para arduino serial
    def callback_IMU(self, data):

        self.pitch, self.roll =data.data[0], data.data[1]


    #Callback para tren superior
    def callback_vel(self,data):
        pid_goal = 0
        current_pos = ((1*self.pitch)*(1023/90)*0.5)
        d = data.dynamixel_state[4].present_velocity
        kp = 4
        kd = 0.2            
        new_speed = kp*(pid_goal-current_pos) + kd*d
        if self.pitch<0:
            new_speed=(abs(new_speed/2))+ 1024 
        else:
            new_speed=(abs(new_speed/2))
                
                

        goal_vel(4, int(new_speed))
        



    def callback_multivuelta(self,data):
        self.pos[0]=data.dynamixel_state[0].present_position
        self.pos[1]=data.dynamixel_state[1].present_position
        
        for i in range(2):
            if self.pos[i]>32768:
                self.pos[i]=self.pos[i]-65536

        print(self.pos)

        j=0
        if (self.joy[0]>0) and (self.pos[0]>=self.home[0]):
            j=1
            print(j)
        h=0
        if (self.joy[1]>0) and (self.pos[1]>=self.home[1]):
            h=1
            print(h)

        if self.cambio==1 or (j==1 or h==1):

            if self.home!=None:
                for i in range(2):
                    velocidad=int(np.abs(200*self.joy[i]))
                    
                    if (velocidad !=0) :
                        if (self.joy[i]>0) :
                            if self.pos[i]>=self.home[i]:
                                goal_position(i+1,self.pos[i],modo=1,vel=1)
                                print("xd")

                            else:
                                goal_position(i+1,28000,modo=1,vel=velocidad)
                                

                        else:

                            goal_position(i+1,-28000,modo=1,vel=velocidad)
                            

                    else:
                        goal_position(i+1,self.pos[i],modo=1,vel=1)
  
            self.cambio=0
        
        
        
    def callback_joy(self,data):
        self.joy=[data.axes[0],data.axes[1]]
        self.cambio=1
        print(self.cambio)
        
  

    def callback_read(self,data):
        self.pos[0]=data.dynamixel_state[0].present_position
        self.pos[1]=data.dynamixel_state[1].present_position
        
        for i in range(2):
            if self.pos[i]>32768:
                self.pos[i]=self.pos[i]-65536
        
    def homie(self,data):
        self.home=data.data
        print(self.home)
        print("offset is : ")
        print(self.home[0])
        print(self.home[1])
        goal_position(3,800,modo=1,vel=300)
        
        

    def Main(self):

        #Se inicia nodo 
        rospy.init_node('Dynamixel', anonymous=True)

        rospy.Subscriber("homie_node", Float64MultiArray, self.homie)
        
        rospy.Subscriber("gyro", Float64MultiArray, self.callback_IMU)
        
        rospy.Subscriber("/dynamixel_workbench/dynamixel_state", DynamixelStateList, self.callback_vel)
        
        rospy.Subscriber("joy", Joy, self.callback_joy,queue_size=1)

        #rospy.Subscriber("/dynamixel_workbench/dynamixel_state", DynamixelStateList, self.callback_read,queue_size=1)
        rospy.Subscriber("/dynamixel_workbench/dynamixel_state", DynamixelStateList, self.callback_multivuelta,queue_size=1)
        
        rospy.spin() 
        



###################################################################3

def goal_position(ID , pos, modo,vel):
    
    if modo==0:
        #rospy.wait_for_service('/dynamixel_command')
        try:
            #Se manda el servicio para mover el motor
            dynamixel_command = rospy.ServiceProxy('/dynamixel_workbench/dynamixel_command' , DynamixelCommand)
            #print ("ServiceProxy success ...")
                
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
            #print ("ServiceProxy success ...")
                
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

#####################################################################################






############################################################################################





if __name__ == "__main__":
    while (time.time()-tiempo)<3:
        pass

    # Inicializa la clase IMU
    imu = Master()

    # Llama a la función listener para iniciar la suscripción al tópico "gyroscope"
    imu.Main()

        
          