#!/usr/bin/env python
# -*- coding: utf-8 -*-

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

with open('look.pkl', 'rb') as archivo: 
    matriz= pickle.load(archivo,encoding='latin1')

tiempo=0
last_received_time = None
class Master():
    def __init__(self):
        self.IMU=list()
        
        self.x=0
        self.y=0

        self.pitch=0
        self.roll=0

        self.home=None

        self.pos=[0,0,0,0,0]
        self.inicio=0
        
        self.angulo=0

        self.present_xy=0

        self.mov=0
    #Callback para arduino serial
    def callback_IMU(self, data):
        self.pitch, self.roll =data.data[0], data.data[1]


    #Callback para tren superior
    def callback_vel(self,data):
        if self.mov==0:
            if self.y!=None:
                pid_goal = 0
                current_pos = ((1*self.y)*(1023/90)*0.5)
                d = data.dynamixel_state[4].present_velocity
                kp = 2
                kd = 0.1
                
                new_speed = kp*(pid_goal-current_pos) + kd*d
                if self.y<0:
                    new_speed=(abs(new_speed/2))+ 1024 
                else:
                    new_speed=(abs(new_speed/2))
                print("Y ", new_speed)
                
                goal_vel(4, int(new_speed))
                
                pid_goal = 0
                current_pos = ((1*self.x)*(1023/90)*0.5)
                d = data.dynamixel_state[3].present_velocity
                kp = 2
                kd = 0.1
                
                new_speed = kp*(pid_goal-current_pos) + kd*d
                if self.x>0:
                    new_speed=(abs(new_speed/2))+ 1024 
                else:
                    new_speed=(abs(new_speed/2))
                
                
                goal_vel(5,int(new_speed))
            
            else:
    
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
                goal_vel(5,0)
        else:

            pid_goal = 0
            current_pos = ((1*self.pitch)*(1023/90)*0.5)
            d = data.dynamixel_state[4].present_velocity
            kp = 2
            kd = 0.1
                
            new_speed = kp*(pid_goal-current_pos) + kd*d
            if self.pitch<0:
                    new_speed=(abs(new_speed/2))+ 1024 
            else:
                new_speed=(abs(new_speed/2))
                
                

                goal_vel(4, int(new_speed+200))
                goal_vel(5,0)
            

    #Callback tren inferior
            
    def callback_prueba(self,data):
        vel=[data.dynamixel_state[0].present_velocity,data.dynamixel_state[1].present_velocity,data.dynamixel_state[3].present_velocity]
        for i in vel:
            if i>0:
                self.mov=1
            else:
                self.mov=0
            
        global tiempo
        if self.home==None:
            print("xd")
        else:
            offset1= self.home[0]
            offset2= self.home[1]
            if time.time()-tiempo>5:
                print(time.time()-tiempo)
                if self.pitch<-30:
                    a=self.present_xy

                    if a[1]>0:
                        v_motores= matriz[1,a[1]-1]
                        self.present_xy=[1,a[1]-1]
                        goal_position(1, int(v_motores[2]+offset1),modo=1,vel=200)
                        goal_position(2, int(v_motores[3]+offset2),modo=1,vel=200)
                        goal_position(3,v_motores[4],modo=1,vel=200)
                        rospy.sleep(1)
                        tiempo=time.time()

                elif self.pitch>30:
                    a=self.present_xy
                    if a[1]<6:
                        v_motores= matriz[1,a[1]+1]
                        self.present_xy=[1,a[1]+1]
                        goal_position(1, int(v_motores[2]+offset1),modo=1,vel=200)
                        goal_position(2, int(v_motores[3]+offset2),modo=1,vel=200)
                        goal_position(3,v_motores[4],modo=1,vel=200)
                        rospy.sleep(1)
                        tiempo=time.time()
            print(self.present_xy)
            
            rospy.sleep(0.01)
    #Función calback lectura posicion inidical
    def callback_read(self,data):
        
        self.pos[0]=(data.dynamixel_state[0].present_position)
        self.pos[1]=(data.dynamixel_state[1].present_position)
        self.pos[2]=(data.dynamixel_state[3].present_position)
        # for i in range(3):
        #     self.pos[i]=(data.dynamixel_state[i].present_position)


    def homie(self,data):
        self.home=data.data
        print(self.home)
        print("offset is : ")
        print(self.home[0])
        print(self.home[1])
        
        goal_position(3,1020,modo=1,vel=200)
        self.present_xy=[0,7]

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

        rospy.Subscriber("homie_node", Float64MultiArray, self.homie)
            
        rospy.Subscriber("face_node", Float64MultiArray, self.callback_face)
        
        rospy.Subscriber("gyro", Float64MultiArray, self.callback_IMU,queue_size=1)
        
        rospy.Subscriber("/dynamixel_workbench/dynamixel_state", DynamixelStateList, self.callback_vel)
        
        #rospy.Subscriber("/dynamixel_workbench/dynamixel_state", DynamixelStateList, self.callback_prueba)
        
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

        
          