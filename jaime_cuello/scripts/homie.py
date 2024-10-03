#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import Float64MultiArray
from dynamixel_workbench_msgs.srv import *
from dynamixel_workbench_msgs.msg import *
import time 

tiempo=time.time()
home=None
pos=[0,0,0,0,0]
#Función calback lectura posicion inidical
def callback_read(data):
    # rospy.Subscriber("/dynamixel_workbench/dynamixel_state", DynamixelStateList, callback_read)
    global home

    if home==None : 
     pos[0]=(data.dynamixel_state[0].present_position)
     pos[1]=(data.dynamixel_state[1].present_position)
      
     
     home=pos
            
     print(home)
     print("offset is : ")
     print(9000-home[0])
     print(-1000-home[1])
    
# Inicializa el nodo ROS
while (time.time()-tiempo)<5:
  pass

rospy.init_node('homie_publisher', anonymous=True)

# Crea un objeto para publicar los datos
pub = rospy.Publisher('homie_node', Float64MultiArray, queue_size=10)
if home==None:

  rospy.Subscriber("/dynamixel_workbench/dynamixel_state", DynamixelStateList, callback_read)
  time.sleep(0.1)
  data = Float64MultiArray()
  data.data = home
  pub.publish(data)
  
