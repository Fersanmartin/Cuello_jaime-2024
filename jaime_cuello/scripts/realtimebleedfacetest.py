#!/usr/bin/env python3

import cv2
import bleedfacedetector
import time

import rospy
from std_msgs.msg import Float64MultiArray
from dynamixel_workbench_msgs.srv import *
from dynamixel_workbench_msgs.msg import *

rospy.init_node('face_detecter', anonymous=True)

pub = rospy.Publisher('face_node', Float64MultiArray, queue_size=10)

fps = 0

cap = cv2.VideoCapture(0)

while 1:
        start_time = time.time()
        ret, img = cap.read()
        img = cv2.flip( img, 1 )
        cv2.putText(img, 'FPS: {:.2f}'.format(fps), (20, 20),fontFace=cv2.FONT_HERSHEY_SIMPLEX,fontScale=0.7,color=(0, 0, 255))
        
        faces = bleedfacedetector.ssd_detect(img)

        if faces ==[]:
                data = Float64MultiArray()
                #data.data = [input("motor 1 :"),input("motor 2 :"), input("motor 3 :"), input("motor 4:"),input("motor 5 :")]
                data.data = [-1,-1]
                pub.publish(data)
        else:
                faces=[faces[0]]
                
                for (x,y,w,h) in faces:
                        cv2.rectangle(img,(x,y),(x+w,y+h),(0,0,255),2)
                        cv2.putText(img,'Face Detected',(x,y+h+15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 2, cv2.LINE_AA)

                        data = Float64MultiArray()
                        #data.data = [input("motor 1 :"),input("motor 2 :"), input("motor 3 :"), input("motor 4:"),input("motor 5 :")]
                        data.data = [x,y]
                        pub.publish(data)
                
        
        cv2.imshow('img',img)
        fps= (1.0 / (time.time() - start_time))

        

        if cv2.waitKey(1) == ord('q'):
            break

        

cap.release()
cv2.destroyAllWindows()
