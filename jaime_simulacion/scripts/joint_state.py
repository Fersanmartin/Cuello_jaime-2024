#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import time
from std_msgs.msg import Float64MultiArray

joint=[0,0,0,0,0,0]
def read(data):
    global joint
    joint=data.data
    
def talker():
    global joint 

    pub = rospy.Publisher('joint_states', JointState, queue_size=10)
    rospy.init_node('joint_state_publisher')
    rate = rospy.Rate(10) # 10hz

    while not rospy.is_shutdown():  # Bucle para publicar continuamente
        rospy.Subscriber("joint_node", Float64MultiArray, read)

        hello_str = JointState()
        hello_str.header = Header()
        hello_str.header.stamp = rospy.Time.now()
        hello_str.name = ['l1_to_base_link','l2_to_l1','l3_to_l2','l4_to_l3','l5_to_l4','l6_to_l5']  # Lista de nombres de articulaciones
        hello_str.position = joint    # Lista de posiciones correspondientes
        hello_str.velocity = []
        hello_str.effort = []

        pub.publish(hello_str)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass


