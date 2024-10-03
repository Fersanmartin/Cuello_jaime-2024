import rospy
from std_msgs.msg import Float64MultiArray

theta1 = 0
theta2 = 0

def callback(data):
    global theta1, theta2
    theta1 = data.data[0]
    theta2 = data.data[1]

def main():
    rospy.init_node('joint_publisher', anonymous=True)

    # Crear el Publisher una vez
    pub = rospy.Publisher('joint_node', Float64MultiArray, queue_size=10)
    
    # Crear el Subscriber una vez
    rospy.Subscriber('Newton_R', Float64MultiArray, callback)
    
    # Controlar la frecuencia de publicación (10 Hz en este ejemplo)
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        data = Float64MultiArray()
        data.data = [theta1, theta1, theta2, theta2, 0]
        
        pub.publish(data)
        
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

