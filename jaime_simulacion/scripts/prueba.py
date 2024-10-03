import rospy
from std_msgs.msg import Float64
import time 


rospy.init_node('IK', anonymous=True)
tiempo=time.time()
ang=0
while not rospy.is_shutdown():
  if (time.time()-tiempo)>1:
    # Crea un objeto para publicar los datos
    pub = rospy.Publisher('/jaime/joint1_position_controller/command', Float64, queue_size=1)
    
    
    data = Float64()
    data.data=ang

    pub.publish(data)
    ang+=0.1
    if ang>0.7:
      ang=0
    tiempo=time.time()