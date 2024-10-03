import rospy
from std_msgs.msg import Float64MultiArray, Int32

def main():
    # Inicializa el nodo
    rospy.init_node('goal_position_publisher', anonymous=True)
    
    # Crea el publicador para las posiciones goal en un tópico
    goal_pub = rospy.Publisher('Goal_pos', Float64MultiArray, queue_size=1)
    
    # Crea el publicador para la confirmación en un tópico
    conf_pub = rospy.Publisher('confirmation', Int32, queue_size=1)

    rate = rospy.Rate(100)  # Define la tasa de publicación (1 Hz)

    while not rospy.is_shutdown():
        # Solicita al usuario ingresar x e y
        try:
            x = float(input("Ingrese el valor de x para la posición objetivo: "))
            y = float(input("Ingrese el valor de y para la posición objetivo: "))
        except ValueError:
            print("Por favor, ingrese valores numéricos válidos.")
            continue
        
        # Crea el mensaje de posición objetivo
        goal_msg = Float64MultiArray()
        goal_msg.data = [x, y]
        
        # Publica la posición objetivo
        goal_pub.publish(goal_msg)
        rospy.loginfo(f"Publicando posición objetivo: x={x}, y={y}")

        s=0
        while s==0:
            # Espera confirmación del usuario
            confirmation = input("¿Confirmas esta posición? (s/n): ").strip().lower()
            
            if confirmation == 's':
                # Publica la confirmación (1 para confirmar)
                conf_msg = Int32()
                conf_msg.data = 1
                conf_pub.publish(conf_msg)
                rospy.loginfo("Confirmación enviada.")
            elif confirmation == "1":
                s=1
            else:
                # Publica la no-confirmación (0 para no confirmar)
                conf_msg = Int32()
                conf_msg.data = 0
                conf_pub.publish(conf_msg)
                rospy.loginfo("Confirmación no enviada.")

        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass