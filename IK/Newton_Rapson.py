import numpy as np
import sympy as sp
import rospy
from std_msgs.msg import Float64MultiArray
import time 


l_0 = 70
l_1 = 21
l_2 = 10
l_3 = 21
l_4 = 12
l_5 = 18.5

# Límites para theta1 y theta2 en radianes
theta1_min = np.deg2rad(0)
theta1_max = np.deg2rad(50)
theta2_min = np.deg2rad(0)
theta2_max = np.deg2rad(50)
# Función para verificar y ajustar los ángulos si están fuera de los límites
def check_limits(theta1, theta2):
    if theta1 < theta1_min or theta1 > theta1_max or theta2 < theta2_min or theta2 > theta2_max:
        return False
    return True

def Newton_R(x_input,y_input,punto_inicial=[-15.310485937923819,129.126881116257]):
    # Ejemplo de creación y visualización de un sistema de linkages
    global l_0,l_1,l_2,l_3,l_4,l_5

    a=sp.Symbol("a")
    b=sp.Symbol("b")

    ang_0 = np.deg2rad(117)
    ang_1 = np.deg2rad(117) 
    ang_2 = ang_1+np.deg2rad(5) - a
    ang_3 = ang_2-np.deg2rad(60)-a
    ang_4 = ang_3-np.deg2rad(0)-b
    ang_5 = ang_4-np.deg2rad(17)-b


    x = l_0*sp.cos(ang_0) + l_1*sp.cos(ang_1) + l_2*sp.cos(ang_2) + l_3*sp.cos(ang_3) + l_4*sp.cos(ang_4) + l_5*sp.cos(ang_5)
    y = l_0*sp.sin(ang_0) + l_1*sp.sin(ang_1)+ l_2*sp.sin(ang_2) + l_3*sp.sin(ang_3) + l_4*sp.sin(ang_4) + l_5*sp.sin(ang_5)

   

    


    # Suposiciones iniciales para theta1 y theta2
    initial_conditions = [
        (np.deg2rad(0), np.deg2rad(0)),
        (np.deg2rad(50), np.deg2rad(10)),
        (np.deg2rad(10), np.deg2rad(50)),
        (np.deg2rad(50), np.deg2rad(50))  # Agregar más si es necesario
    ]



    ########################Newton rapson##############################


    x_val=x_input
    y_val=y_input

    # Calcular la distancia desde el origen a (x_val, y_val)
    distance = np.sqrt((float(x_val) - l_0*np.cos(ang_0) - l_1*np.cos(ang_1))**2 + (float(y_val) - l_0*np.sin(ang_0) - l_1*np.sin(ang_1))**2)
    max_reach =l_2 + l_3 + l_4 + l_5
    if distance > max_reach:
        print("El punto (x_val, y_val) está fuera del alcance del mecanismo.")
        return punto_inicial


    n=0
    i=0
    theta1=punto_inicial[0]
    theta2=punto_inicial[1]

    while n<100:

        f1=(x.subs({a: theta1, b: theta2}).evalf()  - x_val)
        f2=(y.subs({a: theta1, b: theta2}).evalf()  - y_val)
        if (f1 and f2)>1:
            f1=f1/10
            f2=f2/10
        # Calcular las derivadas parciales

        dx_da = float(sp.diff(x, a).subs({a: theta1, b: theta2}).evalf())
        dx_db = float(sp.diff(x, b).subs({a: theta1, b: theta2 }).evalf())
        dy_da = float(sp.diff(y, a).subs({a: theta1, b: theta2}).evalf())
        dy_db = float(sp.diff(y, b).subs({a: theta1, b: theta2}).evalf())

        J = np.matrix([[dx_da,dx_db],[dy_da,dy_db]]).I  #Jacobian matrix
        f3 = np.matrix([[f1],[f2]])
        T = np.matrix([[theta1],[theta2]])
        Delta = J*f3

            
        E = T-Delta
        theta1 = np.squeeze(np.asarray(E[0]))
        theta2 = np.squeeze(np.asarray(E[1]))
        if i<3:
            if not check_limits(theta1,theta2):
                
                theta1=np.deg2rad(initial_conditions[i][0])
                theta2=np.deg2rad(initial_conditions[i][1])
                i+=1
                
                
        n+=1

    if not check_limits(theta1,theta2):
        print("valor no alcanzado")
        theta1,theta2 = punto_inicial
    return [theta1,theta2]


def puntos_array(x_i, x_f, y_i, y_f, separation=2):
    # Calculate the distance between start and end points
    distance = np.sqrt((x_f - x_i)**2 + (y_f - y_i)**2)
    
    # Calculate the number of points needed based on the separation distance
    num_points = int(distance / separation) + 1
    
    # Generate linearly spaced points for x and y coordinates
    x_points = np.linspace(x_i, x_f, num_points)
    y_points = np.linspace(y_i, y_f, num_points)
    
    # Combine x and y coordinates into a single array of points
    points = np.column_stack((x_points, y_points))
    
    return points

conf=0
def callback(data):
    global conf
    conf=data.data[0]
    return 

x_i,y_i =[0,110]

x,y=[float(input("x: ")),float(input("y: "))]

puntos=puntos_array(x_i,x,y_i,y)




angulos=[]
for i in range(len(puntos)):
    x,y=puntos[i]
    theta1,theta2= Newton_R(x,y,punto_inicial=[x_i,y_i])
    x_i,y_i=[float(theta1),float(theta2)]
    print(x_i,y_i)
    angulos.append([x_i,y_i])

ang=[]
for i in range(len(puntos)):
    ang.append(angulos[len(angulos)-1-i])  

angulos.extend(ang)
print(len(angulos))
tiempo=time.time()    
i=0
rospy.init_node('IK', anonymous=True)

while not rospy.is_shutdown():
  # Crea un objeto para publicar los datos
    theta1,theta2 = angulos[i]
    rospy.Subscriber('confirmacion', Float64MultiArray, callback)
    
    pub = rospy.Publisher('Newton_R', Float64MultiArray, queue_size=1)
    
    data = Float64MultiArray()
    data.data = [np.rad2deg(float(theta1)),np.rad2deg(float(theta2))]
    pub.publish(data)
    if conf==1 and (time.time()-tiempo)>0.5:
        if i<len(angulos):
            print(i)
            i+=1
            tiempo=time.time()
    
