import numpy as np
import time

# Límites para theta1 y theta2 en radianes
theta1_min = np.deg2rad(0)
theta1_max = np.deg2rad(50)
theta2_min = np.deg2rad(0)
theta2_max = np.deg2rad(50)

"""
Se define la cinemática directa del manipulador SCARA para obtener la posición del efector final
"""
def F(theta):
    l_0 = 70
    l_1 = 21
    l_2 = 10
    l_3 = 21
    l_4 = 12
    l_5 = 18.5
    ang_0 = np.deg2rad(117)
    ang_1 = np.deg2rad(117) 
    ang_2 = ang_1+np.deg2rad(0) - theta[0]
    ang_3 = ang_2-np.deg2rad(0)-theta[0]
    ang_4 = ang_3-np.deg2rad(0)-theta[1]
    ang_5 = ang_4-np.deg2rad(0)-theta[1]

    f1 = l_0*np.cos(ang_0) + l_1*np.cos(ang_1) + l_2*np.cos(ang_2) + l_3*np.cos(ang_3) + l_4*np.cos(ang_4) + l_5*np.cos(ang_5)
    f2 = l_0*np.sin(ang_0) + l_1*np.sin(ang_1)+ l_2*np.sin(ang_2) + l_3*np.sin(ang_3) + l_4*np.sin(ang_4) + l_5*np.sin(ang_5)
    return np.array([f1, f2])

#Se crea una funcion para checkear que los angulos estan dentro de los limites
def check_limits(theta1, theta2):
    
    if theta1 < theta1_min or theta1 > theta1_max or theta2 < theta2_min or theta2 > theta2_max:
        return False
    return True

"""
Calcula la matriz jacobiana de una funcion f en un punto x
Parametros: 
    f: función de la que se calcula la matriz jacobiana (vector)
    x: punto en que se evalua la matriz jacobiana (vector)
    epsilon: error admisible para calcular la derivada (escalar)
Retorna:
    jacobian_matrix: matriz jacobiana de la función en el punto x (matriz)
"""
def jacobian(f, x, epsilon = 1e-1):
    n = len(x)
    jacobian_matrix = np.zeros((n, n))
    f_x = f(x)
    
    for i in range(n):
        perturbation = np.zeros(n)
        perturbation[i] = epsilon
        f_x_plus = f(x + perturbation)
        jacobian_matrix[:, i] = (f_x_plus - f_x) / epsilon
        
    return jacobian_matrix

"""
Obtener los parámetros (resolucion cinematica inversa)
NewtonRapson(funcion, punto inicial, posicion deseada)
punto incial representa los valores de los joints, como semilla,
en este caso son 3: dos rotaciones y uno de traslacion
posicion deseada es un vector de 3: x,y,z en el espacio cartesiano

"""
def NewtonRaphson(f, q0, target_pos, tol=1e-3, max_iter=1000):
    q = q0 #theta0 es el initial guess
    
    for _ in range(max_iter):
        # calcular jacobiano con la funcion anterior
        jacobiano_val = jacobian(f, q)
        # primero calcular la matriz inversa
        try:
            inv_jacobiano = np.linalg.inv(jacobiano_val)
        except np.linalg.LinAlgError:
            #si no es invertible, habrá un error, en ese caso se calcula la pseudoinversa
            inv_jacobiano = np.linalg.pinv(jacobiano_val)
            
        #aplicar metodo newton raphson
        if np.abs(np.linalg.det(jacobiano_val)) > tol:
            q_new = q - inv_jacobiano @ (f(q) - target_pos)

            if not check_limits(q_new[0], q_new[1]):
                q_new[0], q_new[1] = [np.random.uniform(theta1_min, theta1_max), np.random.uniform(theta2_min, theta2_max)]

        #si el determinante es cercano a 0, se agrega ruido al vector theta, para que no diverja 
        else:
            q_new = q + np.random.rand(len(q)) #se mueve theta aleatoriamente
        #si la norma de la diferencia entre theta_new y theta es menor que la tolerancia, se retorna theta_new
        if np.linalg.norm(q_new - q) < tol: # condiciones de convergencia
            return q_new 
        q = q_new
    raise ValueError("No se pudo encontrar la solución después de %d iteraciones" % max_iter)

def adecuacion_q(q):
    """
    Se realiza una transformacion de los angulos para que estén restringidos
    a un rango de -pi a pi
    """
    q_rad = [np.mod(q[0] + np.pi, 2 * np.pi) - np.pi, 
            np.mod(q[1] + np.pi, 2 * np.pi) - np.pi]
    return q_rad

def IK_NR(xyz,q):
    q0 = np.array(q)
    q_deg = NewtonRaphson(F, q0 = q0, target_pos = np.array(xyz))
    return q_deg

def puntos_array(q0, x_f, y_f, separation=2):
    x_i,y_i = F(q0)
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








