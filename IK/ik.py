import numpy as np
import matplotlib.pyplot as plt

######visualizar######
l_0 = 70
l_1 = 21
l_2 = 10
l_3 = 21
l_4 = 12
l_5 = 18.5

theta1=0.78125409 
theta2=0.79490095

class Linkage():
    def __init__(self, start_point, length, angle):
        self.start_point = start_point
        self.length = length
        self.angle = angle  # En grados para facilidad de uso
        self.end_point = self.calculate_end_point() 

    def calculate_end_point(self):
        # Convertir ángulo a radianes
        angle_rad = np.deg2rad(self.angle)
        end_x = self.start_point[0] + self.length * np.cos(angle_rad)
        end_y = self.start_point[1] + self.length * np.sin(angle_rad)
        return (end_x, end_y)

    def plot(self, ax):
        ax.plot([self.start_point[0], self.end_point[0]],
                [self.start_point[1], self.end_point[1]], '-o', label=f'{self.length} units @ {self.angle}°')


def plot_linkages(linkages):
    fig, ax = plt.subplots()
    for linkage in linkages:
        linkage.plot(ax)
    ax.set_aspect('equal', adjustable='datalim')
    ax.grid(True)
    plt.legend()
    plt.show()

ang_0 = 117
ang_1 = 117
ang_2 = ang_1 - np.rad2deg(float(theta1))
ang_3 = ang_2-np.rad2deg(float(theta1))
ang_4 = ang_3-np.rad2deg(float(theta2))
ang_5 = ang_4-np.rad2deg(float(theta2))


x = l_0*np.cos(np.deg2rad(ang_0)) + l_1*np.cos(np.deg2rad(ang_1)) + l_2*np.cos(np.deg2rad(ang_2)) + l_3*np.cos(np.deg2rad(ang_3)) + l_4*np.cos(np.deg2rad(ang_4)) + l_5*np.cos(np.deg2rad(ang_5))

y = l_0*np.sin(np.deg2rad(ang_0)) + l_1*np.sin(np.deg2rad(ang_1)) + l_2*np.sin(np.deg2rad(ang_2)) + l_3*np.sin(np.deg2rad(ang_3)) + l_4*np.sin(np.deg2rad(ang_4)) + l_5*np.sin(np.deg2rad(ang_5))

linkage0 = Linkage([0, 0], l_0, ang_0)
linkage1 = Linkage(linkage0.end_point, l_1, ang_1)
linkage2 = Linkage(linkage1.end_point, l_2, ang_2)
linkage3 = Linkage(linkage2.end_point, l_3, ang_3)
linkage4 = Linkage(linkage3.end_point, l_4, ang_4)
linkage5 = Linkage(linkage4.end_point, l_5, ang_5)
print(linkage5.end_point)
print(theta1,theta2)
plot_linkages([linkage0, linkage1, linkage2, linkage3, linkage4, linkage5])
