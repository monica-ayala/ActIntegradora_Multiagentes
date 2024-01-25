# Autor: Ivan Olmos Pineda


import pygame
from pygame.locals import *

# Cargamos las bibliotecas de OpenGL
from OpenGL.GL import *
from OpenGL.GLU import *
from OpenGL.GLUT import *

import random
import math
import numpy as np

import random
import math
import numpy as np

class Robot:
    def __init__(self, dim, vel, Scala, cube_list=None):
        # vertices del cubo
        self.points = np.array([[-1.0, -1.0, 1.0], [1.0, -1.0, 1.0], [1.0, -1.0, -1.0], [-1.0, -1.0, -1.0],
                                [-1.0, 1.0, 1.0], [1.0, 1.0, 1.0], [1.0, 1.0, -1.0], [-1.0, 1.0, -1.0]])

        self.DimBoard = dim
        # Se inicializa una posicion aleatoria en el tablero
        self.Position = []
        self.Position.append(random.randint(-1 * self.DimBoard, self.DimBoard))
        self.Position.append(5.0)
        self.Position.append(random.randint(-1 * self.DimBoard, self.DimBoard))
        # Se inicializa un vector de direccion aleatorio
        self.Direction = []
        self.Direction.append(random.random())
        self.Direction.append(5.0)
        self.Direction.append(random.random())
        # Se normaliza el vector de direccion
        m = math.sqrt(self.Direction[0] * self.Direction[0] + self.Direction[2] * self.Direction[2])
        self.Direction[0] /= m
        self.Direction[2] /= m
        # Se cambia la magnitud del vector direccion
        self.Direction[0] *= vel
        self.Direction[2] *= vel
        self.Robots = []
        self.collision = 0
        self.radio = 2
        self.Scala = Scala
        self.cube_list = [] if cube_list is None else cube_list

    def getRobots(self, Ncubos):
        self.Robots = Ncubos

    def update(self, cubos):
        self.CollitionDetection(cubos)

        if self.collision == 0:
            new_x = self.Position[0] + self.Direction[0]
            new_z = self.Position[2] + self.Direction[2]

            if abs(new_x) <= self.DimBoard:
                self.Position[0] = new_x
            else:
                self.Direction[0] *= -1.0
                self.Position[0] += self.Direction[0]

            if abs(new_z) <= self.DimBoard:
                self.Position[2] = new_z
            else:
                self.Direction[2] *= -1.0
                self.Position[2] += self.Direction[2]
        else:

            # Si hay colisión, dirigir el robot hacia el origen (0, 0, 0) de manera más lenta
            direction_to_origin = np.array([0.0 - self.Position[0], 0.0 - self.Position[1], 0.0 - self.Position[2]])
            direction_to_origin /= np.linalg.norm(direction_to_origin)  # Normalizar el vector

            # Generar velocidad aleatoria entre 0.1 y 1.0
            speed_factor = random.uniform(0.1, 0.3)

            self.Direction = direction_to_origin * (self.Scala * speed_factor)  # Ajustar velocidad
            self.Position[0] += self.Direction[0]
            self.Position[2] += self.Direction[2]

            # Verificar si el robot ha llegado al origen y restablecer la colisión a 0
            if np.allclose(self.Position, [0.0, 0.0, 0.0], atol=1e-2):
                self.Direction[0]+=0.5
                print("Robot ha llegado al origen. Restableciendo colisión a 0.")
                self.collision = 0

        self.collision = 0


    def CollitionDetection(self, cubos):
        for obj in cubos:
            if self != obj:
                d_x = self.Position[0] - obj.Position[0]
                d_z = self.Position[2] - obj.Position[2]
                d_c = math.sqrt(d_x * d_x + d_z * d_z)

                # Usar radio en la comparación
                if d_c - (self.radio + obj.radio) < 10.0:
                    self.collision = 1
                    obj.Scale = 0

    def drawFaces(self):
        glBegin(GL_QUADS)
        glVertex3fv(self.points[0])
        glVertex3fv(self.points[1])
        glVertex3fv(self.points[2])
        glVertex3fv(self.points[3])
        glEnd()
        glBegin(GL_QUADS)
        glVertex3fv(self.points[4])
        glVertex3fv(self.points[5])
        glVertex3fv(self.points[6])
        glVertex3fv(self.points[7])
        glEnd()
        glBegin(GL_QUADS)
        glVertex3fv(self.points[0])
        glVertex3fv(self.points[1])
        glVertex3fv(self.points[5])
        glVertex3fv(self.points[4])
        glEnd()
        glBegin(GL_QUADS)
        glVertex3fv(self.points[1])
        glVertex3fv(self.points[2])
        glVertex3fv(self.points[6])
        glVertex3fv(self.points[5])
        glEnd()
        glBegin(GL_QUADS)
        glVertex3fv(self.points[2])
        glVertex3fv(self.points[3])
        glVertex3fv(self.points[7])
        glVertex3fv(self.points[6])
        glEnd()
        glBegin(GL_QUADS)
        glVertex3fv(self.points[3])
        glVertex3fv(self.points[0])
        glVertex3fv(self.points[4])
        glVertex3fv(self.points[7])
        glEnd()

    def draw(self):
        glPushMatrix()
        glTranslatef(self.Position[0], self.Position[1], self.Position[2])
        glScaled(self.Scala, self.Scala, self.Scala)
        glColor3f(1.0, 0.0, 1.0)

        for cube in self.cube_list:
            cube.update()
            cube.draw()

        self.drawFaces()
        glPopMatrix()
