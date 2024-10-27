import pygame
import random
import math
from multiprocessing import Pool

# Constantes
number_bodies = 30
GRAVITATIONAL_CONSTANT = 1
DELTA_T = 0.0001
CUTOFF = 1000000
width, height = 500, 500
radius = 2

# Clase Body (adaptada del código anterior)
class Body:
    def __init__(self, position=None, velocity=None, acceleration=None, mass=300000.0):
        if position is None:
            position = [0, 0]
        if velocity is None:
            velocity = [random.uniform(-1.0, 1.0), random.uniform(-1.0, 1.0)]
        if acceleration is None:
            acceleration = [0, 0]
        
        self.position = position
        self.velocity = velocity
        self.acceleration = acceleration
        self.mass = mass

    def random_position(self, max_x, max_y):
        self.position[0] = random.uniform(0.0, 1.0) * max_x
        self.position[1] = random.uniform(0.0, 1.0) * max_y

# Funciones auxiliares para operar con vectores
def vector_subtract(v1, v2):
    return [v1[0] - v2[0], v1[1] - v2[1]]

def scalar_multiply(n, v):
    return [n * v[0], n * v[1]]

def vector_add(v1, v2):
    return [v1[0] + v2[0], v1[1] + v2[1]]

def norm_of_vector(v):
    return math.sqrt(v[0] ** 2 + v[1] ** 2)

# Función calcular la dinámica de los cuerpos
def calcular(bodies):
    for i in range(number_bodies):
        sum_forces_i = [0, 0]
        for j in range(number_bodies):
            if i == j:
                continue

            vector_distancia = vector_subtract(bodies[j].position, bodies[i].position)
            norma_distancia = norm_of_vector(vector_distancia)

            if norma_distancia > CUTOFF:
                continue

            f_ij = scalar_multiply(
                GRAVITATIONAL_CONSTANT * bodies[i].mass * bodies[j].mass * math.pow(norma_distancia + 10, -3),
                vector_distancia
            )
            sum_forces_i = vector_add(sum_forces_i, f_ij)

        bodies[i].acceleration = scalar_multiply(1 / bodies[i].mass, sum_forces_i)
        bodies[i].velocity = vector_add(bodies[i].velocity, scalar_multiply(DELTA_T, bodies[i].acceleration))
        bodies[i].position = vector_add(bodies[i].position, scalar_multiply(DELTA_T, bodies[i].velocity))

# Configuración inicial de Pygame
pygame.init()
window = pygame.display.set_mode((width, height))
pygame.display.set_caption('n-Bodies')

# Crear los cuerpos y asignarles posiciones aleatorias
bodies = [Body() for _ in range(number_bodies)]
for body in bodies:
    body.random_position(width, height)

# Crear los círculos para cada cuerpo
circles = []
for _ in range(number_bodies):
    circle = pygame.Surface((2 * radius, 2 * radius), pygame.SRCALPHA)
    random_color = (random.randint(0, 255), random.randint(0, 255), random.randint(0, 255))
    pygame.draw.circle(circle, random_color, (radius, radius), radius)
    circles.append(circle)

# Ciclo principal del programa
running = True
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    # Limpiar la pantalla
    window.fill((0, 0, 0))

    # Calcular la dinámica de los cuerpos
    calcular(bodies)

    # Dibujar los cuerpos en la pantalla
    for i in range(number_bodies):
        pos_x = int(bodies[i].position[0])
        pos_y = int(height - bodies[i].position[1])
        window.blit(circles[i], (pos_x - radius, pos_y - radius))

    # Actualizar la ventana
    pygame.display.flip()

pygame.quit()
