import pygame
import random
import math
from typing import List, Tuple

# Constantes
NUMBER_BODIES = 15
GRAVITATIONAL_CONSTANT = 1
DELTA_T = 0.001
CUTOFF = 100
EPSILON = 1e-2
MAX_VELOCITY = 10
WIDTH, HEIGHT = 500, 500

# Clase Vector2D para manejar operaciones vectoriales
class Vector2D:
    def __init__(self, x: float, y: float):
        self.x = x
        self.y = y

    def __add__(self, other: 'Vector2D') -> 'Vector2D':
        return Vector2D(self.x + other.x, self.y + other.y)

    def __sub__(self, other: 'Vector2D') -> 'Vector2D':
        return Vector2D(self.x - other.x, self.y - other.y)

    def __mul__(self, scalar: float) -> 'Vector2D':
        return Vector2D(self.x * scalar, self.y * scalar)

    def norm(self) -> float:
        return math.sqrt(self.x ** 2 + self.y ** 2)

    def limit(self, max_value: float) -> 'Vector2D':
        norm = self.norm()
        if norm > max_value:
            return self * (max_value / norm)
        return self

# Clase para el cuerpo (Body)
class Body:
    def __init__(self, position: Vector2D = None, velocity: Vector2D = None,
                 acceleration: Vector2D = None, mass: float = 300000.0, radius: float = 5.0,
                 color: Tuple[int, int, int] = None):
        self.position = position if position else Vector2D(0, 0)
        self.velocity = velocity if velocity else Vector2D(random.uniform(-1.0, 1.0), random.uniform(-1.0, 1.0))
        self.acceleration = acceleration if acceleration else Vector2D(0, 0)
        self.mass = mass
        self.radius = radius
        self.color = color if color else self.random_color()

    def random_position(self, max_x: int, max_y: int):
        """Asigna una posición aleatoria al cuerpo."""
        self.position = Vector2D(random.uniform(0.0, max_x), random.uniform(0.0, max_y))

    def random_color(self) -> Tuple[int, int, int]:
        """Genera un color aleatorio para el cuerpo."""
        return (random.randint(0, 255), random.randint(0, 255), random.randint(0, 255))

# Clase Physics para manejar la simulación de la física
class Physics:
    @staticmethod
    def calculate_gravitational_force(body1: Body, body2: Body) -> Vector2D:
        """Calcula la fuerza gravitacional entre dos cuerpos."""
        vector_distance = body2.position - body1.position
        distance = max(vector_distance.norm(), EPSILON)
        force_magnitude = GRAVITATIONAL_CONSTANT * body1.mass * body2.mass / (distance ** 2 + EPSILON)
        force = vector_distance * (force_magnitude / distance)  # Dirección y magnitud correctas
        return force

    @staticmethod
    def update_body(body: Body, force: Vector2D):
        """Actualiza la aceleración, velocidad y posición del cuerpo basado en la fuerza neta."""
        body.acceleration = force * (1 / body.mass)
        body.velocity = (body.velocity + body.acceleration * DELTA_T).limit(MAX_VELOCITY)
        body.position = body.position + body.velocity * DELTA_T

# Clase para la simulación principal
class Simulation:
    def __init__(self, bodies: List[Body]):
        self.bodies = bodies

    def step(self):
        """Realiza un paso de la simulación, calculando todas las fuerzas y actualizando posiciones."""
        for body in self.bodies:
            total_force = Vector2D(0, 0)
            for other_body in self.bodies:
                if body != other_body:
                    total_force += Physics.calculate_gravitational_force(body, other_body)
            Physics.update_body(body, total_force)

    def assign_orbital_velocities(self, central_body: Body):
        """Asigna velocidades iniciales a los cuerpos para que orbiten el cuerpo central."""
        for body in self.bodies:
            if body != central_body:
                # Vector entre el cuerpo central y el cuerpo
                vector_to_central = body.position - central_body.position
                distance_to_central = vector_to_central.norm()

                # Velocidad orbital circular
                orbital_velocity_magnitude = math.sqrt(GRAVITATIONAL_CONSTANT * central_body.mass / distance_to_central)

                # Dirección de la velocidad: perpendicular al vector hacia el cuerpo central
                tangent_direction = Vector2D(-vector_to_central.y, vector_to_central.x)  # Perpendicular
                body.velocity = tangent_direction * (orbital_velocity_magnitude / tangent_direction.norm())

# Inicialización de Pygame
pygame.init()
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption('Simulación de N Cuerpos con Órbitas Estables')

# Inicialización de los cuerpos
bodies = [Body() for _ in range(NUMBER_BODIES)]
for body in bodies:
    body.random_position(WIDTH, HEIGHT)

# Cuerpo central masivo
central_body = Body(position=Vector2D(WIDTH / 2, HEIGHT / 2), mass=10000000, radius=15, color=(255, 0, 0))
bodies.append(central_body)

# Inicialización de la simulación
simulation = Simulation(bodies)
simulation.assign_orbital_velocities(central_body)  # Asignar velocidades orbitales

# Bucle principal
running = True
while running:
    screen.fill((0, 0, 0))
    
    # Manejo de eventos
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    simulation.step()

    for body in bodies:
        pygame.draw.circle(screen, body.color, (int(body.position.x), int(body.position.y)), int(body.radius), 0)

    pygame.display.flip()

# Cierre de Pygame
pygame.quit()
