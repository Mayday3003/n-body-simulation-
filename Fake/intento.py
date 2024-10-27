import pygame
import random
import math
from typing import List, Tuple

# Constantes
NUMBER_BODIES = 15
GRAVITATIONAL_CONSTANT = 1
DELTA_T = 0.0001
CUTOFF = 100
WIDTH, HEIGHT = 500, 500
FPS = 60

# Clase para representar puntos en el QuadTree
class Point:
    def __init__(self, x: float, y: float, body: 'Body'):
        self.x = x
        self.y = y
        self.body = body  # Referencia al cuerpo

# Clase para definir los límites (rectángulos) de consulta en el QuadTree
class Rectangle:
    def __init__(self, x: float, y: float, w: float, h: float):
        self.x = x
        self.y = y
        self.w = w
        self.h = h

    def contains(self, point: Point) -> bool:
        """Verifica si un punto está dentro de los límites del rectángulo."""
        return (self.x - self.w <= point.x <= self.x + self.w and
                self.y - self.h <= point.y <= self.y + self.h)

    def intersects(self, range: 'Rectangle') -> bool:
        """Verifica si el rectángulo actual se intersecta con otro."""
        return not (range.x - range.w > self.x + self.w or
                    range.x + range.w < self.x - self.w or
                    range.y - range.h > self.y + self.h or
                    range.y + range.h < self.y - self.h)

# Clase para el QuadTree, que divide el espacio para consultas más rápidas
class QuadTree:
    def __init__(self, boundary: Rectangle, capacity: int):
        self.boundary = boundary
        self.capacity = capacity
        self.points: List[Point] = []
        self.divided = False

    def subdivide(self):
        """Divide el QuadTree en cuatro sub-árboles."""
        x, y, w, h = self.boundary.x, self.boundary.y, self.boundary.w / 2, self.boundary.h / 2
        self.northeast = QuadTree(Rectangle(x + w, y - h, w, h), self.capacity)
        self.northwest = QuadTree(Rectangle(x - w, y - h, w, h), self.capacity)
        self.southeast = QuadTree(Rectangle(x + w, y + h, w, h), self.capacity)
        self.southwest = QuadTree(Rectangle(x - w, y + h, w, h), self.capacity)
        self.divided = True

    def insert(self, point: Point) -> bool:
        """Inserta un punto en el QuadTree."""
        if not self.boundary.contains(point):
            return False

        if len(self.points) < self.capacity:
            self.points.append(point)
            return True
        else:
            if not self.divided:
                self.subdivide()

            return (self.northeast.insert(point) or
                    self.northwest.insert(point) or
                    self.southeast.insert(point) or
                    self.southwest.insert(point))

    def query(self, range: Rectangle, found: List[Point]) -> List[Point]:
        """Encuentra todos los puntos en un rango dado."""
        if not self.boundary.intersects(range):
            return found

        for p in self.points:
            if range.contains(p):
                found.append(p)

        if self.divided:
            self.northwest.query(range, found)
            self.northeast.query(range, found)
            self.southwest.query(range, found)
            self.southeast.query(range, found)

        return found

# Clase para los cuerpos en la simulación
class Body:
    def __init__(self, position: List[float] = None, velocity: List[float] = None,
                 acceleration: List[float] = None, mass: float = 300000.0, radius: float = 5.0,
                 color: Tuple[int, int, int] = None):
        self.position = position if position else [0, 0]
        self.velocity = velocity if velocity else [random.uniform(-1.0, 1.0), random.uniform(-1.0, 1.0)]
        self.acceleration = acceleration if acceleration else [0, 0]
        self.mass = mass
        self.radius = radius  # Radio único para cada cuerpo
        self.color = color if color else self.random_color()  # Color del cuerpo

    def random_position(self, max_x: int, max_y: int):
        """Asigna una posición aleatoria al cuerpo dentro de los límites."""
        self.position[0] = random.uniform(0.0, 1.0) * max_x
        self.position[1] = random.uniform(0.0, 1.0) * max_y

    def random_color(self) -> Tuple[int, int, int]:
        """Genera un color aleatorio para el cuerpo."""
        return (random.randint(0, 255), random.randint(0, 255), random.randint(0, 255))

# Funciones auxiliares para operar con vectores
def vector_subtract(v1: List[float], v2: List[float]) -> List[float]:
    return [v1[0] - v2[0], v1[1] - v2[1]]

def scalar_multiply(n: float, v: List[float]) -> List[float]:
    return [n * v[0], n * v[1]]

def vector_add(v1: List[float], v2: List[float]) -> List[float]:
    return [v1[0] + v2[0], v1[1] + v2[1]]

def norm_of_vector(v: List[float]) -> float:
    return math.sqrt(v[0] ** 2 + v[1] ** 2)

# Función para calcular las fuerzas y actualizar los cuerpos
def calcular(bodies: List[Body]):
    boundary = Rectangle(WIDTH / 2, HEIGHT / 2, WIDTH / 2, HEIGHT / 2)
    qtree = QuadTree(boundary, 4)

    # Insertar los cuerpos en el QuadTree
    for body in bodies:
        point = Point(body.position[0], body.position[1], body)
        qtree.insert(point)

    # Calcular la dinámica de los cuerpos
    for body in bodies:
        sum_forces = [0, 0]
        range_query = Rectangle(body.position[0], body.position[1], 100, 100)
        found_points = qtree.query(range_query, [])

        for point in found_points:
            other_body = point.body
            if other_body is body:
                continue

            vector_distancia = vector_subtract(other_body.position, body.position)
            norma_distancia = norm_of_vector(vector_distancia)

            if norma_distancia > CUTOFF:
                continue

            # Calcular la fuerza entre los cuerpos
            f_ij = scalar_multiply(
                GRAVITATIONAL_CONSTANT * body.mass * other_body.mass * math.pow(norma_distancia + 10, -3),
                vector_distancia
            )
            sum_forces = vector_add(sum_forces, f_ij)

        # Actualizar aceleración, velocidad y posición
        body.acceleration = scalar_multiply(1 / body.mass, sum_forces)
        body.velocity = vector_add(body.velocity, scalar_multiply(DELTA_T, body.acceleration))
        body.position = vector_add(body.position, scalar_multiply(DELTA_T, body.velocity))

# Función para dibujar los cuerpos
def draw_bodies(window, bodies):
    for body in bodies:
        pos_x = int(body.position[0])
        pos_y = int(HEIGHT - body.position[1])
        pygame.draw.circle(window, body.color, (pos_x, pos_y), int(body.radius), 0)

# Función para dibujar el QuadTree
def draw_quadtree(qtree, surface):
    pygame.draw.rect(surface, (255, 255, 255), pygame.Rect(
        qtree.boundary.x - qtree.boundary.w, HEIGHT - qtree.boundary.y - qtree.boundary.h,
        qtree.boundary.w * 2, qtree.boundary.h * 2), 1)

    if qtree.divided:
        draw_quadtree(qtree.northeast, surface)
        draw_quadtree(qtree.northwest, surface)
        draw_quadtree(qtree.southeast, surface)
        draw_quadtree(qtree.southwest, surface)

# Configuración inicial de Pygame
pygame.init()
window = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption('n-Bodies with QuadTree Optimization')
clock = pygame.time.Clock()

# Crear los cuerpos y asignar posiciones aleatorias
bodies = [Body(radius=random.uniform(3, 8)) for _ in range(NUMBER_BODIES)]
for body in bodies:
    body.random_position(WIDTH, HEIGHT)

# Control de simulación
paused = False

# Ciclo principal de Pygame
running = True
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_p:
                paused = not paused
            if event.key == pygame.K_r:
                bodies = [Body(radius=random.uniform(3, 8)) for _ in range(NUMBER_BODIES)]
                for body in bodies:
                    body.random_position(WIDTH, HEIGHT)

    window.fill((0, 0, 0))

    if not paused:
        calcular(bodies)

    draw_bodies(window, bodies)

    pygame.display.update()
    clock.tick(FPS)

pygame.quit()
