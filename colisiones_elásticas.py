import pygame
import math
import random

# Clase para representar un vector en 2D
class Vector2D:
    def __init__(self, x=0.0, y=0.0):
        """Inicializa el vector con coordenadas x e y."""
        self.x = x
        self.y = y

    def __add__(self, other):
        """Suma dos vectores (sobrecarga del operador +)."""
        return Vector2D(self.x + other.x, self.y + other.y)

    def __sub__(self, other):
        """Resta dos vectores (sobrecarga del operador -)."""
        return Vector2D(self.x - other.x, self.y - other.y)

    def __mul__(self, scalar):
        """Multiplica el vector por un escalar (sobrecarga del operador *)."""
        return Vector2D(self.x * scalar, self.y * scalar)

    def __truediv__(self, scalar):
        """Divide el vector por un escalar (sobrecarga del operador /)."""
        if scalar != 0:
            return Vector2D(self.x / scalar, self.y / scalar)
        else:
            raise ValueError("No se puede dividir por 0.")

    def dot(self, other):
        """Calcula el producto punto (dot product) entre dos vectores."""
        return self.x * other.x + self.y * other.y

    def magnitude(self):
        """Calcula la magnitud (norma) del vector."""
        return math.sqrt(self.x ** 2 + self.y ** 2)

    def normalize(self):
        """Devuelve un vector unitario (dirección) del vector actual."""
        mag = self.magnitude()
        if mag != 0:
            return Vector2D(self.x / mag, self.y / mag)
        else:
            return Vector2D()

    def __repr__(self):
        """Representación del vector en formato legible."""
        return f"Vector2D({self.x}, {self.y})"

# Clase que representa un cuerpo en la simulación
class Body:
    def __init__(self, position, velocity, mass, radius, color):
        """Inicializa un cuerpo con posición, velocidad, masa, radio y color."""
        self.position = position
        self.velocity = velocity
        self.mass = mass
        self.radius = radius
        self.color = color

    def update(self, dt):
        """Actualiza la posición del cuerpo usando su velocidad."""
        self.position += self.velocity * dt

# Clase que maneja las colisiones
class Collision:
    @staticmethod
    def detect_collision(body1, body2):
        """Detecta si dos cuerpos están colisionando."""
        distance = (body1.position - body2.position).magnitude()
        return distance < (body1.radius + body2.radius)

    @staticmethod
    def resolve_collision(body1, body2):
        """Resuelve una colisión elástica entre dos cuerpos."""
        normal = body1.position - body2.position
        distance = normal.magnitude()
        
        if distance == 0:  # Evita divisiones por cero
            return
        
        # Normaliza el vector normal
        normal = normal * (1 / distance)
        
        # Calcula la velocidad relativa
        relative_velocity = body1.velocity - body2.velocity
        speed = relative_velocity.dot(normal)

        # Si los cuerpos se están separando, no hay colisión que resolver
        if speed >= 0:
            return
        
        # Cálculo del impulso (fórmula de colisión elástica)
        impulse = (2 * speed) / (body1.mass + body2.mass)
        
        # Actualiza las velocidades de ambos cuerpos
        body1.velocity -= normal * impulse * body2.mass
        body2.velocity += normal * impulse * body1.mass

# Clase que representa la simulación
class Simulation:
    def __init__(self, width, height):
        """Inicializa la simulación con una lista de cuerpos y dimensiones."""
        self.width = width
        self.height = height
        self.bodies = []

    def add_body(self, body):
        """Agrega un cuerpo a la simulación."""
        self.bodies.append(body)

    def step(self, dt=1):
        """Actualiza la simulación, incluyendo detección y resolución de colisiones."""
        for i in range(len(self.bodies)):
            self.bodies[i].update(dt)

            # Detección y resolución de colisiones con otros cuerpos
            for j in range(i + 1, len(self.bodies)):
                if Collision.detect_collision(self.bodies[i], self.bodies[j]):
                    Collision.resolve_collision(self.bodies[i], self.bodies[j])

            # Colisiones con los bordes de la pantalla
            body = self.bodies[i]
            if body.position.x - body.radius < 0 or body.position.x + body.radius > self.width:
                body.velocity.x *= -1  # Invierte la velocidad en x
            if body.position.y - body.radius < 0 or body.position.y + body.radius > self.height:
                body.velocity.y *= -1  # Invierte la velocidad en y

# Función principal que corre la simulación
def run_simulation():
    pygame.init()
    width, height = 800, 600
    screen = pygame.display.set_mode((width, height))
    pygame.display.set_caption("Simulación de Cuerpos con Colisiones Elásticas")

    simulation = Simulation(width, height)

    # Crear cuerpos aleatorios
    for _ in range(10):
        position = Vector2D(random.uniform(50, width - 50), random.uniform(50, height - 50))
        velocity = Vector2D(random.uniform(-100, 100), random.uniform(-100, 100))
        mass = random.uniform(1, 10)
        radius = random.uniform(10, 20)
        color = [random.randint(0, 255) for _ in range(3)]
        body = Body(position, velocity, mass, radius, color)
        simulation.add_body(body)

    clock = pygame.time.Clock()
    running = True

    while running:
        dt = clock.tick(60) / 1000  # 60 FPS, el tiempo delta en segundos

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        simulation.step(dt)

        screen.fill((0, 0, 0))

        for body in simulation.bodies:
            pygame.draw.circle(screen, body.color, (int(body.position.x), int(body.position.y)), int(body.radius), 0)

        pygame.display.flip()

    pygame.quit()

if __name__ == "__main__":
    run_simulation()
