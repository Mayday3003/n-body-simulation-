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
