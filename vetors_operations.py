import math

def add_vectors(v1, v2):
    v1[0] += v2[0]
    v1[1] += v2[1]

def vector_add(v1, v2):
    return [v1[0] + v2[0], v1[1] + v2[1]]

def vector_subtract(v1, v2):
    return [v1[0] - v2[0], v1[1] - v2[1]]

def scalar_multiply(n, v):
    return [n * v[0], n * v[1]]

def norm_of_vector(v):
    return math.sqrt(v[0] * v[0] + v[1] * v[1])
