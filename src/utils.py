import math
import random

def euclidean(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])

def roulette_wheel(weights):
    s = sum(weights)
    r = random.uniform(0, s)
    cumulative = 0
    for i, w in enumerate(weights):
        cumulative += w
        if cumulative >= r:
            return i
    return len(weights) - 1
