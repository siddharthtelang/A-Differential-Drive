import numpy as np
import math
from Obstacle import *


def halfRound(n):
    return round(2*n)/2


def toRadian(angle):
    return np.pi * angle / 180
def toDegree(angle):
    return 180 * angle / np.pi