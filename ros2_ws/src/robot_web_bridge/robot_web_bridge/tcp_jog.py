"""Przyrost pozy TCP w symulacji; bez transportu i komend napędów."""
from copy import deepcopy
import math


def multiply(a, b):
    x, y, z, w = a
    u, v, t, s = b
    return (w*u+x*s+y*t-z*v, w*v-x*t+y*s+z*u,
            w*t+x*v-y*u+z*s, w*s-x*u-y*v-z*t)


def increment_pose(pose, frame, axis, distance):
    """distance: metry lub radiany; obroty wokół początku TCP."""
    result = deepcopy(pose)
    q = (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
    index = ('x', 'y', 'z', 'rx', 'ry', 'rz').index(axis) % 3
    vector = [0., 0., 0.]
    vector[index] = distance
    if axis in ('x', 'y', 'z'):
        if frame == 'tool':
            vector = multiply(multiply(q, (*vector, 0.)), (-q[0], -q[1], -q[2], q[3]))[:3]
        for name, value in zip(('x', 'y', 'z'), vector):
            setattr(result.position, name, getattr(result.position, name) + value)
    else:
        rotation = [0., 0., 0., math.cos(distance/2)]
        rotation[index] = math.sin(distance/2)
        orientation = multiply(rotation, q) if frame == 'base' else multiply(q, rotation)
        norm = math.sqrt(sum(value*value for value in orientation))
        for name, value in zip(('x', 'y', 'z', 'w'), orientation):
            setattr(result.orientation, name, value/norm)
    return result
