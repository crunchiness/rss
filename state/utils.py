import numpy as np
from math import pi, exp, sqrt


def norm_pdf(x, mu, sigma):
    """
    Normal distribution pdf
    implemented from http://stackoverflow.com/a/8669381/3160671
    """
    u = (x - mu) / abs(sigma)
    y = (1 / (sqrt(2 * pi) * abs(sigma))) * exp(-u * u / 2)
    return y

def intersects_at((p, r), (q, s)):
    """
    Line segments p to p+r and q to q+s
    implemented from http://stackoverflow.com/a/565282
    :return: point where segments intersect or None if they don't
    """
    if np.cross(r, s) == 0:
        return None
    qp = np.subtract(q, p)
    rs = np.cross(r, s)
    t = np.cross(qp, s) / rs
    u = np.cross(qp, r) / rs
    if 0 <= t <= 1 and 0 <= u <= 1:
        return np.add(p, np.multiply(t, r))
    else:
        return None

def intersects(vec1, vec2):
    return intersects_at(vec1, vec2) is not None

def at_orientation(vectors, orientation):
    """
    Rotates the VECTORS by angle ORIENTATION (measured from y axis clockwise)
    :param vectors:
    :param orientation:
    :return: rotated vectors
    """
    rot_matrix = np.array([
        [ np.cos(orientation), np.sin(orientation)],
        [-np.sin(orientation), np.cos(orientation)]
    ])
    if type(vectors) is tuple:
        return [rot_matrix.dot(vectors[0]), rot_matrix.dot(vectors[1])]
    else:
        return rot_matrix.dot(vectors)