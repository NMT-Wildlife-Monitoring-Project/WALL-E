"""Pure-numpy scoring for D2OC exploration (matches d2oc-speedup.py brain).

score = clip(uncertainty(p) - visited_freq, 0, 1) / log(gamma*d + offset)
"""
import numpy as np

GAMMA_DEFAULT = 0.1
OFFSET_DEFAULT = 1.5


def uncertainty(prob):
    """Triangular uncertainty: 1 - 2*|p - 0.5|, clipped to [0, 1].

    Peaks at p=0.5 (unknown) = 1.0; 0.0 at fully free/occupied.
    """
    p = np.asarray(prob, dtype=np.float64)
    return np.clip(1.0 - 2.0 * np.abs(p - 0.5), 0.0, 1.0)


def travel_cost(distance, gamma=GAMMA_DEFAULT, offset=OFFSET_DEFAULT):
    """Logarithmic distance penalty: log(gamma*d + offset).

    offset > 1 keeps the cost positive and finite at d=0.
    """
    d = np.asarray(distance, dtype=np.float64)
    return np.log(gamma * d + offset)


def information_score(prob, visited_freq, distance, gamma=GAMMA_DEFAULT, offset=OFFSET_DEFAULT):
    """Information gain per unit travel cost.

    error = clip(uncertainty(p) - visited_freq, 0, 1)
    score = error / travel_cost(distance)
    """
    error = np.clip(uncertainty(prob) - np.asarray(visited_freq, dtype=np.float64), 0.0, 1.0)
    cost = travel_cost(distance, gamma=gamma, offset=offset)
    return error / (cost + 1e-9)
