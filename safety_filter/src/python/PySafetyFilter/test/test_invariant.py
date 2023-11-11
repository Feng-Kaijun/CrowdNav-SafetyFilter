from __future__ import absolute_import
import numpy as np
from PySafetyFilter.SafetyFilter_wrapper import Polyhedron, LTIsystem
import matplotlib.pyplot as plt
import scipy.spatial
import mpl_toolkits.mplot3d as a3

def get_ReachableSet(domain, X0, U) -> Polyhedron:
    X0.appendDimensions(U)
    T = np.array([[1, 0, -0.25, 0.866, -0.125],
                  [0, 1, 0.433, 0.5, 0.2165],
                  [0, 0, 1, 0, 0],
                  [0, 0, 0, 1, 0],
                  [0, 0, 0, 0, 1]])
    X0.invAffineMap(T)
    proj = [1, 1, 1, 0, 0]
    X0.projection(proj)
    X0.intersect(domain)
    return X0


def get_invariantSet(max_iter) -> Polyhedron:
    converged = False
    stateA = np.array([[-0.8974, -0.4412, 0],
                       [-0.8128, -0.5826, 0],
                       [0.9080,  0.4190, 0],
                       [-0.4046, -0.9145, 0],
                       [0,  0.0995, 0],
                       [0,       0, 1],
                       [0,       0,-1]])
    stateB = np.array([2.6886, 2.3710, 0.7290, 2.8745, 0.9950, 3.1416, 3.1416]).reshape(7, 1)
    X0 = Polyhedron(stateA, stateB)
    domain = Polyhedron(stateA, stateB)
    inputA = np.array([[0, -0.7071],
                       [0.7071, 0],
                       [0, 0.7071],
                       [-0.7071, 0]])
    inputB = np.array([0.7071, 0.7071, 0.7071, 0.7071]).reshape(4, 1)
    U = Polyhedron(inputA, inputB)

    for i in range(max_iter):
        if converged:
            break
        X = get_ReachableSet(domain, X0, U)
        X.intersect(X0)
        if X.isEqual(X0):
            converged = True
            print("Compute invariant set need ", i + 1, " iterations!")
        else:
            X0 = X
    return X

if __name__ == '__main__':
    safeset = get_invariantSet(100)
