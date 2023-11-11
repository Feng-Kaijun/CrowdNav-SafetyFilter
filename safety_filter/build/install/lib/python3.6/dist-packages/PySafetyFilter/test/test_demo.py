from __future__ import absolute_import
import numpy as np
from PySafetyFilter.SafetyFilter_wrapper import Polyhedron, LTIsystem
import matplotlib.pyplot as plt
import scipy.spatial
import mpl_toolkits.mplot3d as a3

def test_Polyhedron():
    fig, ax = plt.subplots(figsize=(8, 8))
    ax.tick_params(labelsize=16)
    ax.set_xlim(-6, 6)
    ax.set_ylim(-6, 6)
    A1 = np.array([[0, 1], [1, 0], [0, -1], [-1, 0]])
    b1 = np.array([1, 1, 1, 1]).reshape(4, 1)
    P1 = Polyhedron(A1, b1)
    points = np.array(P1.generateVertex())
    hull = scipy.spatial.ConvexHull(points)
    ax.add_patch(plt.Polygon(xy=points[hull.vertices], closed=True, alpha=0.1, color='r'))

    A2 = np.array([[0, 1], [1, 0], [0, -1], [-1, 0]])
    b2 = np.array([2, 2, 2, 2]).reshape(4, 1)
    P2 = Polyhedron(A2, b2)
    points = np.array(P2.generateVertex())
    hull = scipy.spatial.ConvexHull(points)
    ax.add_patch(plt.Polygon(xy=points[hull.vertices], closed=True, alpha=0.2, color='b'))

    R = np.array([[np.cos(np.pi / 3), -np.sin(np.pi / 3)], [np.sin(np.pi / 3), np.cos(np.pi / 3)]])
    P3 = Polyhedron(A2, b2)
    # P3.intersect(P2)
    P3.invAffineMap(R)
    points = np.array(P3.generateVertex())
    hull = scipy.spatial.ConvexHull(points)
    ax.add_patch(plt.Polygon(xy=points[hull.vertices], closed=True, alpha=0.3, color='k'))

    point = np.array([1, 0]).reshape(2, 1)
    if (P1.isInside(point, 0)):
        print("inside point")
    else: print("outside point")

    if (P3.isSubset(P2)):
        print("P3 is subset of P1")
    else: print("P3 is not subset of P2")

    A4 = np.array([[1], [-1]])
    b4 = np.array([[1], [1]])
    P4 = Polyhedron(A4, b4)
    points = np.array(P4.generateVertex())
    # ax.add_patch(plt.Polygon(xy=points, closed=True, alpha=0.4, color='y'))

    plt.figure()
    a3d = a3.Axes3D(plt.gcf())
    a3d.set_xlim(-6, 6)
    a3d.set_ylim(-6, 6)
    a3d.set_zlim(-6, 6)
    P5 = Polyhedron(A2, b2)
    P5.appendDimensions(P4)
    points = np.array(P5.generateVertex())
    hull = scipy.spatial.ConvexHull(points)
    for simplex in hull.simplices:
        poly = a3.art3d.Poly3DCollection([points[simplex]], facecolor='r', edgecolor='k', alpha=0.2)
        a3d.add_collection3d(poly)

    P6 = Polyhedron(A2, b2)
    P6.appendDimensions(P6)
    points1 = np.array(P6.generateVertex())
    P6.projection([1, 0, 1, 1])
    points2 = np.array(P6.generateVertex())
    hull = scipy.spatial.ConvexHull(points2)
    for simplex in hull.simplices:
        poly = a3.art3d.Poly3DCollection([points2[simplex]], facecolor='b', edgecolor='k', alpha=0.2)
        a3d.add_collection3d(poly)
    plt.show()

if __name__ == '__main__':
    test_Polyhedron()