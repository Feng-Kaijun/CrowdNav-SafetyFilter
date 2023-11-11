from __future__ import print_function
import numpy as np
import matplotlib.pyplot as plt
import scipy.spatial
from matplotlib.colors import colorConverter
import mpl_toolkits.mplot3d as a3
from PySafetyFilter.SafetyFilter_wrapper import Polyhedron

def draw2d(self, points, ax=None):
    if ax is None:
        ax = plt.gca()
    hull = scipy.spatial.ConvexHull(points)
    ax.add_patch(plt.Polygon(xy=points[hull.vertices], closed=True))

def draw3d(self, points, ax=None, **kwargs):
    if ax is None:
        ax = a3.Axes3D(plt.gcf())
    kwargs.setdefault("facecolor", 'r')
    kwargs.setdefault("edgecolor", "k")
    kwargs.setdefault("facecolor", "r")
    kwargs.setdefault("alpha", 0.5)
    kwargs["facecolor"] = colorConverter.to_rgba(kwargs["facecolor"], kwargs["alpha"])
    hull = scipy.spatial.ConvexHull(points)
    for simplex in hull.simplices:
        poly = a3.art3d.Poly3DCollection([points[simplex]], **kwargs)
        if "alpha" in kwargs:
            poly.set_alpha(kwargs["alpha"])
        ax.add_collection3d(poly)