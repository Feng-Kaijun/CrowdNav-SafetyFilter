import numpy as np
import math
import copy
import time

dt = T_GRID_RESOLUTION = 0.1
XY_GRID_RESOLUTION = 0.3
YAW_GRID_RESOLUTION = np.deg2rad(15.0)
V_GRID_RESOLUTION = 1.0
MAX_T = 5.0  # plan time
MAX_V = 10.0


class Config:
    def __init__(self, min_x_m, max_x_m, min_y_m, max_y_m,
                 xy_resolution, yaw_resolution):
        self.min_t = 0
        self.max_t = round(MAX_T / T_GRID_RESOLUTION)
        self.t_w = round(self.max_t - self.min_t) + 1

        self.min_x = round(min_x_m / xy_resolution)
        self.min_y = round(min_y_m / xy_resolution)
        self.max_x = round(max_x_m / xy_resolution)
        self.max_y = round(max_y_m / xy_resolution)

        self.x_w = round(self.max_x - self.min_x) + 1
        self.y_w = round(self.max_y - self.min_y) + 1

        self.min_yaw = round(- math.pi / yaw_resolution)
        self.max_yaw = round(math.pi / yaw_resolution)
        self.yaw_w = round(self.max_yaw - self.min_yaw) + 1

        self.min_v = 0
        self.max_v = round(MAX_V / V_GRID_RESOLUTION)
        self.v_w = round(self.max_v - self.min_v) + 1


class Point:
    def __init__(self, x, y, phi):
        self.x = x
        self.y = y
        self.phi = phi


class Box2D(object):
    def __init__(self, x0, y0, phi0, v0, L, W, lanetype):
        self.lanetype = lanetype
        self.x = x0
        self.y = y0
        self.phi = phi0
        self.v = v0
        self.L = L
        self.W = W
        self.traj = [Point(x0, y0, phi0)]
        self.get_traj()

    def get_traj(self):
        for i in range(1, config.t_w):
            x = self.x + self.v*i*dt*np.cos(self.phi)
            y = self.y + self.v*i*dt*np.sin(self.phi)
            self.traj.append(Point(x, y, self.phi))


def get_test_points(obs, point):
    num_l, num_w = 100, 100
    point_buf = np.zeros((num_l*num_w, 2))
    for i, l in enumerate(np.linspace(-(obs.L+egoL)/2.0, (obs.L+egoL)/2.0, num_l)):
        for j, w in enumerate(np.linspace(-(obs.W+egoW)/2.0, (obs.W+egoW)/2.0, num_w)):
            point_buf[i*num_l+j] = np.array([point.x+l*np.cos(point.phi)+w*np.cos(point.phi-(np.pi/2.0)),
                                             point.y+l*np.sin(point.phi)+w*np.sin(point.phi-(np.pi/2.0))])

    return point_buf.T


def XY2mapIndex2(point, c):
    y_index = np.round(point[1]/XY_GRID_RESOLUTION) - c.min_y
    x_index = np.round(point[0]/XY_GRID_RESOLUTION) - c.min_x
    if len(point[0]) == 1: return int(x_index[0]), int(y_index[0])
    y_index = np.where((y_index >= 0)&(y_index < c.y_w), y_index, -1)
    x_index = np.where((x_index >= 0)&(x_index < c.x_w), x_index, -1)
    x_dels = np.argwhere(x_index == -1)
    y_dels = np.argwhere(y_index == -1)
    af_y_index = np.array([y_index[i] for i in range(len(y_index))  if y_index[i] != -1 and x_index[i] != -1])
    af_x_index = np.array([x_index[i] for i in range(len(x_index))  if y_index[i] != -1 and x_index[i] != -1])
    return [af_x_index.astype(int), af_y_index.astype(int)]


config = Config(0.0, 50.0, -4.5, 4.5, XY_GRID_RESOLUTION, YAW_GRID_RESOLUTION)
obs0 = Box2D(12.0, -1.5, 0.0, 0.4, 1.0, 1.5, "M")
egoL = 2.0
egoW = 1.0
obs_buf = [obs0]
map_grid = np.full((config.t_w, config.x_w, config.y_w), int(10), dtype=np.int8)
map_grid[:,:, 0:8] = 0
map_grid[:,:, config.y_w-8:] = 0

for obs in obs_buf:
    for t, test_point in enumerate(obs.traj):
        points = get_test_points(obs, test_point)
        x_index, y_index = XY2mapIndex2(points, config)
        map_grid[t, x_index, y_index] = 0

import matplotlib
import matplotlib.pyplot as plt
# set up matplotlib
is_ipython = 'inline' in matplotlib.get_backend()
if is_ipython:
    from IPython import display
plt.ion()
plt.figure(figsize=(8, 100), dpi=150)
for t in range(int(MAX_T/T_GRID_RESOLUTION)):
    plt.clf()
    plt.imshow(map_grid[t].T, cmap=plt.cm.hot, interpolation='nearest', vmin=0, vmax=10)
    # plt.colorbar()
    plt.xlim(-1, config.x_w)  # 设置x轴范围
    plt.ylim(-1, config.y_w)  # 设置y轴范围
    my_x_ticks = np.arange(0, config.x_w, 1)
    my_y_ticks = np.arange(0, config.y_w, 1)
    plt.xticks([])
    plt.yticks([])
    plt.grid(True)
    if is_ipython:
        display.clear_output(wait=True)
        display.display(plt.gcf())
