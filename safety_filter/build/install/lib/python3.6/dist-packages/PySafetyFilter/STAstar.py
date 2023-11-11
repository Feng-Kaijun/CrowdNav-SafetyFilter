import heapq
import math
import numpy as np

from STAstar_utils import Config


MAX_THETA = np.deg2rad(60.0)  # [rad] maximum steering angle
L = 3.0
N_STEER = 10
MAX_ACC = 5.0
MIN_ACC = -10.0
N_ACC = 15
N_THATA = 20  # number of steer command

dt = T_GRID_RESOLUTION = 0.1
XY_GRID_RESOLUTION = 0.3
YAW_GRID_RESOLUTION = np.deg2rad(15.0)
V_GRID_RESOLUTION = 1.0
MAX_T = 5.0  # plan time
MAX_V = 10.0

N_T = int(MAX_T/T_GRID_RESOLUTION)


class Node:
    def __init__(self, t_ind, x_ind, y_ind, yaw_ind, v_ind,
                 t, x, y, yaw, v, theta=0.0, acc=0.0,
                 parent_index=None, cost=None):
        self.t_index = t_ind
        self.x_index = x_ind
        self.y_index = y_ind
        self.yaw_index = yaw_ind
        self.v_index = v_ind
        self.t = t
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v
        self.theta = theta
        self.acc = acc
        self.parent_index = parent_index
        self.cost = cost


class Path:
    def __init__(self, t_list, x_list, y_list, yaw_list, v_list, cost):
        self.t_list = t_list
        self.x_list = x_list
        self.y_list = y_list
        self.yaw_list = yaw_list
        self.v_list = v_list
        self.cost = cost


def pi_2_pi(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi


def calc_motion_inputs():
    for theta in np.concatenate((np.linspace(-MAX_THETA, MAX_THETA, N_THATA), [0.0])):
        for acc in np.concatenate((np.linspace(MIN_ACC, MAX_ACC, N_ACC), [0.0])):
            yield [theta, acc]


def move(x, y, yaw, v, theta, acc):
    v += acc*T_GRID_RESOLUTION
    yaw += theta
    x += v * math.cos(yaw) * T_GRID_RESOLUTION
    y += v * math.sin(yaw) * T_GRID_RESOLUTION
    return x, y, yaw, v


def verify_index(node, c):
    t_ind, x_ind, y_ind, v_ind = node.t, node.x_index, node.y_index, node.v_index
    if (0 <= x_ind < c.x_w-1) and (0 <= y_ind < c.y_w-1) and (0 <= t_ind < c.t_w-2) and (0 <= v_ind < c.v_w-1):
        return True

    return False


def calc_index(node, c):
    ind = (node.t_index * c.x_w * c.y_w * c.yaw_w * c.v_w +
           node.v_index * c.x_w * c.y_w * c.yaw_w +
           node.yaw_index  * c.x_w * c.y_w +
           node.y_index  * c.x_w +
           node.x_index)

    if ind < 0:
        print("Error(calc_index):", ind)

    return ind


def isNodeFree(node, st_grid_map, c):
    t_ind, x_ind, y_ind = node.t_index, node.x_index, node.y_index
    if (0 <= x_ind < c.x_w-1) and (5 <= y_ind < c.y_w-5) and (0 <= t_ind < c.t_w):
        if st_grid_map[t_ind, x_ind, y_ind] > 0:
            #             if (node.y > 3.0 or node.y < -3.0):
            #                 print(st_grid_map[t_ind, x_ind, y_ind], t_ind, x_ind, y_ind)
            return True
    return False


def get_neighbors(current, config, st_grid_map):
    for theta, acc in calc_motion_inputs():
        node = calc_next_node(current, theta, acc, config)
        if node and verify_index(node, config) and isNodeFree(node, st_grid_map, config):
            yield node


def calc_next_node(current, theta, acc, config):
    x, y, yaw, v = current.x, current.y, current.yaw, current.v
    x, y, yaw, v = move(x, y, yaw, v, theta, acc)
    if np.cos(yaw) < 0.0 or v < 0.0 or v > MAX_V: return None
    x_ind = round(x / XY_GRID_RESOLUTION) - config.min_x
    y_ind = round(y / XY_GRID_RESOLUTION) - config.min_y
    yaw_ind = round(yaw / YAW_GRID_RESOLUTION) - config.min_yaw
    v_ind = round(v / V_GRID_RESOLUTION) - config.min_v

    e = pow((current.x - x)**2+(current.y - y)**2, 0.5)
    c = abs(v - MAX_V)
    d = current.x - x
    cost = 0.65*e + c + 5.0*d
    node = Node(current.t_index+1, x_ind, y_ind, yaw_ind, v_ind,
                current.t+T_GRID_RESOLUTION, x, y, yaw, v, theta=theta, acc=acc,
                parent_index=calc_index(current, config),
                cost=cost)
    return node


def is_same_grid(n1, n2):
    if ((n1.t_index == n2.t_index)
            and (n1.x_index == n2.x_index)
            and (n1.y_index == n2.y_index)
            and (n1.yaw_index == n2.yaw_index)
            and (n1.v_index == n2.v_index)):
        return True
    return False


def culcontrolpoints(p1, p2):
    control_points = np.zeros((2, 4))
    control_points[:,0] = p1[0:2]
    control_points[:,3] = p2[0:2]
    dx = p1[0] - p2[0]
    dy = p1[1] - p2[1]
    d = np.sqrt(dx*dx+dy*dy)/3.0
    control_points[:,1] = np.array([p1[0]+d*np.cos(p1[2]), p1[1]+d*np.sin(p1[2])])
    control_points[:,2] = np.array([p2[0]-d*np.cos(p2[2]), p2[1]-d*np.sin(p2[2])])
    return control_points


def recursive(control_points, T, B0=0, B1=0, dB0=0, dB1=0, ddB0=0, ddB1=0):
    if len(control_points) == 1:
        return control_points[0], -B0+(1-T)*dB0+B1+T*dB1, -2*dB0+(1-T)*ddB0+2*dB1+T*ddB1
    else:
        B0, dB0, ddB0 = recursive(control_points[0:-1], T)
        B1, dB1, ddB1 = recursive(control_points[1:], T)
        return (1-T)*B0 + T*B1 , -B0+(1-T)*dB0+B1+T*dB1, -2*dB0+(1-T)*ddB0+2*dB1+T*ddB1


def cullengthofcurve(x, y):
    s = 0
    for i in range(len(x)-1):
        dx = x[i+1] - x[i]
        dy = y[i+1] - y[i]
        s += np.sqrt(dx*dx+dy*dy)
    return s


def calc_cost(n, goal_node, c):
    h = abs(n.y-goal_node.y)
    return n.cost + h


def get_final_path(closed, goal_node):
    reversed_t = []
    reversed_x = []
    reversed_y = []
    reversed_yaw = []
    reversed_v = []

    nid = goal_node.parent_index
    final_cost = goal_node.cost
    while nid:
        n = closed[nid]
        reversed_t.append(n.t)
        reversed_x.append(n.x)
        reversed_y.append(n.y)
        reversed_yaw.append(n.yaw)
        reversed_v.append(n.v)
        nid = n.parent_index
    reversed_t = list(reversed(reversed_t))
    reversed_x = list(reversed(reversed_x))
    reversed_y = list(reversed(reversed_y))
    reversed_yaw = list(reversed(reversed_yaw))
    reversed_v = list(reversed(reversed_v))

    path = Path(reversed_t, reversed_x, reversed_y, reversed_yaw, reversed_v, final_cost)

    return path


def STAStar_planning(start, goal, st_grid_map, xy_resolution, yaw_resolution):
    start[2], goal[2] = pi_2_pi(start[2]), pi_2_pi(goal[2])
    config = Config(0.0, 50.0, -4.5, 4.5, xy_resolution, yaw_resolution)
    start_node = Node(0, round(start[0] / xy_resolution) - config.min_x,
                      round(start[1] / xy_resolution) - config.min_y,
                      round(start[2] / yaw_resolution) - config.min_yaw,
                      round(start[3] / V_GRID_RESOLUTION) - config.min_v,
                      0.0, start[0], start[1], start[2], start[3], cost=0.0)
    goal_node = Node(config.t_w-1, round(goal[0] / xy_resolution) - config.min_x,
                     round(goal[1] / xy_resolution) - config.min_y,
                     round(goal[2] / yaw_resolution) - config.min_yaw,
                     round(goal[3] / V_GRID_RESOLUTION) - config.min_v,
                     MAX_T, goal[0], goal[1], goal[2], goal[3], cost=0.0)
    openList, closedList = {}, {}
    pq = []
    openList[calc_index(start_node, config)] = start_node
    heapq.heappush(pq, (calc_cost(start_node, goal_node, config),
                        calc_index(start_node, config)))
    final_path = None
    ite = 0
    while ite < 50000:
        if not openList:
            print("Error: Cannot find path, No open set")
            return [], [], []

        cost, c_id = heapq.heappop(pq)
        if c_id in openList:
            current = openList.pop(c_id)
            closedList[c_id] = current
        else: continue

        if  ((current.t_index == goal_node.t_index)
                and (current.y_index == goal_node.y_index)):
            print("path found")
            goal_node.t_index = current.t_index
            goal_node.t = current.t
            goal_node.cost = current.cost
            goal_node.parent_index = calc_index(current, config)
            break
        for neighbor in get_neighbors(current, config, st_grid_map):
            neighbor_index = calc_index(neighbor, config)
            if neighbor_index in closedList.keys():
                continue
            if (neighbor_index not in openList.keys()) or (openList[neighbor_index].cost > neighbor.cost):
                heapq.heappush(pq, (calc_cost(neighbor, goal_node, config), neighbor_index))
                openList[neighbor_index] = neighbor
        ite += 1

    path = get_final_path(closedList, goal_node)
    return path