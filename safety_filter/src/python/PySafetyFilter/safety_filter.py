import numpy as np
import itertools
import copy

from crowd_sim.envs.utils.state import FullState
from crowd_sim.envs.utils.state import ObservableState
from crowd_sim.envs.utils.action import ActionRot, ActionXY
import math
import time
from PySafetyFilter.SafetyFilter_wrapper import Safety_Filter
from PySafetyFilter.SafetyFilter_wrapper import Polyhedron


class SafetyFilter(object):
    def __init__(self, lb, ub, K, phi, system, T, speed_samples, rotation_samples):
        self.np_action_space = None
        self.free_action_space = None
        self.sample_action_space = None
        self.speed_samples = speed_samples
        self.rotation_samples = rotation_samples
        self.lb = lb
        self.ub = ub
        self.K = K
        self.phi = phi
        self.system = system
        self.system_A = None
        self.system_B = None
        self.T = T
        self.test_one = True
        self.output = {
                        'safe_action': None,
                        'unsafe_action': None,
                        'iris_vertexs': None,
                        'iris_poly': None,
                        'obstacle_hullPoints': None,
                        'safe_set': None,
                        'safe_set_poly': None,
                        'local_goal': None,
                        'vo_vertex': None,
                        'path': None,
                        'time': None
                      }

    def compuate_safe_set(self, self_state:FullState, human_states):
        t1 = time.time()
        irisRegion = self.state_constrain(self_state, human_states)
        systemA, systemB = self.system_matrix(self_state)
        self.system_A, self.system_B = systemA, systemB
        SF = Safety_Filter(systemA, systemB)
        SF.set_state_constrains(irisRegion)
        SF.set_input_constrains(self.input_constrain())
        SF.set_rho(0)

        safe_set = SF.get_InvariantSet(100)

        safe_set = safe_set.minHRep()
        self.output['safe_set_poly'] = safe_set

        if self.np_action_space is None:
            self.trans_action_space(self_state.v_pref)

        self.build_free_action_space(self_state, human_states, 10, 40)

        if self.system == "holonomic":
            if self.test_one == True:
                points = np.array(safe_set.getVertices())
                self.output['safe_set'] = points
        elif self.system == "unicycle":
            # # add constrain : theta = self_state.theta
            theta_consA = np.array([[0, 0, 1], [0, 0, -1]])
            theta_consb = np.array([[self_state.theta], [-self_state.theta]])
            safe_set = safe_set.addConstraints(theta_consA, theta_consb)
            safe_set_proj = safe_set.projection(1)
            if self.test_one == True:
                points = np.array(safe_set_proj.getVertices())
                self.output['safe_set'] = points

        planning_path = self.get_STAstar_goal(self_state, human_states, safe_set, 1)
        self.output['local_goal'] = planning_path[0]
        self.output['path'] = planning_path

        t2 = time.time()
        self.output['time'] = round((t2 - t1) * 1000)
        return self.output


    def build_sample_action_space(self, v_pref):
        # data/221129
        holonomic = True if self.system == 'holonomic' else False
        speed = v_pref / 2
        if holonomic:
            rotations = np.linspace(0, 2 * np.pi, 8, endpoint=False)
        else:
            rotations = np.linspace(-np.pi / 4, np.pi / 4, 8)
            rotations[-1] = np.pi / 4
            rotations /= self.T

        action_space = []
        for rotation in rotations:
            if holonomic:
                action_space.append(np.array([[speed * np.cos(rotation)], [speed * np.sin(rotation)]]))
            else:
                action_space.append(np.array([[speed], [rotation]]))
        self.sample_action_space = action_space


    def search_sample_path(self, N_P_states, R_state, sample_action, cur_Depth, Depth, cost):
        if Depth + 1 == cur_Depth:
            if cost < self.min_cost:
                self.min_cost = copy.deepcopy(cost)
                self.optimal_path = copy.deepcopy(self.path)
            return

        # Depth=4, Nsteps=10
        # if cur_Depth == 1:
        #     sta_index = 0
        #     end_index = 0
        # elif cur_Depth == 2:
        #     sta_index = 1
        #     end_index = 2
        # elif cur_Depth == 3:
        #     sta_index = 3
        #     end_index = 5
        # elif cur_Depth == 4:
        #     sta_index = 6
        #     end_index = 9

        # Depth=4, Nsteps=40
        if cur_Depth == 1:
            sta_index = 0
            end_index = 9
        elif cur_Depth == 2:
            sta_index = 10
            end_index = 19
        elif cur_Depth == 3:
            sta_index = 20
            end_index = 29
        elif cur_Depth == 4:
            sta_index = 30
            end_index = 39

        cur_cost = 0
        for i in range(end_index - sta_index + 1):
            if self.system == "holonomic":
                cur_R_state = [0, 0]
                cur_R_state[0] = R_state[0] + sample_action[0][0] * self.T * (i + 1)
                cur_R_state[1] = R_state[1] + sample_action[1][0] * self.T * (i + 1)
            elif self.system == "unicycle":
                cur_R_state = [0, 0, 0]
                cur_R_state[2] = R_state[2] + sample_action[1][0] * self.T * (i + 1)
                cur_R_state[0] = R_state[0] + math.cos(cur_R_state[2]) * sample_action[0][0] * self.T * (i + 1)
                cur_R_state[1] = R_state[1] + math.sin(cur_R_state[2]) * sample_action[0][0] * self.T * (i + 1)

        cur_cost += self.compute_stage_cost(cur_R_state, N_P_states[i], self.goal)


        for action in self.sample_action_space:
            self.path.append(cur_R_state)
            self.search_sample_path(N_P_states, cur_R_state, action, cur_Depth=cur_Depth+1, Depth=Depth, cost=cost+cur_cost)
            self.path.pop()

        return


    def get_searchtree_goal(self, self_state:FullState, human_states, safe_set:Polyhedron, Depth=4, Nsteps=10):
        N_P_states = []
        for i in range(1, Nsteps):
            P_states = []
            for human_state in human_states:
                Px = human_state.px + human_state.vx * self.T * i
                Py = human_state.py + human_state.vy * self.T * i
                P_states.append([Px, Py, human_state.vx, human_state.vy])
            N_P_states.append(P_states)

        if self.system == "holonomic":
            R_state = [self_state.px, self_state.py]
        elif self.system == "unicycle":
            R_state = [self_state.px, self_state.py, self_state.theta]


        if self.sample_action_space is None:
            self.build_sample_action_space(self_state.v_pref)

        self.optimal_path = []
        self.path = []
        self.min_cost = float('inf')
        self.goal = [self_state.gx, self_state.gy]
        for action in self.sample_action_space:
            self.search_sample_path(N_P_states, R_state, action, cur_Depth=1, Depth=Depth, cost=0)

        ref_path = []
        for i in range(len(self.optimal_path)):
            if self.system == "holonomic":
                x = self.optimal_path[i][0]
                y = self.optimal_path[i][1]
                ref_path.append(np.array([[x], [y]]))
            elif self.system == "unicycle":
                x = self.optimal_path[i][0]
                y = self.optimal_path[i][1]
                theta = self.optimal_path[i][2]
                ref_path.append(np.array([[x], [y], [theta]]))
            if math.sqrt((x - self.goal[0]) ** 2 + (y - self.goal[1]) ** 2) < 1e-3:
                break

        return ref_path


    def get_STAstar_goal(self, self_state:FullState, human_states, safe_set:Polyhedron, Nsteps):
        planning_path = []

        if self.system == 'holonomic':
            R_state = np.array([[self_state.px], [self_state.py]])
        else:
            R_state = np.array([[self_state.px], [self_state.py], [self_state.theta]])
        goal = np.array([[self_state.gx], [self_state.gy]])

        for i in range(1, Nsteps + 1):
            P_states = []
            max_P_radius = 0
            for human_state in human_states:
                if human_state.radius > max_P_radius:
                    max_P_radius = human_state.radius
                Px = human_state.px + human_state.vx * self.T * i
                Py = human_state.py + human_state.vy * self.T * i
                P_states.append(np.array([[Px], [Py]]))

            min_CH_val = float('inf')
            optimal_next_state = R_state
            for action in self.np_action_space:
                if self.system == 'holonomic':
                    next_px = R_state[0][0] + action[0][0] * self.T
                    next_py = R_state[1][0] + action[1][0] * self.T
                    next_R_state = np.array([[next_px], [next_py]])
                else:
                    next_theta = R_state[2][0] + action[1][0] * self.T
                    next_px = R_state[0][0] + action[0][0] * np.cos(next_theta) * self.T
                    next_py = R_state[1][0] + action[0][0] * np.sin(next_theta) * self.T
                    next_R_state = np.array([[next_px], [next_py], [next_theta]])

                if i <= 1:
                    if safe_set.isContain(next_R_state) == False:
                        continue

                if np.linalg.norm(goal - np.array([[next_R_state[0][0]], [next_R_state[1][0]]])) < 1e-4:
                    planning_path.append(goal)
                    return planning_path

                CH_val = self.compute_cost_and_heuristic_value(R_state, next_R_state, goal, P_states, max_P_radius + self_state.radius + 3)
                if CH_val < min_CH_val:
                    min_CH_val = CH_val
                    optimal_next_state = next_R_state

            planning_path.append(optimal_next_state)
            R_state = optimal_next_state

        return planning_path

    def compute_cost_and_heuristic_value(self, R_state, next_R_state, goal, P_states, obstacle_radius):
        heuristic = np.linalg.norm(np.array([[next_R_state[0][0]], [next_R_state[1][0]]]) - goal)
        effciency = np.linalg.norm(R_state - next_R_state)

        safe = 0
        Krep = 40
        for human_state in P_states:
            D = np.linalg.norm(np.array([[next_R_state[0][0]], [next_R_state[1][0]]]) - human_state)
            if D <= obstacle_radius:
                safe += 0.5 * Krep * ((1 / D - 1 / obstacle_radius) ** 2)

        result = 0.1 * heuristic + 0 * effciency + 1 * safe
        return result

    def build_free_action_space(self, self_state:FullState, human_states, t_lim, steps):
        delta_t = t_lim / steps
        free_action_space = []
        for j, action in enumerate(self.np_action_space):
            free = True
            for i in range(steps):
                t = i * delta_t
                if self.system == "holonomic":
                    Rx = self_state.px + t * action[0][0]
                    Ry = self_state.py + t * action[1][0]
                elif self.system == "unicycle":
                    if action[1][0] == 0:
                        Rx = t * action[0][0] * math.cos(self_state.theta) + self_state.px
                        Ry = t * action[0][0] * math.sin(self_state.theta) + self_state.py
                    else:
                        Rx = (action[0][0] / action[1][0]) * math.sin(action[1][0] * t + self_state.theta) \
                             + self_state.px - (action[0][0] / action[1][0]) * math.sin(self_state.theta)
                        Ry = - (action[0][0] / action[1][0]) * math.cos(action[1][0] * t + self_state.theta) \
                             + self_state.py + (action[0][0] / action[1][0]) * math.cos(self_state.theta)

                for human_state in human_states:
                    Px = human_state.px + t * human_state.vx
                    Py = human_state.py + t * human_state.vy
                    r = 0
                    if (Rx - Px) ** 2 + (Ry - Py) ** 2 <= (self_state.radius + human_state.radius + r) ** 2:
                        free = False
                        break

                if free is False:
                    break

            if free is True:
                free_action_space.append(action)

        self.free_action_space = free_action_space


    def get_minpotential_goal(self, self_state:FullState, human_states, safe_set:Polyhedron):
        systemA, systemB = self.system_matrix(self_state)
        if self.system == "holonomic":
            current_state = np.array([[self_state.px], [self_state.py]])
            min_potential_goal = np.array([[100], [100]])
        elif self.system == "unicycle":
            current_state = np.array([[self_state.px], [self_state.py], [self_state.theta]])
            min_potential_goal = np.array([[100], [100], [-1]])

        if self.np_action_space is None:
            self.trans_action_space(self_state.v_pref)

        min_potential_value = float('inf')

        for action in self.np_action_space:
            if action[0] == 0:
                continue
            next_state = systemA.dot(current_state) + systemB.dot(action)
            if safe_set.isContain(np.array([[next_state[0][0]], [next_state[1][0]]])) == False:
                continue

            global_goal = np.array([[self_state.gx], [self_state.gy]])
            potential_value = self.get_potential_value(next_state, global_goal, human_states, Katt=1,
                                                       Krep=1, Radd=self_state.radius*3)
            if potential_value < min_potential_value:
                min_potential_value = potential_value
                min_potential_goal = next_state

        if self.system == "holonomic":
            if (min_potential_goal == np.array([[100], [100]])).all():
                return current_state
        elif self.system == "unicycle":
            if (min_potential_goal == np.array([[100], [100], [-1]])).all():
                return current_state

        return min_potential_goal

    def get_potential_value(self, next_state, global_goal, human_states, Katt, Krep, Radd):
        if np.size(next_state) == 3:
            next_state = next_state[0:2, :]
        Uatt = 0.5 * Katt * (np.linalg.norm(next_state - global_goal) ** 2)

        Urep = []
        for human_state in human_states:
            Xobs = np.array([[human_state.px], [human_state.py]])
            R = human_state.radius + Radd
            D = np.linalg.norm(next_state - Xobs)
            if D > R:
                Urep.append(0)
            else:
                Urep.append(0.5 * Krep * ((1 / D - 1 / R) ** 2))

            # Urep.append(-0.5 * Krep * (D ** 2))

        U = Uatt
        for value in Urep:
            U += value
        return U

    def get_loacl_goal(self, self_state:FullState, human_states, safe_set:Polyhedron):
        VO_R = 0
        Rx = self_state.px
        Ry = self_state.py
        Rcenter = np.array([Rx, Ry])
        VRobot = np.array([self_state.vx, 0]) + np.array([0, self_state.vy])
        # now the original goal is outside the safe set
        # find sector as obstacle, [[start_point[x, y], end_point[x, y]], ...]
        obstacle_sectors = []
        self.output['vo_vertex'] = []
        for i, human_state in enumerate(human_states):
            Hcenter = np.array([human_state.px, human_state.py])
            VHuman = np.array([human_state.vx, 0]) + np.array([0, human_state.vy])
            V = VRobot - VHuman
            Vtheta = self.direction_twoPoints(np.array([0, 0]), V)
            P1, P2 = self.tangent_point_PointCircle(Rcenter, Hcenter, human_state.radius + self_state.radius + VO_R)

            direction_s = self.direction_twoPoints(Rcenter, P1)
            direction_e = self.direction_twoPoints(Rcenter, P2)
            Htheta = self.direction_twoPoints(Rcenter, Hcenter)
            if math.fabs(direction_s - Htheta) < math.pi / 2 and math.fabs(direction_e - Htheta) < math.pi / 2:
                if direction_s > direction_e:
                    direction_s, direction_e = direction_e, direction_s
            elif math.fabs(direction_s - Htheta) >= math.pi / 2 or math.fabs(direction_e - Htheta) >= math.pi / 2:
                if direction_s < direction_e:
                    direction_s, direction_e = direction_e, direction_s
            # vo
            if Vtheta < direction_s or Vtheta > direction_e:
                continue

            obstacle_sectors.append([direction_s, i, direction_e, i])
            self.output['vo_vertex'].append(np.array([P1, P2, Rcenter]))
        obstacle_sectors = self.merge_sector(obstacle_sectors)

        # if original goal is inside the safe set, just return the original goal as local goal
        original_goal = np.array([[self_state.gx], [self_state.gy]])
        if safe_set.isContain(original_goal):
            if self.system == "holonomic":
                return original_goal
            elif self.system == "unicycle":
                gtheta = self.direction_twoPoints(Rcenter, np.array([original_goal[0][0], original_goal[1][0]]))
                return np.array([original_goal[0][0], original_goal[1][0], gtheta])

        if len(obstacle_sectors) == 0:
            # no obstacle, make the local goal in a line(from robot state to original goal),
            # moreover make the local goal on the edge of safe set
            # For convenience, directly return the original goal as local goal
            if self.system == "holonomic":
                return original_goal
            elif self.system == "unicycle":
                gtheta = self.direction_twoPoints(Rcenter, np.array([original_goal[0][0], original_goal[1][0]]))
                return np.array([original_goal[0][0], original_goal[1][0], gtheta])
        else:
            # change obstacle_sector to free_sector
            free_sectors = []
            for i in range(len(obstacle_sectors)):
                if i == len(obstacle_sectors) - 1:
                    s = obstacle_sectors[i][2]
                    s_index = obstacle_sectors[i][3]
                    e = obstacle_sectors[0][0]
                    e_index = obstacle_sectors[0][1]
                    free_sectors.append([s, s_index, e, e_index])
                else:
                    s = obstacle_sectors[i][2]
                    s_index = obstacle_sectors[i][3]
                    e = obstacle_sectors[i + 1][0]
                    e_index = obstacle_sectors[i + 1][1]
                    free_sectors.append([s, s_index, e, e_index])

            # select one sector in free_sectors
            heading = self.select_heading(free_sectors, self_state, human_states, math.pi / 5)
            if heading == -1:
                return np.array([0, 0, heading])
            L = 1
            XY = Rcenter + np.array([math.cos(heading), math.sin(heading)]) * L
            return np.array([XY[0], XY[1], heading])
            # # find which line the local_goal site on
            # points = safe_set.getVertices()
            # vertice_theta = []
            # for i in range(len(points)):
            #     theta = self.direction_twoPoints(Rcenter, points[i])
            #     vertice_theta.append([points[i][0], points[i][1], theta])
            # vertice_theta.sort(key=lambda x: x[2])
            # first_vertex = np.array([100, 100])
            # second_vertex = np.array([100, 100])
            # for i in range(len(vertice_theta)):
            #     if heading == vertice_theta[i][2]:
            #         return np.array([vertice_theta[0], vertice_theta[1], heading])
            #     elif heading > vertice_theta[i][2]:
            #         first_vertex = np.array([vertice_theta[i][0], vertice_theta[i][1]])
            #     else:
            #         second_vertex = np.array([vertice_theta[i][0], vertice_theta[i][1]])
            #         break
            # if (first_vertex == np.array([100, 100])).all():
            #     first_vertex = np.array([vertice_theta[-1][0], vertice_theta[-1][1]])
            # if (second_vertex == np.array([100, 100])).all():
            #     second_vertex = np.array([vertice_theta[0][0], vertice_theta[0][1]])
            # goal = self.line_intersect(Rcenter, Rcenter + np.array([math.cos(heading), math.sin(heading)]),
            #                            first_vertex, second_vertex)
            # return np.array([goal[0], goal[1], heading])

    def line_intersect(self, L1P1, L1P2, L2P1, L2P2):
        x11 = L1P1[0]
        y11 = L1P1[1]
        x12 = L1P2[0]
        y12 = L1P2[1]

        x21 = L2P1[0]
        y21 = L2P1[1]
        x22 = L2P2[0]
        y22 = L2P2[1]

        k1 = math.tan(self.direction_twoPoints(L1P1, L1P2))
        b1 = y11 - x11 * k1

        k2 = math.tan(self.direction_twoPoints(L2P1, L2P2))
        b2 = y21 - x21 * k2

        cx = (b2 - b1) / (k1 - k2)
        cy = k1 * cx + b1
        return np.array([cx, cy])

    def select_heading(self, free_sectors, self_state, human_states, delta):
        Rcenter = np.array([self_state.px, self_state.py])
        goal = np.array([self_state.gx, self_state.gy])
        goal_heading = self.direction_twoPoints(Rcenter, goal)
        min_heading = float('inf')
        heading_point = np.array([100, 100])
        for i in range(len(free_sectors)):
            s = free_sectors[i][0]
            e = free_sectors[i][2]
            # filter some too small sectors, in order to make the smallest distance which robot can pass
            if e >= s:
                s += delta / 2
                e -= delta / 2
                if e - s <= 0:
                    continue
            else:
                A = (math.pi * 2 - s) + e
                if A <= delta:
                    continue
                s += delta / 2
                if s >= math.pi * 2:
                    s -= math.pi * 2
                e -= delta / 2
                if e <= 0:
                    e += math.pi * 2

            # P1 = human_states[free_sectors[i][3]]
            # P2 = human_states[free_sectors[i][1]]
            # b = self.distance_twoPoints(Rcenter, np.array([P1.px, P1.py]))
            # c = self.distance_twoPoints(Rcenter, np.array([P2.px, P2.py]))
            # a = math.sqrt(b ** 2 + c ** 2 - 2 * b * c * math.cos(A))
            # if a <= 2 * (self_state.radius + r):
            #     continue
            # if goal is between s and e, return goal_heading
            if e >= s:
                if s <= goal_heading and goal_heading <= e:
                    return goal_heading
            else:
                if goal_heading <= math.pi and e >= goal_heading:
                    return goal_heading
                elif goal_heading > math.pi and goal_heading >= s:
                    return goal_heading
            # compute a smallest heading in this free_sector; the smallest heading is from robot to goal
            v1 = Rcenter + np.array([np.cos(s), np.sin(s)])
            heading_1 = self.get_vector_angle(Rcenter, goal, v1)
            v2 = Rcenter + np.array([np.cos(e), np.sin(e)])
            heading_2 = self.get_vector_angle(Rcenter, goal, v2)
            if heading_1 < heading_2:
                if heading_1 < min_heading:
                    min_heading = heading_1
                    heading_point = v1
            else:
                if heading_2 < min_heading:
                    min_heading = heading_2
                    heading_point = v2
        if (heading_point == np.array([100, 100])).all():
            return -1
        return self.direction_twoPoints(Rcenter, heading_point)

    def get_triangle_delta(self, Rtheta, s, e):
        angle_1 = Rtheta - s
        if angle_1 < 0:
            angle_1 += math.pi
        angle_2 = Rtheta - e
        if angle_2 < 0:
            angle_2 += math.pi
        return min(angle_1, angle_2)

    def compuate_safe_action(self, self_state:FullState, current_action, human_states):
        systemA, systemB = self.system_matrix(self_state)
        input_poly = self.input_constrain()
        inputA = input_poly.getA()
        inputB = input_poly.getB().reshape(inputA.shape[0], 1)
        safe_set = self.output['safe_set_poly']
        safe_setA = safe_set.getA()
        safe_setB = safe_set.getB().reshape(safe_setA.shape[0], 1)

        if self.system == "holonomic":
            current_state = np.array([[self_state.px], [self_state.py]])
            local_goal_state = self.output['local_goal'].reshape(2, 1)
            goal_state = np.array([[self_state.gx], [self_state.gy]])
            current_input = np.array([[current_action.vx], [current_action.vy]])
        elif self.system == "unicycle":
            current_state = np.array([[self_state.px], [self_state.py], [self_state.theta]])
            local_goal_state = self.output['local_goal'].reshape(3, 1)
            goal_state = np.array([[self_state.gx], [self_state.gy], [0]])
            current_input = np.array([[current_action.v], [current_action.r / self.T]])

        A = np.vstack((inputA, np.dot(safe_setA, systemB)))
        Aub = np.vstack((inputB, safe_setB - np.dot(safe_setA, systemA).dot(current_state)))
        # check and find a safe action with higher value
        AF = 1
        GNF = 10
        ANF = 10
        min_object_value = float('inf')
        min_safe_action = None
        if len(self.free_action_space) <= 0:  # gvo is infeasible
            for action in self.np_action_space:
                constrain_result = A.dot(action) - Aub
                if constrain_result.max() <= 0:
                    first = action - current_input
                    next_state = systemA.dot(current_state) + systemB.dot(action)

                    # if self.system == "holonomic":
                    #     second = next_state - goal_state
                    # elif self.system == "unicycle":
                    #     next_state[2] = 0
                    #     second = next_state - goal_state
                    second = next_state - local_goal_state

                    object_value = np.linalg.norm(first) + GNF * np.linalg.norm(second)
                    if object_value <= min_object_value:
                        min_object_value = object_value
                        min_safe_action = action
        else:
            for action in self.free_action_space:  # gvo is feasible
                constrain_result = A.dot(action) - Aub
                if constrain_result.max() <= 0:
                    first = action - current_input
                    next_state = systemA.dot(current_state) + systemB.dot(action)

                    if self.system == "holonomic":
                        second = next_state - goal_state
                    elif self.system == "unicycle":
                        next_state[2] = 0
                        second = next_state - goal_state

                    object_value = np.linalg.norm(first) + AF * np.linalg.norm(second)
                    if object_value <= min_object_value:
                        min_object_value = object_value
                        min_safe_action = action

        # gvo and safe_set are infeasible, or only have zero solution
        # if min_safe_action is None or (min_safe_action == np.array([[0], [0]])).all():
        if min_safe_action is None:
            min_object_value = float('inf')
            for action in self.np_action_space:
                first = action - current_input
                next_state = systemA.dot(current_state) + systemB.dot(action)

                # if self.system == "holonomic":
                #     second = next_state - local_goal_state
                # elif self.system == "unicycle":
                #     next_state[2] = 0
                #     second = next_state - local_goal_state
                second = next_state - local_goal_state

                object_value = 1 * np.linalg.norm(first) + ANF * np.linalg.norm(second)
                if object_value <= min_object_value:
                    min_object_value = object_value
                    min_safe_action = action

        if self.system == "unicycle":
            return ActionRot(float(min_safe_action[0][0]), float(min_safe_action[1][0])*self.T)
        return ActionXY(float(min_safe_action[0][0]), float(min_safe_action[1][0]))

    def get_collision_cost(self, self_state:FullState, next_state, sample_action, human_states, steps):
        N_P_states = []
        for i in range(1, steps + 1):
            P_states = []
            for human_state in human_states:
                Px = human_state.px + human_state.vx * self.T * i
                Py = human_state.py + human_state.vy * self.T * i
                P_states.append([Px, Py, human_state.vx, human_state.vy, human_state.radius])
            N_P_states.append(P_states)

        if self.system == "holonomic":
            R_state = [next_state[0][0], next_state[1][0], sample_action[0][0], sample_action[1][0], self_state.radius]
        elif self.system == "unicycle":
            R_state = [next_state[0][0], next_state[1][0], next_state[2][0], sample_action[0][0], sample_action[1][0], self_state.radius]

        value_init = self.compute_stage_cost(R_state, N_P_states[0])
        value_list = []

        for action in self.np_action_space:
            self.backtrack(R_state, N_P_states, action, value_init, value_list, 2, steps)
        # self.backtrack(R_state, N_P_states, sample_action, value_init, value_list, 2, steps)

        # average_cost = sum(value_list)/len(value_list)
        # return average_cost

        return max(value_list)

    def backtrack(self, R_state, N_P_states, cur_action, cur_value, value_list, cur_step, steps):
        if self.system == "holonomic":
            R_state[0] = R_state[0] + cur_action[0][0] * self.T
            R_state[1] = R_state[1] + cur_action[1][0] * self.T
            R_state[2] = cur_action[0][0]
            R_state[3] = cur_action[1][0]
        elif self.system == "unicycle":
            R_state[2] = R_state[2] + cur_action[1][0] * self.T
            R_state[0] = R_state[0] + math.cos(R_state[2]) * cur_action[0][0] * self.T
            R_state[1] = R_state[1] + math.sin(R_state[2]) * cur_action[0][0] * self.T
            R_state[2] = cur_action[0][0]
            R_state[3] = cur_action[1][0]

        cur_value += self.compute_stage_cost(R_state, N_P_states[cur_step - 1])
        if cur_step == steps:
            value_list.append(cur_value)
            return

        for action in self.np_action_space:
            self.backtrack(R_state, N_P_states, action, cur_value, value_list, cur_step+1, steps)
        # self.backtrack(R_state, N_P_states, cur_action, cur_value, value_list, cur_step+1, steps)

        return


    def compute_stage_cost(self, R_state, P_states, goal, dist=0.0):
        Rx = R_state[0]
        Ry = R_state[1]
        stage_cost = math.sqrt((Rx - goal[0]) ** 2 + (Ry - goal[1]) ** 2)
        for P_state in P_states:
            Px = P_state[0]
            Py = P_state[1]
            stage_cost += 1 / (math.sqrt((Rx - Px) ** 2 + (Ry - Py) ** 2) - dist)
        return stage_cost

    def trans_action_space(self, v_pref):
        """
        Action space consists of 25 uniformly sampled actions in permitted range and 25 randomly sampled actions.
        """
        # data/221129 and data/220812
        holonomic = True if self.system == 'holonomic' else False
        speeds = [(np.exp((i + 1) / self.speed_samples) - 1) / (np.e - 1) * v_pref for i in range(self.speed_samples)]
        if holonomic:
            rotations = np.linspace(0, 2 * np.pi, self.rotation_samples, endpoint=False)
        else:
            rotations = np.linspace(-np.pi / 4, np.pi / 4, self.rotation_samples)
            rotations /= self.T

        action_space = [np.array([[0], [0]])]
        for rotation, speed in itertools.product(rotations, speeds):
            if holonomic:
                action_space.append(np.array([[speed * np.cos(rotation)], [speed * np.sin(rotation)]]))
            else:
                action_space.append(np.array([[speed], [rotation]]))
        self.np_action_space = action_space

        # data/230114
        # holonomic = True if self.system == 'holonomic' else False
        # if holonomic:
        #     speeds = [(np.exp((i + 1) / self.speed_samples) - 1) / (np.e - 1) * v_pref for i in range(self.speed_samples)]
        #     rotations = np.linspace(0, 2 * np.pi, self.rotation_samples, endpoint=False)
        # else:
        #     speeds = [(i / (self.speed_samples -1)) * v_pref for i in range(self.speed_samples)]
        #     rotations = np.linspace(-np.pi / 4, np.pi / 4, self.rotation_samples)
        #     rotations /= self.T
        #
        # action_space = []
        # for rotation, speed in itertools.product(rotations, speeds):
        #     if holonomic:
        #         action_space.append(np.array([[speed * np.cos(rotation)], [speed * np.sin(rotation)]]))
        #     else:
        #         action_space.append(np.array([[speed], [rotation]]))
        #
        # self.np_action_space = action_space

    def all_collion_constrain(self, self_state, human_states):
        # compute the all distance between robot and humans
        result = None
        for human in human_states:
            Hcenter = np.array([human.px, human.py])
            Rcenter = np.array([self_state.px, self_state.py])
            r = 0
            s = self.intersection_point_ClineCircle(Hcenter, human.radius + self_state.radius + r, Rcenter)
            k = - (s[0] - Hcenter[0]) / (s[1] - Hcenter[1])
            b = s[1] - k * s[0]
            if self_state.py - k * self_state.px - b <= 0:
                collion_constrain_A = np.array([[-k, 1]])
                collion_constrain_b = np.array([[b]])
            else:
                collion_constrain_A = np.array([[k, -1]])
                collion_constrain_b = np.array([[-b]])

            tmp = Polyhedron(collion_constrain_A, collion_constrain_b)
            if result is None:
                result = tmp
            else:
                result = result.addConstraints(tmp.getA(), tmp.getB())
        return result

    def cirle_state_constrain(self, self_state:FullState, human_states):
        R = 3
        Rx = self_state.px
        Ry = self_state.py
        Rcenter = np.array([Rx, Ry])
        # sort human_states, according direction from robot to human, 0 -> 2pi
        # human_states = self.sort_human_states(self_state, human_states)

        # find sector as obstacle, [[start_point[x, y], end_point[x, y]], ...]
        obstacle_sectors = []
        for human_state in human_states:
            Hcenter = np.array([human_state.px, human_state.py])
            dist = self.distance_twoPoints(Hcenter, Rcenter)
            if dist - human_state.radius <= R:
                # P1, P2 = self.tangent_point_PointCircle(Rcenter, Hcenter, human_state.radius)
                P1, P2 = self.tangent_point_PointCircle(Rcenter, Hcenter, human_state.radius + 0.3)
                direction_s = self.direction_twoPoints(Rcenter, P1)
                direction_e = self.direction_twoPoints(Rcenter, P2)
                if direction_s > direction_e:
                    direction_s, direction_e = direction_e, direction_s
                obstacle_sectors.append([direction_s, direction_e])
        obstacle_sectors.sort(key=lambda x: x[0])
        obstacle_sectors = self.merge_sector(obstacle_sectors)
        # change obstacle_sector to free_sector, in order to construct state constrain
        if len(obstacle_sectors) == 0:
            # no obstacle, use a regular polygon as state constrain
            n = 4
            d = math.pi * 2 / n
            vertices = np.zeros((n, 2))
            p = np.array([[0], [R]])
            for i in range(n):
                theta = d / 2 + d * i
                rot_mat = np.array([[np.cos(theta), -np.sin(theta)],
                                    [np.sin(theta), np.cos(theta)]])
                vertex = rot_mat.dot(p).T + Rcenter
                vertices[i, :] = vertex
            sector_poly = Polyhedron(vertices)
        else:
            free_sectors = []
            for i in range(len(obstacle_sectors)):
                if i == len(obstacle_sectors) - 1:
                    s = obstacle_sectors[i][1]
                    e = obstacle_sectors[0][0]
                else:
                    s = obstacle_sectors[i][1]
                    e = obstacle_sectors[i + 1][0]
                free_sectors.append([s, e])
            # cut sector if the angle from free_sectors.s to free_sectors.e is too larger
            free_sectors = self.cut_sector(free_sectors)

            # select one sector in free_sectors
            sector = self.select_sector(free_sectors, self_state, R)
            # reshape the sector
            valid_sector_vertex = self.reshape_sector(sector, R, Rcenter)
            sector_poly = Polyhedron(valid_sector_vertex)

        bound_constrain_A = np.array([[1, 0], [-1, 0], [0, 1], [0, -1]])
        bound_constrain_b = np.array([[self.ub], [self.ub], [self.ub], [self.ub]])
        state_constrain = Polyhedron(bound_constrain_A, bound_constrain_b)
        state_constrain = state_constrain.addConstraints(sector_poly.getA(), sector_poly.getB())
        state_constrain = state_constrain.minHRep()
        vertexs = state_constrain.getVertices()
        self.output['iris_vertexs'] = vertexs

        if self.system == "unicycle":
            theta_A = np.array([[1], [-1]])
            theta_b = np.array([[2 * 3.1415926], [0]])
            theta_cons = Polyhedron(theta_A, theta_b)
            state_constrain = state_constrain.addDimensions(theta_cons)

        state_constrain = state_constrain.minHRep()
        self.output['iris_poly'] = state_constrain
        return state_constrain

    def reshape_sector(self, select_sector, R, Rcenter):
        r = 0.3
        s = select_sector[0]
        e = select_sector[1]
        if e >= s:
            alpha = e - s
        else:
            alpha = (math.pi * 2 - s) + e
        # P2 = Rcenter + np.array([R * np.cos(s), R * np.sin(s)])
        # P3 = Rcenter + np.array([R * np.cos(e), R * np.sin(e)])
        L = r / math.sin(alpha / 2)
        DR = np.array([L * math.cos(s + alpha / 2), L * math.sin(s + alpha / 2)])
        D = Rcenter - DR
        v1 = self.intersection_point_AnylineCircle(Rcenter, R, D, s)
        v2 = self.intersection_point_AnylineCircle(Rcenter, R, D, e)
        return np.array([D, v1, v2])

    def cut_sector(self, free_sectors):
        # angle = from free_sectors.s to free_sectors.e
        # pi / 2 < angle <= pi, 2 cut
        # pi < angle <= pi * 3 / 2, 3 cut
        # pi * 3 / 2 < angle <= 2 * pi, 4 cut
        pi = math.pi
        result = []
        for i in range(len(free_sectors)):
            s = free_sectors[i][0]
            e = free_sectors[i][1]
            if e >= s:
                angle = e - s
            else:
                angle = (pi * 2 - s) + e

            # if pi / 2 < angle and angle <= pi:
            #     angle /= 2
            #     result.append([s, s + angle])
            #     result.append([s + angle, e])
            # elif pi < angle and angle <= pi * 3 / 2:
            #     angle /= 3
            #     result.append([s, s + angle])
            #     result.append([s + angle, s + angle * 2])
            #     result.append([s + angle * 2, e])
            # elif pi * 3 / 2 < angle and angle <= pi * 2:
            #     angle /= 4
            #     result.append([s, s + angle])
            #     result.append([s + angle, s + angle * 2])
            #     result.append([s + angle * 2, s + angle * 3])
            #     result.append([s + angle * 3, e])
            # else:
            #     result.append([s, e])

            if pi * 2 / 3 < angle and angle <= pi * 4 / 3:
                angle /= 2
                result.append([s, s + angle])
                result.append([s + angle, e])
            elif pi * 4 / 3 < angle and angle <= pi * 2:
                angle /= 3
                result.append([s, s + angle])
                result.append([s + angle, s + angle * 2])
                result.append([s + angle * 2, e])
            else:
                result.append([s, e])
        return result

    def get_triangle_area(self, R, s, e):
        L = 2 * math.pi * R
        if e >= s:
            l = L * (e - s) / (2 * math.pi)
        else:
            l = L * (2 * math.pi - s + e) / (2 * math.pi)

        return 0.5 * l * R

    def get_triangle_dc(self, A, B, C, goal):
        # first compute inner center of a triangle, then return the distance between inner center and goal
        a = self.distance_twoPoints(B, C)
        b = self.distance_twoPoints(C, A)
        c = self.distance_twoPoints(A, B)
        # (aX1+bX2+cX3)/(a+b+c)
        x = (a * A[0] + b * B[0] + c * C[0]) / (a + b + c)
        # (aY1+bY2+cY3)/(a+b+c)
        y = (a * A[1] + b * B[1] + c * C[1]) / (a + b + c)
        inner_center = np.array([x, y])
        return self.distance_twoPoints(inner_center, goal)

    def get_vector_angle(self, center_point, P1, P2):
        # return: 0 -> pi
        # θ=acos(v1⋅v2/|v1||v2|)
        v1 = P1 - center_point
        v2 = P2 - center_point
        d1 = np.linalg.norm(v1)
        d2 = np.linalg.norm(v2)
        return math.acos(np.dot(v1, v2) / (d1 * d2))

    def get_triangle_delta(self, Rtheta, s, e):
        angle_1 = Rtheta - s
        if angle_1 < 0:
            angle_1 += math.pi
        angle_2 = Rtheta - e
        if angle_2 < 0:
            angle_2 += math.pi
        return min(angle_1, angle_2)

    def normalization(self, data):
        if np.max(data) == np.min(data):
            return data / np.min(data)
        _range = np.max(data) - np.min(data)
        return (data - np.min(data)) / _range

    def select_sector(self, free_sectors, self_state: FullState, R):
        Rcenter = np.array([self_state.px, self_state.py])
        # compute sector area, dc, delta
        area = []
        dc = []
        detal = []
        for sector in free_sectors:
            P2 = Rcenter + np.array([R * np.cos(sector[0]), R * np.sin(sector[0])])
            P3 = Rcenter + np.array([R * np.cos(sector[1]), R * np.sin(sector[1])])
            goal = np.array([self_state.gx, self_state.gy])

            area.append(self.get_triangle_area(R, sector[0], sector[1]))
            dc.append(self.get_triangle_dc(Rcenter, P2, P3, goal))
            detal.append(self.get_triangle_delta(self_state.theta, sector[0], sector[1]))
        area = self.normalization(np.array(area))
        dc = self.normalization(np.array(dc))
        detal = self.normalization(np.array(detal))

        object_value = dc - area
        # object_value = - area
        index = np.argsort(object_value)[0]

        return free_sectors[index]

    def merge_sector(self, sectors_directions):
        if len(sectors_directions) == 0: return sectors_directions
        for i in range(len(sectors_directions)):
            s = sectors_directions[i][0]
            e = sectors_directions[i][2]
            if s > e:
                sectors_directions[i][0] -= math.pi * 2

        sectors_directions.sort(key=lambda x: x[0])

        result = []
        result.append(sectors_directions[0])
        for i in range(1, len(sectors_directions)):
            last = result[-1]
            if last[2] >= sectors_directions[i][0]:
                if last[2] >= sectors_directions[i][2]:
                    continue
                else:
                    result[-1] = [last[0], last[1], sectors_directions[i][2], sectors_directions[i][3]]
            else:
                result.append(sectors_directions[i])

        # if result[-1][2] - result[0][0] <= math.pi:
        #     if result[0][0] < 0:
        #         result[0][0] += math.pi * 2
        #     if result[-1][2] < 0:
        #         result[-1][2] += math.pi * 2
        #     return [[result[0][0], result[0][1], result[-1][2], result[-1][3]]]

        for i in range(len(result)):
            s = result[i][0]
            if s < 0:
                result[i][0] += math.pi * 2

        if result[-1][0] > math.pi * 3 / 2 and result[-1][0] < math.pi * 2 and result[-1][2] > math.pi * 3 / 2 and result[-1][2] < math.pi * 2:
            if result[-1][2] >= result[0][0]:
                if result[-1][0] >= result[0][0]:
                    del result[-1]
                else:
                    result[0][0] = result[-1][0]
                    result[0][1] = result[-1][1]
                    del result[-1]

        return result
        # if len(sectors_directions) == 0: return sectors_directions
        # sectors_directions.sort(key=lambda x: x[0])
        # result = []
        # result.append(sectors_directions[0])
        # for i in range(1, len(sectors_directions)):
        #     last = result[-1]
        #     if last[1] >= sectors_directions[i][0]:
        #         result[-1] = [last[0], max(last[1], sectors_directions[i][1])]
        #     else:
        #         result.append(sectors_directions[i])
        # return result

    def direction_twoPoints(self, start_point, end_point):
        # from start_point to end_point
        # vector[x, y]
        # return: -pi -> +pi ==> 0 -> 2pi
        vector = end_point - start_point
        direction = math.atan2(vector[1], vector[0])
        if direction < 0:
            return math.atan2(vector[1], vector[0]) + math.pi * 2
        return math.atan2(vector[1], vector[0])
    def distance_twoPoints(self, P1, P2):
        return math.sqrt((P1[0] - P2[0])**2 + (P1[1] - P2[1])**2)

    def sort_human_states(self, self_state:FullState, human_states):
        sort_list = []
        Rcenter = np.array([self_state.px, self_state.py])
        for i, human_state in enumerate(human_states):
            Hcenter = np.array([human_state.px, human_state.py])
            direction = self.direction_twoPoints(Rcenter, Hcenter)
            sort_list.append([i, direction])
        # sort direction from 0 to 2pi
        sort_list.sort(key=lambda x: x[1])
        result = human_states
        for i, list in enumerate(sort_list):
            index = list[0]
            result[i] = human_states[index]
        return result

    def tangent_point_PointCircle(self, point, center, r):
        # compute 2 tangent points between one outside point and one circle
        # point: np, [x, y]
        # center: np, [cx, cy]
        # return: s1, s2, np, [x1, y1] and [x2, y2]
        Rx = point[0]
        Ry = point[1]
        Px = center[0]
        Py = center[1]

        k = -(Rx - Px) / (Ry - Py)
        b = Py - k * Px
        A = 1 + k ** 2
        B = 2 * (b - Py) * k - 2 * Px
        C = Px ** 2 + (b - Py) ** 2 - r ** 2

        x1 = (-B + math.sqrt(B ** 2 - 4 * A * C)) / (2 * A)
        y1 = k * x1 + b
        s1 = np.array([x1, y1])
        x2 = (-B - math.sqrt(B ** 2 - 4 * A * C)) / (2 * A)
        y2 = k * x2 + b
        s2 = np.array([x2, y2])
        return s1, s2

    def intersection_point_AnylineCircle(self, center, r, other_point, theta):
        # any line is contruct by center and other_point, direct from other_point
        # compute intersection point between any line and circle
        cx = center[0]
        cy = center[1]
        x = other_point[0]
        y = other_point[1]

        k = math.tan(theta)
        b = y - k * x
        A = 1 + k ** 2
        B = 2 * k * (b - cy) - 2 * cx
        C = cx ** 2 + (b - cy) ** 2 - r ** 2
        x1 = (-B + math.sqrt(math.pow(B, 2) - 4 * A * C)) / (2 * A)
        y1 = k * x1 + b
        x2 = (-B - math.sqrt(math.pow(B, 2) - 4 * A * C)) / (2 * A)
        y2 = k * x2 + b
        if math.pow(x - x1, 2) + math.pow(y - y1, 2) > math.pow(x - x2, 2) + math.pow(y - y2, 2):
            sx = x1
            sy = y1
        else:
            sx = x2
            sy = y2
        return np.array([sx, sy])

    def intersection_point_ClineCircle(self, center, r, other_point):
        # Cline is contruct by center and other_point, direct from center to other_point
        # compute intersection point between Cline and circle
        cx = center[0]
        cy = center[1]
        x = other_point[0]
        y = other_point[1]

        a = (cy - y) / (cx - x)
        b = y - cy
        A = 1 + a ** 2
        B = -2 * cx - 2 * math.pow(a, 2) * x + 2 * a * b
        C = math.pow(cx, 2) + math.pow(a, 2) * math.pow(x, 2) - 2 * a * b * x + math.pow(b, 2) - math.pow(r, 2)
        x1 = (-B + math.sqrt(math.pow(B, 2) - 4 * A * C)) / (2 * A)
        y1 = a * (x1 - x) + y
        x2 = (-B - math.sqrt(math.pow(B, 2) - 4 * A * C)) / (2 * A)
        y2 = a * (x2 - x) + y
        if math.pow(x - x1, 2) + math.pow(y - y1, 2) < math.pow(x - x2, 2) + math.pow(y - y2, 2):
            sx = x1
            sy = y1
        else:
            sx = x2
            sy = y2
        return np.array([sx, sy])

    def state_constrain(self, self_state, human_states):
        bound_constrain_A = np.array([[1, 0], [-1, 0], [0, 1], [0, -1]])
        bound_constrain_b = np.array([[self.ub], [self.ub], [self.ub], [self.ub]])
        state_constrain = Polyhedron(bound_constrain_A, bound_constrain_b)

        all_collion_cons_now = self.all_collion_constrain(self_state, human_states)
        state_constrain = state_constrain.addConstraints(all_collion_cons_now.getA(), all_collion_cons_now.getB())

        state_constrain = state_constrain.minHRep()
        vertexs = state_constrain.getVertices()
        self.output['iris_vertexs'] = vertexs

        if self.system == "unicycle":
            theta_A = np.array([[1], [-1]])
            theta_b = np.array([[2 * 3.1415926], [0]])
            theta_cons = Polyhedron(theta_A, theta_b)
            state_constrain = state_constrain.addDimensions(theta_cons)

        state_constrain = state_constrain.minHRep()
        self.output['iris_poly'] = state_constrain
        return state_constrain

    def system_matrix(self, self_state:FullState):
        if self.system == "holonomic":
            A = np.eye(2, dtype=float)
            B = self.T * np.eye(2, dtype=float)
            return A, B
        elif self.system == "unicycle":
            Vlength = math.sqrt(math.pow(self_state.vx, 2) + math.pow(self_state.vy, 2))
            sin = math.sin(self_state.theta)
            cos = math.cos(self_state.theta)
            A = np.array([[1, 0, -self.T * Vlength * sin],
                          [0, 1,  self.T * Vlength * cos],
                          [0, 0,                       1]])
            B = np.array([[self.T * cos, 0],
                          [self.T * sin, 0],
                          [0,       self.T]])
            return A, B

    def input_constrain(self):
        if self.system == "holonomic":
            A = np.array([[1, 0],
                            [-1, 0],
                            [0,  1],
                            [0, -1]])
            a = np.array([[1], [1], [1], [1]])
            cons = Polyhedron(A, a)
            return cons
        elif self.system == "unicycle":
            vA = np.array([[1], [-1]])
            va = np.array([[1], [0]])
            wA = np.array([[1], [-1]])
            wa = np.array([[np.pi], [np.pi]])
            v_cons = Polyhedron(vA, va)
            w_cons = Polyhedron(wA, wa)
            v_cons = v_cons.addDimensions(w_cons)
            return v_cons