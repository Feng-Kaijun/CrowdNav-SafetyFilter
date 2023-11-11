import logging
import copy
import torch
import math
import signal
import numpy as np
from crowd_sim.envs.utils.info import *
from safety_filter.src.python.PySafetyFilter.safety_filter import SafetyFilter

class Explorer(object):
    def __init__(self, env, robot, device, memory=None, gamma=None, target_policy=None):
        self.env = env
        self.robot = robot
        self.device = device
        self.memory = memory
        self.gamma = gamma
        self.target_policy = target_policy
        self.target_model = None

    def update_target_model(self, target_model):
        self.target_model = copy.deepcopy(target_model)

    def signal_handler(self, signum, frame):
        raise NameError('SIGSEGV_Error')

    # @profile
    def run_k_episodes(self, k, phase, update_memory=False, imitation_learning=False, episode=None,
                       print_failure=False):
        self.robot.policy.set_phase(phase)
        success_times = []
        collision_times = []
        timeout_times = []
        success = 0
        collision = 0
        timeout = 0
        too_close = 0
        min_dist = []
        cumulative_rewards = []
        collision_cases = []
        timeout_cases = []

        # SF = SafetyFilter(lb = -5, ub = 5, K = 0.25, phi = math.pi / 6, system="holonomic",
        #                   T=0.25, speed_samples=5, rotation_samples=16)

        for i in range(k):
            ob = self.env.reset(phase)
            done = False
            states = []
            actions = []
            rewards = []
            while not done:
                action = self.robot.act(ob)

                # SFoutput = SF.compuate_safe_set(self.robot.get_full_state(), ob)
                # action = SF.compuate_safe_action(self.robot.get_full_state(), action, ob)
                # ob, reward, done, info = self.env.step(action, SFoutput)

                SFoutput = []
                ob, reward, done, info = self.env.step(action, SFoutput)

                # try:
                #     signal.signal(signal.SIGSEGV, self.signal_handler)
                #     SFoutput = SF.compuate_safe_set(self.robot.get_full_state(), ob)
                # except NameError:
                #     SFoutput = {
                #         'safe_action': None,
                #         'unsafe_action': None,
                #         'iris_vertexs': None,
                #         'iris_poly': None,
                #         'obstacle_hullPoints': None,
                #         'safe_set': None,
                #         'safe_set_poly': None,
                #         'local_goal': None,
                #         'vo_vertex': None,
                #         'path': None,
                #         'time': None
                #     }
                #     logging.info('can not find a safe set in this state')
                # else:
                #     action = SF.compuate_safe_action(self.robot.get_full_state(), action, ob)
                # finally:
                #     ob, reward, done, info = self.env.step(action, SFoutput)

                states.append(self.robot.policy.last_state)
                actions.append(action)
                rewards.append(reward)

                if isinstance(info, Danger):
                    too_close += 1
                    min_dist.append(info.min_dist)
                
            if isinstance(info, ReachGoal):
                success += 1
                success_times.append(self.env.global_time)
                # logging.info('test process: %d / %d ### success', i + 1, k)
            elif isinstance(info, Collision):
                collision += 1
                collision_cases.append(i)
                collision_times.append(self.env.global_time)
                # logging.info('test process: %d / %d ### collision', i + 1, k)
            elif isinstance(info, Timeout):
                timeout += 1
                timeout_cases.append(i)
                timeout_times.append(self.env.time_limit)
                # logging.info('test process: %d / %d ### timeout', i + 1, k)
            else:
                raise ValueError('Invalid end signal from environment')

            logging.info('test process: %d / %d ### %d human ### success: %0.3f ### collision: %0.3f ### timeout: %0.3f', i, k, self.env.human_num ,success/(i + 1), collision/(i + 1), timeout/(i + 1))

            if update_memory:
                if isinstance(info, ReachGoal) or isinstance(info, Collision):
                    # only add positive(success) or negative(collision) experience in experience set
                    self.update_memory(states, actions, rewards, imitation_learning)

            cumulative_rewards.append(sum([pow(self.gamma, t * self.robot.time_step * self.robot.v_pref)
                                           * reward for t, reward in enumerate(rewards)]))

            # if i == 1:
            #     self.env.render('traj', None)

        success_rate = success / k
        collision_rate = collision / k
        assert success + collision + timeout == k
        avg_nav_time = sum(success_times) / len(success_times) if success_times else self.env.time_limit

        extra_info = '' if episode is None else 'in episode {} '.format(episode)
        logging.info('{:<5} {}has success rate: {:.4f}, collision rate: {:.4f}, nav time: {:.2f}, total reward: {:.4f}'.
                     format(phase.upper(), extra_info, success_rate, collision_rate, avg_nav_time,
                            average(cumulative_rewards)))
        if phase in ['val', 'test']:
            num_step = sum(success_times + collision_times + timeout_times) / self.robot.time_step
            logging.info('Frequency of being in danger: %.4f and average min separate distance in danger: %.4f',
                         too_close / num_step, average(min_dist))

        if print_failure:
            logging.info('Collision cases: ' + ' '.join([str(x) for x in collision_cases]))
            logging.info('Timeout cases: ' + ' '.join([str(x) for x in timeout_cases]))

    def update_memory(self, states, actions, rewards, imitation_learning=False):
        if self.memory is None or self.gamma is None:
            raise ValueError('Memory or gamma value is not set!')

        for i, state in enumerate(states):
            reward = rewards[i]

            # VALUE UPDATE
            if imitation_learning:
                # define the value of states in IL as cumulative discounted rewards, which is the same in RL
                state = self.target_policy.transform(state)
                # value = pow(self.gamma, (len(states) - 1 - i) * self.robot.time_step * self.robot.v_pref)
                value = sum([pow(self.gamma, max(t - i, 0) * self.robot.time_step * self.robot.v_pref) * reward
                             * (1 if t >= i else 0) for t, reward in enumerate(rewards)])
            else:
                if i == len(states) - 1:
                    # terminal state
                    value = reward
                else:
                    next_state = states[i + 1]
                    gamma_bar = pow(self.gamma, self.robot.time_step * self.robot.v_pref)
                    value = reward + gamma_bar * self.target_model(next_state.unsqueeze(0)).data.item()
            value = torch.Tensor([value]).to(self.device)

            self.memory.push((state, value))


def average(input_list):
    if input_list:
        return sum(input_list) / len(input_list)
    else:
        return 0
