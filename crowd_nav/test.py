import logging
import argparse
import configparser
import math
import os
import torch
import numpy as np
import gym
from crowd_nav.utils.explorer import Explorer
from crowd_nav.policy.policy_factory import policy_factory
from crowd_sim.envs.utils.robot import Robot
from crowd_sim.envs.policy.orca import ORCA

from safety_filter.src.python.PySafetyFilter.safety_filter import SafetyFilter

import time

def main():
    parser = argparse.ArgumentParser('Parse configuration file')
    parser.add_argument('--env_config', type=str, default='data/221129/cadrl/env.config')
    parser.add_argument('--policy_config', type=str, default='data/221129/cadrl/policy.config')
    parser.add_argument('--policy', type=str, default='cadrl')
    parser.add_argument('--model_dir', type=str, default='data/221129/cadrl')
    parser.add_argument('--il', default=False, action='store_true')
    parser.add_argument('--gpu', default=False, action='store_true')
    parser.add_argument('--visualize', default=True, action='store_true')
    parser.add_argument('--phase', type=str, default='test')
    parser.add_argument('--test_case', type=int, default=52)
    parser.add_argument('--square', default=False, action='store_true')
    parser.add_argument('--circle', default=False, action='store_true')
    parser.add_argument('--video_file', type=str, default=None)
    parser.add_argument('--traj', default=True, action='store_true')
    args = parser.parse_args()

    if args.model_dir is not None:
        env_config_file = os.path.join(args.model_dir, os.path.basename(args.env_config))
        policy_config_file = os.path.join(args.model_dir, os.path.basename(args.policy_config))
        if args.il:
            model_weights = os.path.join(args.model_dir, 'il_model.pth')
        else:
            if os.path.exists(os.path.join(args.model_dir, 'resumed_rl_model.pth')):
                model_weights = os.path.join(args.model_dir, 'resumed_rl_model.pth')
            else:
                model_weights = os.path.join(args.model_dir, 'rl_model.pth')
    else:
        env_config_file = args.env_config
        policy_config_file = args.env_config

    # configure logging and device
    logging.basicConfig(level=logging.INFO, format='%(asctime)s, %(levelname)s: %(message)s',
                        datefmt="%Y-%m-%d %H:%M:%S")
    device = torch.device("cuda:0" if torch.cuda.is_available() and args.gpu else "cpu")
    logging.info('Using device: %s', device)

    # configure policy
    policy = policy_factory[args.policy]()
    policy_config = configparser.RawConfigParser()
    policy_config.read(policy_config_file)
    policy.configure(policy_config)
    if policy.trainable:
        if args.model_dir is None:
            parser.error('Trainable policy must be specified with a model weights directory')
        policy.get_model().load_state_dict(torch.load(model_weights, map_location='cuda:0'))

    # configure environment
    env_config = configparser.RawConfigParser()
    env_config.read(env_config_file)
    env = gym.make('CrowdSim-v0')
    env.configure(env_config)
    if args.square:
        env.test_sim = 'square_crossing'
    if args.circle:
        env.test_sim = 'circle_crossing'
    robot = Robot(env_config, 'robot')
    robot.set_policy(policy)
    env.set_robot(robot)
    explorer = Explorer(env, robot, device, gamma=0.9)

    policy.set_phase(args.phase)
    policy.set_device(device)
    # set safety space for ORCA in non-cooperative simulation
    if isinstance(robot.policy, ORCA):
        if robot.visible:
            robot.policy.safety_space = 0
        else:
            # because invisible case breaks the reciprocal assumption
            # adding some safety space improves ORCA performance. Tune this value based on your need.
            robot.policy.safety_space = 0
        logging.info('ORCA agent buffer: %f', robot.policy.safety_space)

    policy.set_env(env)
    robot.print_info()

    SF = SafetyFilter(lb = -5, ub = 5, K = 0.25, phi = math.pi / 6, system="holonomic",
                      T=0.25, speed_samples=5, rotation_samples=16)

    if args.visualize:
        ob = env.reset(args.phase, args.test_case)
        done = False
        last_pos = np.array(robot.get_position())
        step_nums = 0
        rl_step_time = 0
        sf_step_time = 0
        while not done:
            logging.info('%d', step_nums)
            step_nums += 1
            t1 = time.time()
            action = robot.act(ob)
            t2 = time.time()
            # SFoutput = SF.compuate_safe_set(robot.get_full_state(), ob)
            # safe_action = SF.compuate_safe_action(robot.get_full_state(), action, ob)
            SFoutput = []
            t3 = time.time()
            rl_step_time += int(round((t2 - t1) * 1000))
            sf_step_time += int(round((t3 - t2) * 1000))
            ob, _, done, info = env.step(action, SFoutput)
            current_pos = np.array(robot.get_position())
            logging.debug('Speed: %.2f', np.linalg.norm(current_pos - last_pos) / robot.time_step)
            last_pos = current_pos

        logging.info('It takes %0.2f ms for one RL step. ', rl_step_time / step_nums)
        logging.info('It takes %0.2f ms for one SF step. ', sf_step_time / step_nums)

        if args.traj:
            env.render('traj', args.video_file)
        else:
            env.render('video', args.video_file)

        logging.info('It takes %.2f seconds to finish. Final status is %s', env.global_time, info)
        if robot.visible and info == 'reach goal':
            human_times = env.get_human_times()
            logging.info('Average time for humans to reach goal: %.2f', sum(human_times) / len(human_times))
    else:
        explorer.run_k_episodes(env.case_size[args.phase], args.phase, print_failure=True)
        logging.info('5 human, unicycle, cadrl, new action space')
        # logging.info('randomize_human_num, holonomic, orca+sf, randomize_attributes, 1AF/10GNF/10ANF, all_vo = 0, STA*: Nsteps=1, i <= 1,'
        #              'obstacle_radius+3, Krep=40, 0.1H, 0E, 1S, gvo_free<=0')

        # logging.info('randomize_human_num, holonomic, sarl, randomize_attributes')

        # logging.info('5 human, unicycle, lstm_rl, randomize_attributes')

        # logging.info('5 human, holonomic, orca')


if __name__ == '__main__':
    main()
