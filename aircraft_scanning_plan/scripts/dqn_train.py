#! /usr/bin/env python
from __future__ import absolute_import, division, print_function

import sys
import os
import time
from datetime import datetime
import numpy as np
import tensorflow as tf
import pickle
import argparse

from env.uav_scanning_env import UAVScanningEnv, ViewPoint
from agent.dqn import DQNAgent

def get_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('--ep', type=int, default=10000)
    parser.add_argument('--lr', type=float, default=0.001)
    parser.add_argument('--bs', type=int, default=2048)
    parser.add_argument('--ad', type=int, default=8)
    parser.add_argument('--tc', type=float, default=0.95)
    return parser.parse_args()

def create_agent_params(name, state_dim, action_dim, layer_sizes, learning_rate, mem_cap):
    params = {}
    params['name'] = name
    params['state_dim'] = state_dim
    params['action_dim'] = action_dim
    params['layer_sizes'] = layer_sizes
    params['learning_rate'] = learning_rate
    params['memory_cap'] = mem_cap
    return params

def create_trajectory(env, agent, dir, target, index):
    state = env.reset()
    done, coverage = False, 0
    while done == False and coverage < target:
        vps = np.asarray(state.vpsState)
        digits = agent.qnet_active(vps.reshape(1,-1))
        action = np.argmax(digits.numpy().flatten())
        vpIdx = state.neighbors()[action]
        done, reward, coverage, state= env.play(vpIdx)
    file = "trajecoty_"+str(index)+".txt"
    env.save(os.path.join(dir,file))
    print("save trajectory to ", os.path.join(dir,file))

if __name__ == "__main__":
    args = get_args()
    print("episode:",args.ep,"learning rate:",args.lr,"batcg size:",args.bs,"action dimenension:", args.ad, "target coverage:", args.tc)
    vpFile = os.path.join(os.path.dirname(sys.path[0]+"/viewpoint/"),"viewpoints.txt")
    env = UAVScanningEnv(vpFile,ViewPoint(0,-30,5,0,0,0,0,1,0),args.ad)

    target_coverage = args.tc
    state_dim = len(env.util.viewpoints)
    action_dim = args.ad;
    layer_sizes = [200] # [512,512]
    learning_rate = args.lr
    mem_cap = 100000
    batch_size = args.bs
    discount_rate = 0.99
    decay_rate = 0.9998
    agent_params = create_agent_params("active",state_dim,action_dim,layer_sizes,learning_rate, mem_cap)
    agent = DQNAgent(agent_params)

    date_time = datetime.now().strftime("%Y-%m-%d-%H-%M")
    output_dir = os.path.join(os.path.dirname(sys.path[0]+"/saved_models/dqn/"),date_time)

    # for display the rewards in tersorboard
    summary_writer = tf.summary.create_file_writer(output_dir)
    summary_writer.set_as_default()

    start_time = time.time()

    # learning
    step = 0
    ep = 0
    while ep <= args.ep:
        epsilon = agent.decay_epsilon(ep,decay_rate,1.0,0.1,1)
        state = env.reset()
        rewards = []
        done, coverage = False, 0
        vpCount = 1
        while done == False and coverage < target_coverage and vpCount < state_dim-action_dim:
            vps = np.asarray(state.vpsState)
            action = agent.epsilon_greedy(vps,action_dim)
            vpIdx = state.neighbors()[action]

            done, reward, coverage, nextState = env.play(vpIdx)
            rewards.append(reward)
            nextvps = np.asarray(nextState.vpsState)
            agent.replay_memory.store((vps,action,reward,done,nextvps))
            agent.train(batch_size,discount_rate)

            state = nextState
            step += 1
            vpCount += 1
            # update q-stable net
            if not step%10000:
                agent.qnet_stable.set_weights(agent.qnet_active.get_weights())
                print("stable network is updated.")

        print("epsiode:",ep,"total reward:",sum(rewards),"visited:",vpCount,"coverage:",coverage)
        tf.summary.scalar("total reward", sum(rewards), step=ep)
        tf.summary.scalar("visited vps", vpCount, step=ep)
        if not ep%1000: # save trajectory every 1000 episodes
            create_trajectory(env, agent, output_dir, target_coverage,ep)
        ep+=1

    end_time = time.time()
    train_duration = end_time-start_time
    print("Trainning spent ", train_duration)
    # generate a viewpoints trajectory
    env.reset()
    # save model
    agent.save_model(output_dir)
    agent.plot_model(output_dir)
