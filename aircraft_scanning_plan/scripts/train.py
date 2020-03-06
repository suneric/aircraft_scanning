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

def create_agent_params(name, state_dim, action_dim, layer_sizes, learning_rate, mem_cap):
    params = {}
    params['name'] = name
    params['state_dim'] = state_dim
    params['action_dim'] = action_dim
    params['layer_sizes'] = layer_sizes
    params['learning_rate'] = learning_rate
    params['memory_cap'] = mem_cap
    return params

def create_trajectory(env, agent, dir, index):
    voxels_state = env.voxels_state_array()
    vps_state = env.viewpoints_state_array()
    vp_count = 0
    action_dim = len(vps_state)
    done, rewards = False, []
    while (vp_count < action_dim) and done == False:
        vp_idx = agent.epsilon_greedy(voxels_state, vps_state)
        done, reward = env.action(vp_idx)
        voxels_state = env.viewpoints_state_array()
        vps_state = env.voxels_state_array()
        rewards.append(reward)
        vp_count += 1

    file = "trajecoty_"+str(index)+".txt"
    env.save_visited(os.path.join(dir,file))
    print("save trajectory to ", os.path.join(dir,file))

if __name__ == "__main__":
    #args = get_args()
    env = UAVScanningEnv()
    viewpoints = os.path.join(os.path.dirname(sys.path[0]+"/viewpoint/"),"viewpoints.txt")
    env.load(viewpoints)

    state_dim = env.voxels_count();
    action_dim = env.viewpoints_count();
    layer_sizes = [500,500]
    learning_rate = 0.001
    mem_cap = 50000
    batch_size = 500
    discount_rate = 0.99
    decay_rate = 0.9999
    agent_params = create_agent_params("active",state_dim,action_dim,layer_sizes,learning_rate, mem_cap)
    agent = DQNAgent(agent_params)

    date_time = datetime.now().strftime("%Y-%m-%d-%H-%M")
    output_dir = os.path.join(os.path.dirname(sys.path[0]+"/saved_models/dqn/"),date_time)

    # for display the rewards in tersorboard
    summary_writer = tf.summary.create_file_writer(output_dir)
    summary_writer.set_as_default()

    start_time = time.time()

    # learning
    start_vp = ViewPoint(0,-30,5,0,0,0,0,1,0)
    step = 0;
    ep = 1;
    while ep <= 10000:
        env.reset(start_vp)
        voxels_state = env.voxels_state_array()
        vps_state = env.viewpoints_state_array()
        epsilon = agent.decay_epsilon(ep,decay_rate,1.0,0.1,1)

        done, rewards = False, []
        vp_count = 0
        while (vp_count < action_dim) and done == False:
            # state and action cache for training
            vp_idx = agent.epsilon_greedy(voxels_state, vps_state)
            done, reward = env.action(vp_idx)
            rewards.append(reward) # record total reward
            next_voxels_state = env.voxels_state_array()
            next_vps_state = env.viewpoints_state_array()
            agent.replay_memory.store((voxels_state,vp_idx,reward,done,next_voxels_state,next_vps_state))

            agent.train(batch_size,discount_rate)

            # update states
            voxels_state = next_voxels_state
            vps_state = next_vps_state
            step += 1
            vp_count += 1

            # update q-stable net
            if not step%20000:
                agent.qnet_stable.set_weights(agent.qnet_active.get_weights())
                print("stable network is updated.")

        visited_vps = env.visited_viewpoints()
        print("epsiode:",ep,"total reward:",sum(rewards), "viewpoint count:", len(visited_vps),"/",action_dim)
        tf.summary.scalar("total reward", sum(rewards), step=ep)
        tf.summary.scalar("viewpoint count", len(visited_vps), step=ep)
        if not ep%1000: # save trajectory every 1000 episodes
            create_trajectory(env, agent, output_dir, ep)
        ep+=1

    end_time = time.time()
    train_duration = end_time-start_time
    print("Trainning spent ", train_duration)
    # generate a viewpoints trajectory
    env.reset(start_vp)
    # save model
    agent.save_model(output_dir)
    agent.plot_model(output_dir)
