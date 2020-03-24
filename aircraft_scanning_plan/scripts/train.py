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

def create_trajectory(env, agent, dir, index):
    done = False
    state = env.concatenate_state()
    while done == False:
        digits = agent.qnet_active(state.reshape(1,-1))
        action = np.argmax(digits.numpy().flatten())
        done, reward, coverage = env.action(action)
        state = env.concatenate_state()
    file = "trajecoty_"+str(index)+".txt"
    env.save(os.path.join(dir,file))
    print("save trajectory to ", os.path.join(dir,file))

if __name__ == "__main__":
    args = get_args()
    print("episode:",args.ep,"learning rate:",args.lr,"batcg size:",args.bs,"action dimenension:", args.ad)
    env = UAVScanningEnv(args.ad)
    viewpoints = os.path.join(os.path.dirname(sys.path[0]+"/viewpoint/"),"viewpoints.txt")
    env.load(viewpoints)

    state_dim = env.state_dimension();
    action_dim = args.ad;
    layer_sizes = [64,64] # [512,512]
    learning_rate = args.lr
    mem_cap = 100000
    batch_size = args.bs
    discount_rate = 0.98
    decay_rate = 0.9998
    agent_params = create_agent_params("active",state_dim,action_dim,layer_sizes,learning_rate, mem_cap)
    agent = DQNAgent(agent_params, env)

    date_time = datetime.now().strftime("%Y-%m-%d-%H-%M")
    output_dir = os.path.join(os.path.dirname(sys.path[0]+"/saved_models/dqn/"),date_time)

    # for display the rewards in tersorboard
    summary_writer = tf.summary.create_file_writer(output_dir)
    summary_writer.set_as_default()

    start_time = time.time()

    # learning
    step = 0
    ep = 1
    start_vp = ViewPoint(0,-30,5,0,0,0,0,1,0)
    while ep <= args.ep:
        env.reset(start_vp)
        state = env.concatenate_state()
        epsilon = agent.decay_epsilon(ep,decay_rate,1.0,0.1,1)
        done, rewards, coverage = False, [], 0
        vp_count = 0;
        while vp_count < 100 and done == False:
            action = agent.epsilon_greedy(state,action_dim)
            done, reward, coverage = env.action(action)
            rewards.append(reward)
            next_state = env.concatenate_state()
            # record state and next state for trainig
            agent.replay_memory.store((state,action,reward,done,next_state))
            agent.train(batch_size,discount_rate)

            state = next_state
            step += 1
            vp_count += 1
            # update q-stable net
            if not step%20000:
                agent.qnet_stable.set_weights(agent.qnet_active.get_weights())
                print("stable network is updated.")
        visited = env.state_vps(True)
        unvisited = env.state_vps(False)
        print("epsiode:",ep,"total reward:",sum(rewards),"visited:",len(visited),"unvisited:",len(unvisited),"coverage:",coverage)
        tf.summary.scalar("total reward", sum(rewards), step=ep)
        tf.summary.scalar("coverage", coverage, step=ep)
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
