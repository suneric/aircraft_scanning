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

from env.uav_scanning_env import UAVScanningEnv, UAVScanningState
from env.viewpoint_map import ViewPoint

# a node that represents
class MCTSNode(object):
    def __init__(self, util, state, parent=None):
        self.util = util
        self.state = state
        self.parent = parent
        self.children = []

        self.neighbors = self.state.neighbors()
        self.untriedVps = [self.neighbors[i] for i in range(len(self.neighbors))]

        self.result = 0.
        self.numberOfVisit = 0.

    def vp_index(self):
        return self.state.vp()

    def best_child(self, c_param=1.414):
        weights = [child.q()+c_param*np.sqrt(np.log(self.n())/child.n()) for child in self.children]
        return self.children[np.argmin(weights)]

    def child_nodes(self):
        return self.children
    def parent(self):
        return self.parent

    def n(self):
        return self.numberOfVisit
    def q(self):
        return self.result

    def is_terminal_node(self):
        return self.state.coverage() == 1.0 or len(self.neighbors) < self.util.action_dim

    def is_fully_expanded(self):
        return len(self.untriedVps) == 0

    def expand(self):
        vpIdx = self.untriedVps.pop()
        nextState = self.state.move(vpIdx)
        child = MCTSNode(self.util, nextState, parent=self)
        self.children.append(child)
        return child

    def rollout_policy(self, possibleMoves):
        return possibleMoves[np.random.randint(len(possibleMoves))]

    def rollout(self):
        if self.is_terminal_node():
            return 0

        initCoverage = self.state.coverage()
        cState = self.state
        cVp = cState.vp()
        neighbors = cState.neighbors()
        coverage = cState.coverage()
        cost = 0
        while coverage < 1.0 and len(neighbors) >= self.util.action_dim:
            nextVp = self.rollout_policy(neighbors)
            cState = cState.move(nextVp)
            cost += self.util.distance_i(cVp,nextVp)
            cVp = nextVp
            neighbors = cState.neighbors()
            coverage = cState.coverage()
        result = (coverage-initCoverage)/cost
        return result

    def backpropagate(self,result):
        self.numberOfVisit += 1.
        self.result += result
        if self.parent:
            self.parent.backpropagate(result)

class MonteCarloTreeSearch(object):
    def __init__(self, node):
        self.root = node

    def tree_policy(self):
        node = self.root
        while not node.is_terminal_node():
            if node.is_fully_expanded():
                node = node.best_child(c_param=1.4)
            else:
                return node.expand()
        return node

    def search(self, simulation_number, target, env):
        for i in range(simulation_number):
            v = self.tree_policy()
            r = v.rollout()
            v.backpropagate(r)
            if i%100 == 0:
                s = int(i/100)
                vps, coverage = self.test(env)
                tf.summary.scalar("coverage per viewpoint", coverage/vps, step=s)
                tf.summary.scalar("coverage", coverage, step=s)
                print("iteration", s, "test: viewpoints", vps, "coverage", coverage)
                if coverage >= target:
                    print("desired coverage achieved", target)
                    break
        return self.root.best_child()

    def test(self, env, saved=None):
        node = self.root
        state = env.reset()
        step = 1
        while not node.is_terminal_node():
            done, reward, coverage, state = env.play(node.vp_index())
            if node.is_fully_expanded():
                node = node.best_child(c_param=1.4)
                step += 1
            else:
                break
        # save trajectory
        if saved != None:
            env.save(saved)
            print("save trajectory to ", saved)

        return step, coverage

############################################################
# main
def get_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('--sn', type=int, default=50)
    parser.add_argument('--ad', type=int, default=8)
    parser.add_argument('--tc', type=float, default=0.95)
    return parser.parse_args()

if __name__ == "__main__":
    date_time = datetime.now().strftime("%Y-%m-%d-%H-%M")
    output_dir = os.path.join(os.path.dirname(sys.path[0]+"/saved_models/mcts/"),date_time)
    summary_writer = tf.summary.create_file_writer(output_dir)
    summary_writer.set_as_default()

    args = get_args()
    simulation_count = args.sn
    action_dim = args.ad
    target_coverage = args.tc
    print("parameters: action dimension", action_dim, "iteration number", simulation_count, "desired coverage", target_coverage)

    vpfile = os.path.join(os.path.dirname(sys.path[0]+"/viewpoint/"),"viewpoints.txt")
    env = UAVScanningEnv(vpfile,ViewPoint(0,-30,5,0,0,0,0,1,0),action_dim)
    start_time = time.time()
    state = env.reset()
    root = MCTSNode(env.utility(), state, parent=None)
    mcts = MonteCarloTreeSearch(root)
    mcts.search(simulation_count,target_coverage,env)

    end_time = time.time()
    train_duration = end_time-start_time
    print("monte carlo tree search:", simulation_count, "iteration", train_duration/3600, "hours")
    mcts.test(env, os.path.join(output_dir,"trajectory.txt"))
