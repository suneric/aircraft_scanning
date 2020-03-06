#! /usr/bin/env python
from __future__ import absolute_import, division, print_function
import sys
import os
import numpy as np
import random
import pickle
import tensorflow as tf
from tensorflow import keras
from tensorflow.keras import layers
from tensorflow.keras.layers import Dense
from tensorflow.keras import Model

class Memory:
    def __init__(self, memory_cap):
        self.memory_cap = memory_cap
        self.memory = []
    def store(self, experience):
        if len(self.memory) >= self.memory_cap:
            self.memory.pop(random.randint(0, len(self.memory)-1))
        self.memory.append(experience)

    def sample_batch(self, batch_size):
        if len(self.memory) < batch_size:
            batch = random.sample(self.memory, len(self.memory))
        else:
            batch = random.sample(self.memory, batch_size)
        return zip(*batch)

class DQNAgent:
    def __init__(self, params):
        self.name = params['name']
        self.state_dim = params['state_dim']
        self.action_dim = params['action_dim']
        self.layer_sizes = params['layer_sizes']
        self.learning_rate = params['learning_rate']
        self.epsilon = 1
        self.replay_memory = Memory(memory_cap=params['memory_cap'])

        self.qnet_active = self._nn_mlp(self.state_dim, self.action_dim, self.layer_sizes)
        self.qnet_stable = tf.keras.models.clone_model(self.qnet_active)

        self.optimizer = tf.keras.optimizers.Adam(lr=self.learning_rate)
        self.loss_fn = tf.keras.losses.MeanSquaredError()
        self.mse_metric = tf.keras.metrics.MeanSquaredError()

    def train(self, batch_size, gamma):
        # data: (state, action, reward, next state) patch
        minibatch = self.replay_memory.sample_batch(batch_size)
        (b_states,b_actions,b_rewards,b_doneflags,b_nextstates,b_vpstates) = [np.array(minibatch[i]) for i in range(len(minibatch))]

        # GradientTape for automatic differentiation-computing the gradient of a compuation with respect
        # to its input and intermediate variables
        # record all operation executed inside the context of a gradienttape onto a tape
        # use the tape and the gradients associated with each recorded operation
        # to compute the gradients of a recorded computation using reverse mode differentiation
        with tf.GradientTape() as tape:
            # run forward pass
            logits_a = self.qnet_active(b_states)
            pred_q = tf.math.reduce_sum(tf.cast(logits_a,tf.float32)*tf.one_hot(b_actions,self.action_dim), axis=-1)

            # use stable network to stablize the q
            logits_s = self.qnet_stable(b_nextstates)
            target_q = b_rewards + (1.-b_doneflags)*gamma*tf.math.reduce_max(logits_s, axis=-1)

            # aim to convergent two networks
            loss_value = self.loss_fn(y_true=target_q, y_pred=pred_q)

        # retrieve the gradients of trainable variables with respect to the loss
        grads = tape.gradient(loss_value, self.qnet_active.trainable_weights)
        # run one step of gradient descent
        self.optimizer.apply_gradients(zip(grads,self.qnet_active.trainable_weights))

        # update, summary and reset metrics
        self.mse_metric(target_q, pred_q)
        #tf.summary.scalar("q value", self.mse_metric.result(), step=self.optimizer.iterations)
        self.mse_metric.reset_states()

    def decay_epsilon(self, episode, decay_rate, init_eps, final_eps, warmup_episodes=64):
        if episode >= warmup_episodes:
            self.epsilon *= decay_rate
        if self.epsilon <= final_eps:
            self.epsilon = final_eps
        return self.epsilon

    # choose next viewpoint (action) based on voxels state (oservation)
    def epsilon_greedy(self,voxels_state,vps_state):
        # select the action with possibility in unvisited viewpoints
        unvisited_vp_indices = np.where(vps_state==0)[0]
        visited_vp_indices = np.nonzero(vps_state)[0]
        #print(vps_state, unvisited_vp_indices, visited_vp_indices)
        vp_idx = 0
        if np.random.rand() > self.epsilon:
            # low epsilon means more randomly choose the action
            digits = self.qnet_active(voxels_state.reshape(1,-1))
            actions = digits.numpy().flatten()
            for vvi in visited_vp_indices:
                actions[vvi] = 0.0
            vp_idx = np.argmax(actions) # most possible actions in unvisited vps
        else:
            if len(unvisited_vp_indices) > 0:
                uvi = np.random.randint(len(unvisited_vp_indices))
                vp_idx = unvisited_vp_indices[uvi]
        return vp_idx

    # build multi-layer perceptron nueral network
    def _nn_mlp(self, state_dim, action_dim, layer_sizes):
        # way 1:  define a Sequential model
        model = tf.keras.Sequential()
        # add a densely-connected layer with layer_sizes[0] units with input
        model.add(Dense(layer_sizes[0], activation='relu', input_shape=(state_dim,)))
        # add other layers
        for i in range(1,len(layer_sizes)):
            model.add(Dense(layer_sizes[i], activation='relu'))
        # add output | softmax make sum of all actions possiblity is 1
        model.add(Dense(action_dim)) #activation='softmax'

        # way 2
        # inputs = tf.keras.Input(shape=(state_dim,),name='state')
        # x = layers.Dense(layer_sizes[0], activation='relu')(inputs)
        # for i in range(1,len(layer_sizes)):
        #     x = layers.Dense(layer_sizes[i], activation='relu')(x)
        # outputs = layers.Dense(action_dim, name='action')(x)
        # model = Model(inputs=inputs,outputs=outputs)
        return model

    def plot_model(self, model_dir):
        png = os.path.join(model_dir,'model.png')
        tf.keras.utils.plot_model(self.qnet_active, png)

    def save_model(self, model_dir):
        self.qnet_active.summary()
        self.qnet_stable.summary()
        # create model saving directory if not exist
        if not os.path.exists(model_dir):
            os.makedirs(model_dir)
        # save model
        self.qnet_active.save(os.path.join(model_dir,'active_model.h5'))
        self.qnet_stable.save(os.path.join(model_dir,'stable_model.h5'))
        print("Q_net models saved at {}".format(model_dir))