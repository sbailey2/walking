#from rllab.envs.base import Env
#from rllab.spaces import Box
#from rllab.envs.base import Step
import gym
from gym import Env
from gym.spaces import Box
import numpy as np
#from rllab.core.serializable import Serializable
import socket
import struct
import logging

logger = logging.getLogger(__name__)

class HumanEnv(Env):

    def __init__(self, window=1, hold=1, log_dir=None, record_log=True):
        #Serializable.quick_init(self, locals())
        # Connect to the simulation in Bullet
        self.HOST, self.PORT = 'localhost', 47138
        self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.s.connect((self.HOST, self.PORT))
        self.s.send(b'Hello!')

        # Height at which to fail the episode
        self.y_threshold = 0.5

        # Number of frames to concatenate together in the state
        self.window = window
        self.dim = 25
        self.usedDim = self.process(np.ones(self.dim)).shape[0]
        self.state = np.zeros(self.usedDim*window)

        # Number of frames to apply the same input
        self.hold = hold

    def restore_socket(self):
        self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.s.connect((self.HOST, self.PORT))
        self.s.send(b'Hello!')

    @property
    def observation_space(self):
        return Box(low=-1e6, high=1e6, shape=(self.usedDim*self.window,))

    @property
    def action_space(self):
        return Box(low=-0.5, high=0.5, shape=(15,))

    def process(self, state):
        mask = np.ones(self.dim).astype(bool)
        mask[[0,2]] = False # Drop the x and z coordinates of the root joint
        return state[mask]

    def reset(self):
        # Reset the simulation
        self.s.send(b'RESET')
        zeros = np.zeros(20).astype(int)
        buff = struct.pack('%si' % len(zeros), *zeros)
        self.s.send(buff)

        # Get the state
        stateSize = self.s.recv(4);
        stateSize = struct.unpack('i',stateSize)[0]
        state = self.s.recv(1000)
        while len(state) < stateSize:
            state += self.s.recv(1000)
        state = np.asarray([struct.unpack('f',state[i:i+4])[0] for i in range(0,len(state),4)])
        self.state = np.zeros(self.usedDim*self.window)
        #self.state[self.usedDim*(self.window-1):] = state
        #print(state)

        # Go one more step because the current state is invalid
        c = np.zeros(15)
        buff = struct.pack('%sf' % len(c), *c)
        self.s.send(buff)

        # Get the state
        stateSize = self.s.recv(4);
        stateSize = struct.unpack('i',stateSize)[0]
        state = self.s.recv(1000)
        while len(state) < stateSize:
            state += self.s.recv(1000)
        state = np.asarray([struct.unpack('f',state[i:i+4])[0] for i in range(0,len(state),4)])
        self.state[self.usedDim*(self.window-1):] = self.process(state)
        print(self.state)
        print(self.usedDim)

        self.steps_beyond_done = None
        return np.array(self.state)

    def step(self, action):
        assert self.action_space.contains(action), "%r (%s) invalid"%(action, type(action))
        state = self.state
        
        # Apply the action
        for _ in range(self.hold):
            c = np.asarray(action)
            buff = struct.pack('%sf' % len(c), *c)
            self.s.send(buff)

            # Receive the state
            stateSize = self.s.recv(4);
            stateSize = struct.unpack('i',stateSize)[0]
            state = self.s.recv(1000)
            while len(state) < stateSize:
                state += self.s.recv(1000)
            state = np.asarray([struct.unpack('f',state[i:i+4])[0] for i in range(0,len(state),4)])
        
        # Update the state
        self.state[:self.usedDim*(self.window-1)] = self.state[self.usedDim:]
        self.state[self.usedDim*(self.window-1):] = self.process(state)

        # Get the y position of the root joint
        y = state[1]
        done = y < self.y_threshold

        if not done:
            reward = 1.0
        elif self.steps_beyond_done is None:
            # skeleton just fell!
            self.steps_beyond_done = 0
            reward = -100.0
        else:
            if self.steps_beyond_done == 0:
                logger.warn("You are calling 'step()' even though this environment has already returned done = True. You should always call 'reset()' once you receive 'done = True' -- any further steps are undefined behavior.")
            self.steps_beyond_done += 1
            reward = 0.0

        next_observation = np.array(self.state)
        self.state = next_observation
        #return Step(observation=next_observation, reward=reward, done=done)
        return np.array(self.state), reward, done, {}

    def render(self,mode='human',close=False):
        print (self.state[0:4])
