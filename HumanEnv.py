from rllab.envs.base import Env
from rllab.spaces import Box
from rllab.envs.base import Step
import numpy as np
from rllab.core.serializable import Serializable
import socket
import struct
import logging

logger = logging.getLogger(__name__)

class HumanEnv(Env):

    def __init__(self, window=1, hold=1, alpha=0.0, log_dir=None, record_log=True):
        Serializable.quick_init(self, locals())
        # Connect to the simulation in Bullet
        self.HOST, self.PORT = 'localhost', 47138
        self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.s.connect((self.HOST, self.PORT))
        self.s.send(b'Hello!')
        state = self.receive_state()

        # Height at which to fail the episode
        self.y_threshold = 0.5

        # Number of frames to concatenate together in the state
        self.window = window
        self.dim = 22
        self.usedDim = self.process(np.ones(self.dim)).shape[0]
        self.state = np.zeros(self.usedDim*window)

        # Number of frames to apply the same input
        self.hold = hold

        # Parameter for exponential average of the actions
        self.alpha = alpha
        self.a = np.zeros(16)

    def restore_socket(self):
        self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.s.connect((self.HOST, self.PORT))
        self.s.send(b'Hello!')

    def receive_state(self):
        stateSize = self.s.recv(4);
        stateMemSize = struct.unpack('i',stateSize)[0]
        print('Reading '+str(stateMemSize)+' bytes of state data')
        state = self.s.recv(stateMemSize)
        while len(state) < stateMemSize:
            state += self.s.recv(stateMemSize - len(state))
        state = np.asarray(np.fromstring(state,dtype='float32')).astype('double')
        print('Read '+str(stateMemSize)+' bytes of state data')
        return state

    def send_action(self,action):
        c = np.asarray(action)
        self.s.send(np.asarray([len(c)]).astype('int32').tostring())
        self.s.send(c.astype('float32').tostring())

    def send_reset(self):
        zeros = np.zeros(18).astype(int)
        buff = struct.pack('%si' % len(zeros), *zeros)
        self.s.send(np.asarray([18]).astype('int32').tostring())
        self.s.send(b'RESET')
        self.s.send(buff[:67])

    @property
    def observation_space(self):
        return Box(low=-1e6, high=1e6, shape=(self.usedDim*self.window,))

    @property
    def action_space(self):
        return Box(low=-30.0, high=30.0, shape=(16,))

    def process(self, state):
        mask = np.ones(self.dim).astype(bool)
        mask[[0,1,2]] = False # Drop the x and z coordinates of the root joint
        return state[mask]

    def reset(self):
        # Reset the simulation
        self.send_reset()

        # Get the state
        state = self.receive_state()

        self.state = np.zeros(self.usedDim*self.window)
        #self.state[self.usedDim*(self.window-1):] = state
        #print(state)

        # Go one more step because the current state is invalid
        c = np.zeros(16)
        self.send_action(c)

        # Get the state
        state = self.receive_state()
        self.state[self.usedDim*(self.window-1):] = self.process(state)

        self.steps_beyond_done = None
        self.a = np.zeros(16)
        self.lastX = 0.0
        return np.array(self.state)

    def step(self, action):
        assert self.action_space.contains(action), "%r (%s) invalid"%(action, type(action))
        state = self.state
        self.a = self.alpha*self.a + (1-self.alpha)*action
        action = self.a
        
        # Apply the action
        for _ in range(self.hold):
            self.send_action(action)

            # Receive the state
            state = self.receive_state()
        
        # Update the state
        self.state[:self.usedDim*(self.window-1)] = self.state[self.usedDim:]
        self.state[self.usedDim*(self.window-1):] = self.process(state)

        # Get the y position of the root joint
        y = state[1]
        x = state[0]
        done = y < self.y_threshold

        if not done:
            reward = x - self.lastX
            self.lastX = x
        elif self.steps_beyond_done is None:
            # skeleton just fell!
            self.steps_beyond_done = 0
            reward = -1.0
        else:
            if self.steps_beyond_done == 0:
                logger.warn("You are calling 'step()' even though this environment has already returned done = True. You should always call 'reset()' once you receive 'done = True' -- any further steps are undefined behavior.")
            self.steps_beyond_done += 1
            reward = 0.0

        next_observation = np.array(self.state)
        self.state = next_observation
        return Step(observation=next_observation, reward=reward, done=done)
        #return np.array(self.state), reward, done, {}

    def render(self,mode='human',close=False):
        print (self.state[0:4])
