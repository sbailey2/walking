from rllab.envs.base import Env
from rllab.spaces import Box
from rllab.envs.base import Step
import numpy as np
from rllab.core.serializable import Serializable
import socket
import struct
import logging
import Conversions

import time

import scipy.io as sio

logger = logging.getLogger(__name__)

class HumanPoseEnv(Env):

    def __init__(self, window=1, hold=1, alpha=0.0, log_dir=None, record_log=True, pos_obs=False):
        Serializable.quick_init(self, locals())
        # Connect to the simulation in Bullet
        self.HOST, self.PORT = 'localhost', 47138
        self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.s.connect((self.HOST, self.PORT))
        self.HOST, self.PORT = 'localhost', 47138
        self.sendSock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sendSock.connect((self.HOST, self.PORT))
        #self.s.send(b'Hello!')
        self.dim = 22
        #state = self.receive_state()

        # Height at which to fail the episode
        self.y_threshold = 0.5

        # Number of frames to concatenate together in the state
        self.window = window+1
        self.usedDim = self.process(np.ones(self.dim)).shape[0]
        self.state = np.zeros(self.usedDim*self.window)

        # Number of frames to apply the same input
        self.hold = hold

        # Parameter for exponential average of the actions
        self.alpha = alpha
        self.a = np.zeros(16)

        # Load the mocap data
        fileName = '/home/sbailey/Documents/CS294-129/project/rllab-chen/deeplocomotion/Bullet/MocapData.mat'
        data = sio.loadmat(fileName)
        #usedJoints = ['rhumerus','rradius','lradius','rfemur','rtibia','lfemur','ltibia','lhumerus']
        usedJoints = ['rfemur','rtibia','lfemur','ltibia','rhumerus','rradius','lhumerus','lradius']
        jointNames = [name.strip() for name in data['jointNames']]
        jointIdx = [jointNames.index(name) for name in usedJoints]
        jointStartIdx = [0]+list(np.cumsum(data['DOF']))
        usedStartIdx = [jointStartIdx[i] for i in jointIdx]
        usedLength = [data['DOF'][0,i] for i in jointIdx]
        jointMask = []
        for i,l in zip(usedStartIdx, usedLength):
            jointMask += range(i,i+l)
        self.mocapData = data['data'][0]
        self.mocapMask = np.asarray(jointMask)
        self.pos_obs = pos_obs

    def restore_socket(self):
        self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.s.connect((self.HOST, self.PORT))
        #self.s.send(b'Hello!')

    def receive_state(self):
        stateSize = self.s.recv(4);
        stateMemSize = struct.unpack('i',stateSize)[0]
        #print('Reading '+str(stateMemSize)+' bytes of state data')
        state = self.s.recv(stateMemSize)
        while len(state) < stateMemSize:
            state += self.s.recv(stateMemSize - len(state))
        state = np.asarray(np.fromstring(state,dtype='float32')).astype('double')
        #print('Read '+str(stateMemSize)+' bytes of state data')
        goodState = np.zeros(self.dim)
        goodState[:len(state)] = state
        return goodState

    def send_action(self,action):
        c = np.asarray(action)
        self.sendSock.sendall(np.asarray([len(c)]).astype('int32').tostring())
        self.sendSock.sendall(c.astype('float32').tostring())

    def send_reset(self):
        zeros = np.zeros(18).astype(int)
        buff = struct.pack('%si' % len(zeros), *zeros)
        self.sendSock.sendall(np.asarray([18]).astype('int32').tostring())
        self.sendSock.sendall(b'RESET')
        self.sendSock.sendall(buff[:67])

    def normalize_rotation_data(self, data):
        data = data + 180
        data = np.mod(data,360)
        data = data - 180
        return data * np.pi / 180

    @property
    def observation_space(self):
        return Box(low=-1e6, high=1e6, shape=(self.usedDim*self.window,))

    @property
    def action_space(self):
        return Box(low=-10.0, high=10.0, shape=(16,))

    def process(self, state):
        mask = np.ones(self.dim).astype(bool)
        mask[[0,1,2,3,4,5]] = False # Drop the x and z coordinates of the root joint
        state = state[mask]
        if self.pos_obs:
            return Converversions.rot2pos(state)
        else:
            return state

    def add_mocap_observation(self):
        frame = min(self.frame,self.mocapSample.shape[0])
        frame = self.normalize_rotation_data(self.mocapSample[frame,self.mocapMask].copy())
        if self.pos_obs:
            self.state[:self.usedDim] = Conversions.rot2pos(frame)
        else:
            self.state[:self.usedDim] = frame

    def reset(self):
        # Reset the simulation
        self.send_reset()

        # Pick a mocap segment to copy
        self.mocapSample = self.mocapData[np.random.randint(0,len(self.mocapData))]
        self.frame = np.random.randint(0,len(self.mocapSample))
        #self.mocapSample = self.mocapData[23]
        #self.frame = 123 % len(self.mocapSample)

        # Get the state
        state = self.normalize_rotation_data(self.receive_state())

        self.state = np.zeros(self.usedDim*self.window)
        #self.state[self.usedDim*(self.window-1):] = state
        #print(state)

        # Go one more step because the current state is invalid
        #c = np.zeros(16)
        #self.send_action(c)

        # Apply a few random steps to have a random start
        numSteps = 10
        for _ in range(numSteps):
            action = self.action_space.sample()
            #a = np.zeros(16)
            #a[:8] = action
            #action = a
            self.send_action(action)
            state = self.normalize_rotation_data(self.receive_state())
            self.state[:self.usedDim*(self.window-1)] = self.state[self.usedDim:]
            self.state[self.usedDim*(self.window-1):] = self.process(state)

        # Get the state
        #state = self.normalize_rotation_data(self.receive_state())
        #self.state[self.usedDim*(self.window-1):] = self.process(state)
        self.add_mocap_observation()

        self.steps_beyond_done = None
        #self.a = np.zeros(16)
        self.a = action
        self.lastX = 0.0
        return np.array(self.state)

    def step(self, action):
        assert self.action_space.contains(action), "%r (%s) invalid"%(action, type(action))

        # Scale down some rotations because the torque adds a lot to the rotational force
        #action[2] *= 0.05
        #action[6] *= 0.05
        #action[8] *= 0.2
        #action[12] *= 0.2
        #action[8:] *= 0.75 # Scale down arm controls
        #a = np.zeros(16)
        #a[:8] = action
        #action = a
        
        state = self.state
        self.a = self.alpha*self.a + (1-self.alpha)*action
        action = self.a


        
        # Apply the action
        for _ in range(self.hold):
            self.send_action(action)

            # Receive the state
            state = self.normalize_rotation_data(self.receive_state())
        
        # Update the state
        self.state[:self.usedDim*(self.window-1)] = self.state[self.usedDim:]
        self.state[self.usedDim*(self.window-1):] = self.process(state)
        self.add_mocap_observation()
        #self.frame += 1

        # Get the y position of the root joint
        #y = state[1]
        #x = state[0]
        #done = y < self.y_threshold

        # Determine if we're at the last frame of the mocap data
        if self.pos_obs:
            eps = 0.01
        else:
            eps = 0.01 # About 6 degrees off on average
        #usedIdx = np.asarray([0,1,2,3,4,5,6,7])
        #usedIdx = np.asarray([0,1,2,3,4,5,6,7,8,9,10,11])
        #usedIdx = np.asarray([12,13,14,15])
        usedIdx = np.arange(self.process(state).shape[0])
        actionMagnitude = np.sum(np.square(action)) * 1e-4
        diff = np.mean(np.square(self.process(state)[usedIdx]-self.state[:self.usedDim][usedIdx]))
        #diff = np.mean(np.square(self.process(state)-self.state[:self.usedDim]))
        #diff += actionMagnitude
        done = False
        if diff < eps:
            done = True
            reward = 1.0
        if not done:
            reward = -diff-actionMagnitude
            #self.lastX = x
        elif self.steps_beyond_done is None:
            self.steps_beyond_done = 0
            reward = 1.0
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
