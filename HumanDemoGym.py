import numpy as np

import sys
sys.path.append('/home/sbailey/Documents/CS294-129/project/rllab/')

import gym
import HumanEnv
import HumanPoseEnv

import time

#env = gym.make('HumanEnv-v0')
env = HumanPoseEnv.HumanPoseEnv(1,1,alpha=0.75)
o = env.reset()

for i in range(100000):
    a = 0*env.action_space.sample()
    #a[11] = -10
    #a[9] = -1
    #a[-3] = -1
    #a[0] = -1
    #a[4] = 1
    #if o[16] < o[0]:
    #a[3]=-1
    #a[7]=-1

    # Leg Controls
    scale=1.25
    #a[0] = np.clip(scale*o[0]-o[16],-1,1)
    #a[1] = np.clip(scale*o[2]-o[18],-1,1)
    #a[2] = np.clip(o[1]-o[17],-1,1)
    #a[3] = np.clip(o[19]-o[3],-1,1)
    #a[4] = np.clip(o[20]-scale*o[4],-1,1)
    #a[5] = np.clip(scale*o[6]-o[22],-1,1)
    #a[6] = np.clip(o[5]-o[21],-1,1)
    #a[7] = np.clip(o[23]-o[7],-1,1)
    #a[3] = np.clip(o[3]-o[19]-0.01,-1,1)

    # Arm controls
    #a[11] = 0.2
    #a[12] = -1.0

    print('Action: '+str(a))
    o,r,d,info = env.step(a)
    #print('Target: '+str(o[:4]*180/np.pi))
    print('State: '+str(o[-4:]*180/np.pi))
    #print('Reward: '+str(r))
    if d:
        env.reset()
