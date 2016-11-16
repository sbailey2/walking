import numpy as np

import gym
import HumanEnv

#env = gym.make('HumanEnv-v0')
env = HumanEnv.HumanEnv(2,1)
env.reset()

for i in range(1000):
    o,r,d,info = env.step(env.action_space.sample())
    if d:
        env.reset()
