import socket
import sys
import struct
import time

import numpy as np

HOST, PORT = 'localhost', 47138

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

s.connect((HOST, PORT))
print('Connected to host')

s.send(b'Hello!')
iteration = 0
while True:
    stateSize = s.recv(4);
    stateSize = struct.unpack('i',stateSize)[0]
    state = s.recv(1000)
    while len(state) < stateSize:
        state += s.recv(1000)
    state = np.asarray([struct.unpack('f',state[i:i+4])[0] for i in range(0,len(state),4)])
    c = np.random.uniform(-0.1,0.1,16)
    buff = struct.pack('%sf' % len(c), *c)
    iteration += 1
    if iteration % 1000 == 0:
        print('Iteration '+str(iteration))
        print(state.shape)
        s.send(b'RESET')
        zeros = np.zeros(20)
        buff = struct.pack('%sf' % len(zeros), *zeros)
        s.send(buff)
    else:
        s.send(buff)

s.close()
