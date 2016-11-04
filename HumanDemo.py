import socket
import sys
import struct
import time

import numpy as np

def convertToAMC(states):

    # Set up the skeleton information
    joints = [('root',6),('lowerback',3),('upperback',3),('thorax',3),('lowerneck',3),('upperneck',3),('head',3),('rclavicle',2),('rhumerus',3),('rradius',1),('rwrist',1),('rhand',2),('rfingers',1),('rthumb',2),('lclavicle',2),('lhumerus',3),('lradius',1),('lwrist',1),('lhand',2),('lfingers',1),('lthumb',2),('rfemur',3),('rtibia',1),('rfoot',2),('rtoes',1),('lfemur',3),('ltibia',1),('lfoot',2),('ltoes',1)]
    jointNames = [j[0] for j in joints]
    dofs = [j[1] for j in joints]
    frames = []

    # Write each frame
    for state,frame in zip(states,range(len(states))):

        # Set up initial zero values
        data = {}
        for j in joints:
            data[j[0]] = [0.0]*j[1]
        c=180.0/np.pi

        # Fill in the data that we have
        s = list(state)
        #s.insert(0,0.0)
        #s.insert(2,0.0)
        #data['root'] = [s[0],s[1],s[2],c*s[3],c*s[5],c*s[4]]
        #data['lowerback'] = [c*s[6],-c*s[7],c*s[8]-90]
        data['rhumerus'] = [c*s[17]-30,c*s[18],c*s[19]-90]
        data['rradius'] = [-c*s[20]]
        data['lhumerus'] = [c*s[21]-30,c*s[22],c*s[23]-90]
        data['lradius'] = [-c*s[24]]
        data['rfemur'] = [-c*s[9],c*s[10],-c*s[11]+15]
        data['rtibia'] = [c*s[12]]
        data['lfemur'] = [-c*s[13],c*s[14],-c*s[15]-15]
        data['ltibia'] = [c*s[16]]

        frames.append(data)

    # Write the AMC file
    amc = ''
    amc += '#!OML:ASF \n'
    amc += ':FULLY-SPECIFIED\n'
    amc += ':DEGREES\n'
    for f,i in zip(frames,range(len(frames))):
        amc += str(i+1)+'\n'
        for j in f:
            amc += j
            for a in f[j]:
                amc += ' '+str(a)
            amc += '\n'

    return amc

HOST, PORT = 'localhost', 47138

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

s.connect((HOST, PORT))
print('Connected to host')

s.send(b'Hello!')
iteration = 0
#while True:
states = []
amc = None
try:
    while True:

#for i in range(200):
        stateSize = s.recv(4);
        stateSize = struct.unpack('i',stateSize)[0]
        state = s.recv(1000)
        while len(state) < stateSize:
            state += s.recv(1000)
        state = np.asarray([struct.unpack('f',state[i:i+4])[0] for i in range(0,len(state),4)])
        #if i % 20 == 0:
        #    print('State:')
        #    print(state)
        c = 0*np.random.uniform(-0.1,0.1,15)
        buff = struct.pack('%sf' % len(c), *c)
        iteration += 1

        if amc is None:
            states.append(state)

        if iteration % 200000 == 0:
            print('Iteration '+str(iteration))
            print(state.shape)
            print(state)
            s.send(b'RESET')
            zeros = np.zeros(20).astype(int)
            buff = struct.pack('%si' % len(zeros), *zeros)
            s.send(buff)
        else:
            s.send(buff)
except KeyboardInterrupt:
    pass
s.close()
        
amc = convertToAMC(states)
fileName='simTest.amc'
with open(fileName,'w') as file:
    file.write(amc)
