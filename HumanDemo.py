import socket
import sys
import struct
import time

import numpy as np

def degrees(rad):
    return rad * 180 / np.pi

def quat2equatorial(q):
      """
      Determine Right Ascension, Declination, and Roll for the object quaternion
      
      :returns: RA, Dec, Roll
      :rtype: numpy array [ra,dec,roll]
      """
      
      q = q
      q2 = q**2

      ## calculate direction cosine matrix elements from $quaternions
      xa = q2[0] - q2[1] - q2[2] + q2[3] 
      xb = 2 * (q[0] * q[1] + q[2] * q[3]) 
      xn = 2 * (q[0] * q[2] - q[1] * q[3]) 
      yn = 2 * (q[1] * q[2] + q[0] * q[3]) 
      zn = q2[3] + q2[2] - q2[0] - q2[1] 

      ##; calculate RA, Dec, Roll from cosine matrix elements
      ra   = degrees(np.arctan2(xb , xa)) ;
      dec  = degrees(np.arctan2(xn , np.sqrt(1 - xn**2)));
      roll = degrees(np.arctan2(yn , zn)) ;
      #if ( ra < 0 ):
      #   ra += 360
      #if ( roll < 0 ):
      #   roll += 360

      return np.array([ra, dec, roll])


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
        #c = 1.0

        # Fill in the data that we have
        s = list(state)
        #s.insert(0,0.0)
        #s.insert(2,0.0)
        #data['root'] = [s[0],s[1],s[2],-c*s[3],-c*s[5],-c*s[4]]
        #data['lowerback'] = [c*s[6],-c*s[7],c*s[8]-90]
        data['lhumerus'] = [c*s[19],c*s[18],c*s[17]]
        data['lradius'] = [-c*s[20]]
        data['rhumerus'] = [c*s[23],c*s[22],c*s[21]]
        data['rradius'] = [-c*s[24]]
        data['rfemur'] = [c*s[13],c*s[14],c*s[15]]
        data['rtibia'] = [c*s[16]]
        data['lfemur'] = [c*s[9],c*s[10],c*s[11]]
        data['ltibia'] = [c*s[12]]

        #data['lhumerus'] = [c*s[16],c*s[15],c*s[14]]
        #data['lradius'] = [-c*s[17]]
        #data['rhumerus'] = [c*s[20],c*s[19],c*s[18]]
        #data['rradius'] = [-c*s[21]]

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
        #q = np.asarray([state[9],state[10],state[11],np.sqrt(1-state[9]**2-state[10]**2-state[11]**2)])
        #r = quat2equatorial(q)
        #if iteration % 10 == 0:
        #    print('YPR: '+str(r[0])+' '+str(r[1])+' '+str(r[2]))

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
