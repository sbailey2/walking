import numpy as np

np.random.seed(1337) # for reproducibility

import theano
import theano.tensor as T

import numpy as np
import theano as th
import theano.tensor as T
from keras.utils import np_utils
import keras.models as models
from keras.layers import Input,merge
from keras.layers.core import Reshape,Dense,Dropout,Activation,Flatten,MaxoutDense
from keras.layers.advanced_activations import LeakyReLU
from keras.activations import *
from keras.layers.wrappers import TimeDistributed
from keras.layers.noise import GaussianNoise
from keras.layers.convolutional import Convolution2D, MaxPooling2D, ZeroPadding2D, UpSampling2D
from keras.layers.recurrent import LSTM
from keras.regularizers import *
from keras.layers.normalization import *
from keras.optimizers import *
import matplotlib.pyplot as plt
import seaborn as sns
from keras.models import Model
#from IPython import display
from keras.utils import np_utils
import scipy.io as sio

# Load the walking data
fileName='/home/sbailey/Documents/bullet3-2.83.7/examples/DynamicControlDemo/MocapData.mat'
data=sio.loadmat(fileName)['data'][0]
maxLength = 4000
#Xseq = np.zeros((len(data),maxLength,len(data[0][0])))
#for frame, x in zip(data, Xseq):
#    l = min(maxLength, len(frame))
#    x[:l] = np.asarray(frame[:l])
X = np.concatenate([np.asarray(frame) for frame in data],0)
usedDim = np.ones(X.shape[1]).astype('bool')
usedDim[39:45] = False
rootMean = np.mean(X[:,39:45],0)
X = X[:,usedDim]
print(X.shape)

# Build the generative  model
zShape = 100
yShape = X.shape[1]
h = 256
g_input = Input(shape=[zShape])
H = Dense(h)(g_input)
H = BatchNormalization(mode=2)(H)
H = Activation('tanh')(H)
H = Dense(h)(H)
H = BatchNormalization(mode=2)(H)
H = Activation('tanh')(H)
g = Dense(yShape)(H)
generator = Model(g_input, g)
generator.compile(loss='MSE', optimizer=Adam())
generator.summary()

# Build the discriminative model
d_input = Input(shape=[yShape])
H = Dense(2*h)(d_input)
H = Activation('tanh')(H)
H = Dense(2*h)(H)
H = Activation('tanh')(H)
H = Dense(2*h)(H)
H = Activation('tanh')(H)
H = Dense(1)(H)
d = Activation('sigmoid')(H)
discriminator = Model(d_input, d)
discriminator.compile(loss='binary_crossentropy', optimizer=Adam())
discriminator.summary()

# Freeze weights in the discriminator for stacked training
def make_trainable(net, val):
    net.trainable = val
    for l in net.layers:
        l.trainable = val
make_trainable(discriminator, False)

# Build the stacked GAN model
gan_input = Input(shape=[zShape])
H = generator(gan_input)
gan = discriminator(H)
GAN = Model(gan_input, gan)
GAN.compile(loss='binary_crossentropy', optimizer=Adam())
GAN.summary()

# Set up training loop
make_trainable(discriminator, True)
def train(nb_epoch = 5000, log_every=100, batch_size=32):

    for t in range(nb_epoch):

        # Generate the poses
        pose_batch = X[np.random.randint(0,X.shape[0],size=batch_size)]
        noise_gen = np.random.uniform(-1,1,size=[batch_size,zShape])
        generated_poses = generator.predict(noise_gen)

        # Train the discriminator on generated poses
        x = np.concatenate([pose_batch, generated_poses],0)
        y = np.zeros([2*batch_size,1])
        y[:batch_size] = 1

        d_loss = discriminator.train_on_batch(x,y)
        noise_train = np.random.uniform(-1,1,size=[batch_size,zShape])
        y = np.ones([batch_size,1])
        g_loss = GAN.train_on_batch(noise_train, y)

        if t%log_every == 0:
            print(str(t)+'- dLoss: '+str(d_loss)+' gLoss: '+str(g_loss))

def generate(N):
    noise_gen = np.random.uniform(-1,1,size=[N,zShape])
    y = generator.predict(noise_gen)
    out = np.zeros((N,y.shape[1]+6))
    out[:,:39] = y[:,:39]
    out[:,45:] = y[:,39:]
    out[:,39:45] = rootMean
    return out

train()
