env = Environment() # Initialize the environment

# Include directories
env.Append(CPPPATH=['/usr/local/include/bullet'])
env.Append(CCFLAGS='-g')

# Library directories
env.Append(LIBPATH=['/usr/local/lib/'])
env.Append(LIBS=['BulletDynamics','BulletCollision','LinearMath'])

source = []
source.append('HumanDemoFast.cpp')
source.append('HumanDemo.cpp')
env.Program(target = 'HumanDemoNoGUI', source = source)