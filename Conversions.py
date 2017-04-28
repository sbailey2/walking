import numpy as np
import transforms3d.euler as euler
import matplotlib.pyplot as plt

# Compute all of the joint transformations
rfemurTransform = np.asarray(euler.euler2mat(0,0,-20*np.pi/180))
rtibiaTransform = np.asarray(euler.euler2mat(0,0,0))
lfemurTransform = np.asarray(euler.euler2mat(0,0,20*np.pi/180))
ltibiaTransform = np.asarray(euler.euler2mat(0,0,0))
rhumerusTransform = np.asarray(euler.euler2mat(180*np.pi/180,30*np.pi/180,90*np.pi/180))
rradiusTransform = np.asarray(euler.euler2mat(0,0,0))
lhumerusTransform = np.asarray(euler.euler2mat(180*np.pi/180,-30*np.pi/180,-90*np.pi/180))
lradiusTransform = np.asarray(euler.euler2mat(0,0,0))

# Compute all of the joint offsets
rfemurOffset = np.asarray(euler.euler2mat(0,0,20*np.pi/180))
rtibiaOffset = np.asarray(euler.euler2mat(0,0,0))
lfemurOffset = np.asarray(euler.euler2mat(0,0,-20*np.pi/180))
ltibiaOffset = np.asarray(euler.euler2mat(0,0,0))
rhumerusOffset = np.asarray(euler.euler2mat(-30*np.pi/180,0,0))
rradiusOffset = np.asarray(euler.euler2mat(0,0,0))
lhumerusOffset = np.asarray(euler.euler2mat(-30*np.pi/180,0,0))
lradiusOffset = np.asarray(euler.euler2mat(0,0,0))

# Set the default local orientation of the joints
lowWaist = np.asarray((0,-0.26,0))
pelvis = np.asarray((0,-0.165,0))
rThigh = np.asarray((-0.1,-0.04,0))
rShin = np.asarray((0,-0.403,0.01))
rFoot = np.asarray((0,-0.45,0))
lThigh = np.asarray((0.1,-0.04,0))
lShin = np.asarray((0,-0.403,-0.01))
lFoot = np.asarray((0,-0.45,0))
rUpArm = np.asarray((-0.17,0.06,0))
rLoArm = np.asarray((-0.311769,0,0))
rHand = np.asarray((-0.311769,0,0))
lUpArm = np.asarray((0.17,0.06,0))
lLoArm = np.asarray((0.311769,0,0))
lHand = np.asarray((0.311769,0,0))

# This converts a mocap pose described in rotation to a pose described by
# the 3D positions of the skeleton joints
def rot2pos(pose):

    # First, convert all euler angles into rotation matrices
    #pose = pose*np.pi/180
    rfemur = np.asarray(euler.euler2mat(pose[0],pose[1],pose[2]))
    rtibia = np.asarray(euler.euler2mat(pose[3],0,0))
    lfemur = np.asarray(euler.euler2mat(pose[4],pose[5],pose[6]))
    ltibia = np.asarray(euler.euler2mat(pose[7],0,0))
    rhumerus = np.asarray(euler.euler2mat(pose[8],pose[9],pose[10]))
    rradius = np.asarray(euler.euler2mat(0,pose[11],0))
    lhumerus = np.asarray(euler.euler2mat(pose[12],pose[13],pose[14]))
    lradius = np.asarray(euler.euler2mat(0,pose[15],0))

    # Change the basis to local coordinates
    rfemur = rfemurTransform.dot(rfemur.dot(np.linalg.inv(rfemurTransform).dot(np.linalg.inv(rfemurOffset))))
    rtibia = rtibiaTransform.dot(rtibia.dot(np.linalg.inv(rtibiaTransform).dot(np.linalg.inv(rtibiaOffset))))
    lfemur = lfemurTransform.dot(lfemur.dot(np.linalg.inv(lfemurTransform).dot(np.linalg.inv(lfemurOffset))))
    ltibia = ltibiaTransform.dot(ltibia.dot(np.linalg.inv(ltibiaTransform).dot(np.linalg.inv(ltibiaOffset))))
    rhumerus = rhumerusTransform.dot(rhumerus.dot(np.linalg.inv(rhumerusTransform).dot(np.linalg.inv(rhumerusOffset))))
    rradius = rradiusTransform.dot(rradius.dot(np.linalg.inv(rradiusTransform).dot(np.linalg.inv(rradiusOffset))))
    lhumerus = lhumerusTransform.dot(lhumerus.dot(np.linalg.inv(lhumerusTransform).dot(np.linalg.inv(lhumerusOffset))))
    lradius = lradiusTransform.dot(lradius.dot(np.linalg.inv(lradiusTransform).dot(np.linalg.inv(lradiusOffset))))

    # Compute the global offsets
    rfemurPos = rfemur.dot(rShin) + lowWaist + pelvis + rThigh
    rtibiaPos = rfemur.dot(rtibia.dot(rFoot)) + rfemurPos
    lfemurPos = lfemur.dot(lShin) +  lowWaist + pelvis + lThigh
    ltibiaPos = lfemur.dot(ltibia.dot(lFoot)) + lfemurPos
    rhumerusPos = rhumerus.dot(rLoArm) + rUpArm
    rradiusPos = rhumerus.dot(rradius.dot(rHand)) + rhumerusPos
    lhumerusPos = lhumerus.dot(lLoArm) + lUpArm
    lradiusPos = lhumerus.dot(lradius.dot(lHand)) + lhumerusPos

    # Combine everything together and return the results
    result = np.stack([rfemurPos,rtibiaPos,lfemurPos,ltibiaPos,rhumerusPos,rradiusPos,lhumerusPos,lradiusPos])
    return result
    
def plotPose(pose,x=0,y=1,color='b'):
    #plt.close()

    root = np.asarray((0,0,0))
    pelvisPos = lowWaist + pelvis
    rThighPos = pelvisPos + rThigh
    lThighPos = pelvisPos + lThigh
    plt.plot([root[x],lowWaist[x]],[root[y],lowWaist[y]],'-'+color+'.',linewidth=3,markersize=10)
    plt.plot([pelvisPos[x],lowWaist[x]],[pelvisPos[y],lowWaist[y]],'-'+color+'.',linewidth=3,markersize=10)
    plt.plot([pelvisPos[x],rThighPos[x]],[pelvisPos[y],rThighPos[y]],'-'+color+'.',linewidth=3,markersize=10)
    plt.plot([pose[0,x],rThighPos[x]],[pose[0,y],rThighPos[y]],'-'+color+'.',linewidth=3,markersize=10)
    plt.plot([pose[0,x],pose[1,x]],[pose[0,y],pose[1,y]],'-'+color+'.',linewidth=3,markersize=10)
    plt.plot([pelvisPos[x],lThighPos[x]],[pelvisPos[y],lThighPos[y]],'-'+color+'.',linewidth=3,markersize=10)
    plt.plot([pose[2,x],lThighPos[x]],[pose[2,y],lThighPos[y]],'-'+color+'.',linewidth=3,markersize=10)
    plt.plot([pose[2,x],pose[3,x]],[pose[2,y],pose[3,y]],'-'+color+'.',linewidth=3,markersize=10)
    plt.plot([root[x],rUpArm[x]],[root[y],rUpArm[y]],'-'+color+'.',linewidth=3,markersize=10)
    plt.plot([pose[4,x],rUpArm[x]],[pose[4,y],rUpArm[y]],'-'+color+'.',linewidth=3,markersize=10)
    plt.plot([pose[4,x],pose[5,x]],[pose[4,y],pose[5,y]],'-'+color+'.',linewidth=3,markersize=10)
    plt.plot([root[x],lUpArm[x]],[root[y],lUpArm[y]],'-'+color+'.',linewidth=3,markersize=10)
    plt.plot([pose[6,x],lUpArm[x]],[pose[6,y],lUpArm[y]],'-'+color+'.',linewidth=3,markersize=10)
    plt.plot([pose[6,x],pose[7,x]],[pose[6,y],pose[7,y]],'-'+color+'.',linewidth=3,markersize=10)
    plt.axis('equal')
    #plt.show()
