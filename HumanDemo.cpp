/*
  Bullet Continuous Collision Detection and Physics Library Copyright (c) 2007 Erwin Coumans
  Motor Demo

  This software is provided 'as-is', without any express or implied warranty.
  In no event will the authors be held liable for any damages arising from the use of this software.
  Permission is granted to anyone to use this software for any purpose, 
  including commercial applications, and to alter it and redistribute it freely, 
  subject to the following restrictions:

  1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
  2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
  3. This notice may not be removed or altered from any source distribution.
*/

#include <iostream>

#include <vector>

#include "btBulletDynamicsCommon.h"

#include "LinearMath/btIDebugDraw.h"
#include "HumanDemo.h"


#include "LinearMath/btAlignedObjectArray.h"
class btBroadphaseInterface;
class btCollisionShape;
class btOverlappingPairCache;
class btCollisionDispatcher;
class btConstraintSolver;
struct btCollisionAlgorithmCreateFunc;
class btDefaultCollisionConfiguration;

#include "../CommonInterfaces/CommonRigidBodyBase.h"

class HumanDemo : public CommonRigidBodyBase
{
    float m_Time;
    float m_fCyclePeriod; // in milliseconds
    float m_fMuscleStrength;
	
    btAlignedObjectArray<class HumanRig*> m_rigs;
	
	
public:
    HumanDemo(struct GUIHelperInterface* helper)
	:CommonRigidBodyBase(helper)
    {
    }

    void initPhysics();
	
    void exitPhysics();
	
    virtual ~HumanDemo()
    {
    }
	
    void spawnHumanRig(const btVector3& startOffset, bool bFixed);
	
    //	virtual void keyboardCallback(unsigned char key, int x, int y);
	
    void setMotorTargets(btScalar deltaTime);
	
    void resetCamera()
    {
	float dist = 11;
	float pitch = 52;
	float yaw = 35;
	float targetPos[3]={0,0.46,0};
	m_guiHelper->resetCamera(dist,pitch,yaw,targetPos[0],targetPos[1],targetPos[2]);
    }
};


#ifndef M_PI
#define M_PI       3.14159265358979323846
#endif

#ifndef M_PI_2
#define M_PI_2     1.57079632679489661923
#endif

#ifndef M_PI_4
#define M_PI_4     0.785398163397448309616
#endif

#ifndef M_PI_8
#define M_PI_8     0.5 * M_PI_4
#endif


// /LOCAL FUNCTIONS



#define NUM_LEGS 6
#define BODYPART_COUNT 2 * NUM_LEGS + 1
#define JOINT_COUNT BODYPART_COUNT - 1

class HumanRig
{
    btDynamicsWorld*	m_ownerWorld;
    std::vector<btCollisionShape*>   m_shapes;
    std::vector<btRigidBody*>        m_bodies;
    std::vector<btTypedConstraint*>  m_joints;

    btRigidBody* localCreateRigidBody (btScalar mass, const btTransform& startTransform, btCollisionShape* shape)
    {
	bool isDynamic = (mass != 0.f);

	btVector3 localInertia(0,0,0);
	if (isDynamic)
	    shape->calculateLocalInertia(mass,localInertia);

	btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);
	btRigidBody::btRigidBodyConstructionInfo rbInfo(mass,myMotionState,shape,localInertia);
	btRigidBody* body = new btRigidBody(rbInfo);

	m_ownerWorld->addRigidBody(body);

	return body;
    }


public:
    HumanRig (btDynamicsWorld* ownerWorld, const btVector3& positionOffset, bool bFixed)
	: m_ownerWorld (ownerWorld)
    {
	btVector3 vUp(0, 1, 0);

	//
	// Setup geometry
	//
	float fBodySize  = 0.25f;
	float fLegLength = 0.45f;
	float fForeLegLength = 0.75f;
	float armLength = 0.277128;
	btCapsuleShape *torsoShape = new btCapsuleShape(btScalar(0.07), btScalar(0.14));
	btSphereShape *headShape = new btSphereShape(btScalar(0.09));
	btCapsuleShape *upWaistShape = new btCapsuleShape(btScalar(0.06), btScalar(0.12));
	btCapsuleShape *loWaistShape = new btCapsuleShape(btScalar(0.06), btScalar(0.12));
	btCapsuleShape *buttShape = new btCapsuleShape(btScalar(0.09), btScalar(0.14));
	btCapsuleShape *rThighShape = new btCapsuleShape(btScalar(0.06), btScalar(0.34));
	btCapsuleShape *rShinShape = new btCapsuleShape(btScalar(0.049), btScalar(0.3));
	btSphereShape *rFootShape = new btSphereShape(btScalar(0.075));
	btCapsuleShape *lThighShape = new btCapsuleShape(btScalar(0.06), btScalar(0.34));
	btCapsuleShape *lShinShape = new btCapsuleShape(btScalar(0.049), btScalar(0.3));
	btSphereShape *lFootShape = new btSphereShape(btScalar(0.075));
	btCapsuleShape *rUpArmShape = new btCapsuleShape(btScalar(0.04), btScalar(armLength));
	btCapsuleShape *rLoArmShape = new btCapsuleShape(btScalar(0.04), btScalar(armLength));
	btSphereShape *rHandShape = new btSphereShape(btScalar(0.04));
	btCapsuleShape *lUpArmShape = new btCapsuleShape(btScalar(0.04), btScalar(armLength));
	btCapsuleShape *lLoArmShape = new btCapsuleShape(btScalar(0.04), btScalar(armLength));
	btSphereShape *lHandShape = new btSphereShape(btScalar(0.04));
	
	m_shapes.push_back(torsoShape);
	m_shapes.push_back(headShape);
	m_shapes.push_back(upWaistShape);
	m_shapes.push_back(loWaistShape);
	m_shapes.push_back(buttShape);
	m_shapes.push_back(rThighShape);
	m_shapes.push_back(rShinShape);
	m_shapes.push_back(rFootShape);
	
	/*int i;
	for ( i=0; i<NUM_LEGS; i++)
	    {
		m_shapes[1 + 2*i] = new btCapsuleShape(btScalar(0.10), btScalar(fLegLength));
		m_shapes[2 + 2*i] = new btCapsuleShape(btScalar(0.08), btScalar(fForeLegLength));
		}*/

	//
	// Setup rigid bodies
	//
	float fHeight = 1.4;
	btTransform offset; offset.setIdentity();
	offset.setOrigin(positionOffset);		

	// root
	btVector3 vAxis = btVector3(btScalar(1.0), btScalar(0.0), btScalar(0.0));
	btVector3 vRoot = btVector3(btScalar(0.0), btScalar(1.4), btScalar(0.0));
	btTransform transform;
	transform.setIdentity();
	transform.setOrigin(vRoot);
	transform.setRotation(btQuaternion(vAxis, M_PI_2));

	btVector3 vLWaist = vRoot + btVector3(btScalar(-0.01), btScalar(-0.26), btScalar(0.0));
	btVector3 vPelvis = vLWaist + btVector3(btScalar(0.0), btScalar(-0.165), btScalar(0.0));
	btVector3 vRThigh = vPelvis + btVector3(btScalar(0.0), btScalar(-0.04), btScalar(-0.1));
	btVector3 vRShin = vRThigh + btVector3(btScalar(0.0), btScalar(-0.403), btScalar(0.01));
	btVector3 vRFoot = vRShin + btVector3(btScalar(0.0), btScalar(-0.45), btScalar(0.0));
	btVector3 vLThigh = vPelvis + btVector3(btScalar(0.0), btScalar(-0.04), btScalar(0.1));
	btVector3 vLShin = vLThigh + btVector3(btScalar(0.0), btScalar(-0.403), btScalar(-0.01));
	btVector3 vLFoot = vLShin + btVector3(btScalar(0.0), btScalar(-0.45), btScalar(0.0));
	btVector3 vRUpArm = vRoot + btVector3(btScalar(0.0), btScalar(0.06), btScalar(-0.17));
	btVector3 vRLoArm = vRUpArm + btVector3(btScalar(0.0), btScalar(0.0), btScalar(-0.311769));
	btVector3 vRHand = vRLoArm + btVector3(btScalar(0.0), btScalar(0.0), btScalar(-0.311769));
	btVector3 vLUpArm = vRoot + btVector3(btScalar(0.0), btScalar(0.06), btScalar(0.17));
	btVector3 vLLoArm = vLUpArm + btVector3(btScalar(0.0), btScalar(0.0), btScalar(0.311769));
	btVector3 vLHand = vLLoArm + btVector3(btScalar(0.0), btScalar(0.0), btScalar(0.311769));

	// Torso
	btRigidBody *torsoBody;
	if (bFixed) {
	    torsoBody = localCreateRigidBody(btScalar(0.0), offset*transform, torsoShape);
	}
	else {
	    torsoBody = localCreateRigidBody(btScalar(1.0), offset*transform, torsoShape);
	}
	m_bodies.push_back(torsoBody);

	// Head
	btTransform headTransform;
	headTransform.setIdentity();
	btVector3 vHeadOrigin = vRoot + btVector3(btScalar(0.0), btScalar(0.19), btScalar(0.0));
	headTransform.setOrigin(vHeadOrigin);
	btRigidBody *headBody = localCreateRigidBody(btScalar(1.0), offset*headTransform, headShape);
	m_bodies.push_back(headBody);
	btTransform neckPivot;
	neckPivot.setIdentity();
	neckPivot.setOrigin(vRoot);
	neckPivot = offset * neckPivot;
	btTransform neckHeadFrame = headBody->getWorldTransform().inverse() * neckPivot;
	btTransform neckTorsoFrame = torsoBody->getWorldTransform().inverse() * neckPivot;
	btTypedConstraint *neckConstraint = new btFixedConstraint(*torsoBody, *headBody, neckTorsoFrame, neckHeadFrame);
	m_ownerWorld->addConstraint(neckConstraint, true);
	m_joints.push_back(neckConstraint);

	// Upper Waist
	btTransform upWaistTransform;
	upWaistTransform.setIdentity();
	btVector3 vUpWaistOrigin = vRoot + btVector3(btScalar(-0.01), btScalar(-0.12), btScalar(0.0));
	upWaistTransform.setOrigin(vUpWaistOrigin);
	upWaistTransform.setRotation(btQuaternion(vAxis, M_PI_2));
	btRigidBody *upWaistBody = localCreateRigidBody(btScalar(1.0), offset*upWaistTransform, upWaistShape);
	m_bodies.push_back(upWaistBody);
	btTransform chestPivot;
	chestPivot.setIdentity();
	chestPivot.setOrigin(vRoot);
	chestPivot = offset * chestPivot;
	btTransform chestTorsoFrame = torsoBody->getWorldTransform().inverse() * chestPivot;
	btTransform chestWaistFrame = upWaistBody->getWorldTransform().inverse() * chestPivot;
	btTypedConstraint *chestConstraint = new btFixedConstraint(*torsoBody, *upWaistBody, chestTorsoFrame, chestWaistFrame);
	m_ownerWorld->addConstraint(chestConstraint, true);
	m_joints.push_back(chestConstraint);

	// Lower Waist
	btTransform loWaistTransform;
	loWaistTransform.setIdentity();
	btVector3 vLoWaistOrigin = vLWaist + btVector3(btScalar(0.0), btScalar(0.0), btScalar(0.0));
	loWaistTransform.setOrigin(vLoWaistOrigin);
	loWaistTransform.setRotation(btQuaternion(vAxis, M_PI_2));
	btRigidBody *loWaistBody = localCreateRigidBody(btScalar(1.0), offset*loWaistTransform, loWaistShape);
	m_bodies.push_back(loWaistBody);
	btTransform loWaistPivot;
	loWaistPivot.setIdentity();
	loWaistPivot.setOrigin(vLWaist);
	loWaistPivot = offset * loWaistPivot;
	btTransform loWaistUpFrame = upWaistBody->getWorldTransform().inverse() * loWaistPivot;
	btTransform loWaistLoFrame = loWaistBody->getWorldTransform().inverse() * loWaistPivot;
	btConeTwistConstraint *loWaistConstraint = new btConeTwistConstraint(*upWaistBody, *loWaistBody, loWaistUpFrame, loWaistLoFrame);
	loWaistConstraint->setLimit(btScalar(0.0), btScalar(M_PI_4), btScalar(M_PI_4));
	m_ownerWorld->addConstraint(loWaistConstraint, true);
	m_joints.push_back(loWaistConstraint);

	// Butt
	btTransform buttTransform;
	buttTransform.setIdentity();
	btVector3 vButtOrigin = vPelvis + btVector3(btScalar(-0.02), btScalar(0.0), btScalar(0.0));
	buttTransform.setOrigin(vButtOrigin);
	buttTransform.setRotation(btQuaternion(vAxis, M_PI_2));
	btRigidBody *buttBody = localCreateRigidBody(btScalar(1.0), offset*buttTransform, buttShape);
	m_bodies.push_back(buttBody);
	btVector3 pelvisPivot = offset * vPelvis;
	btVector3 pelvisWaistFrame = loWaistBody->getWorldTransform().inverse() * pelvisPivot;
	btVector3 pelvisButtFrame = buttBody->getWorldTransform().inverse() * pelvisPivot;
	btVector3 pelvisAxis = btVector3(btScalar(1.0), btScalar(0.0), btScalar(0.0));
	btHingeConstraint *pelvisConstraint = new btHingeConstraint(*loWaistBody, *buttBody, pelvisWaistFrame, pelvisButtFrame, pelvisAxis, pelvisAxis);
	pelvisConstraint->setLimit(btScalar(-0.610865), btScalar(0.610865));
	m_ownerWorld->addConstraint(pelvisConstraint, true);
	m_joints.push_back(pelvisConstraint);

	// Right Thigh
	btTransform rThighTransform;
	rThighTransform.setIdentity();
	btVector3 vRThighOrigin = vRThigh + btVector3(btScalar(0.0), btScalar(-0.175), btScalar(0.0));
	rThighTransform.setOrigin(vRThighOrigin);
	btRigidBody *rThighBody = localCreateRigidBody(btScalar(1.0), offset*rThighTransform, rThighShape);
	m_bodies.push_back(rThighBody);
	btTransform rHipPivot;
	rHipPivot.setIdentity();
	rHipPivot.setOrigin(vRThigh);
	rHipPivot = offset * rHipPivot;
	btTransform rHipThighFrame = rThighBody->getWorldTransform().inverse() * rHipPivot;
	btTransform rHipButtFrame = buttBody->getWorldTransform().inverse() * rHipPivot;
	btConeTwistConstraint *rHipConstraint = new btConeTwistConstraint(*buttBody, *rThighBody, rHipButtFrame, rHipThighFrame);
	rHipConstraint->setLimit(btScalar(M_PI_2), btScalar(M_PI_2), btScalar(0.0));
	m_ownerWorld->addConstraint(rHipConstraint, true);
	m_joints.push_back(rHipConstraint);

	// Right Shin
	btTransform rShinTransform;
	rShinTransform.setIdentity();
	btVector3 vRShinOrigin = vRShin + btVector3(btScalar(0.0), btScalar(-0.15), btScalar(0.0));
	rShinTransform.setOrigin(vRShinOrigin);
	btRigidBody *rShinBody = localCreateRigidBody(btScalar(1.0), offset*rShinTransform, rShinShape);
	m_bodies.push_back(rShinBody);
	btVector3 rKneePivot = offset * vRShin;
	btVector3 rKneeThighFrame = rThighBody->getWorldTransform().inverse() * rKneePivot;
	btVector3 rKneeShinFrame = rShinBody->getWorldTransform().inverse() * rKneePivot;
	btVector3 rKneeAxis = btVector3(btScalar(0.0), btScalar(0.0), btScalar(1.0));
	btHingeConstraint *rKneeConstraint = new btHingeConstraint(*rThighBody, *rShinBody, rKneeThighFrame, rKneeShinFrame, rKneeAxis, rKneeAxis);
	rKneeConstraint->setLimit(btScalar(-0.05), btScalar(M_PI_2));
	m_ownerWorld->addConstraint(rKneeConstraint, true);
	m_joints.push_back(rKneeConstraint);

	// Right Foot
	btTransform rFootTransform;
	rFootTransform.setIdentity();
	btVector3 vRFootOrigin = vRFoot + btVector3(btScalar(0.0), btScalar(0.1), btScalar(0.0));
	rFootTransform.setOrigin(vRFootOrigin);
	btRigidBody *rFootBody = localCreateRigidBody(btScalar(1.0), offset*rFootTransform, rFootShape);
	m_bodies.push_back(rFootBody);
	btTransform rAnklePivot;
	rAnklePivot.setIdentity();
	rAnklePivot.setOrigin(vRFoot);
	rAnklePivot = offset * rAnklePivot;
	btTransform rAnkleShinFrame = rShinBody->getWorldTransform().inverse() * rAnklePivot;
	btTransform rAnkleFootFrame = rFootBody->getWorldTransform().inverse() * rAnklePivot;
	btTypedConstraint *rAnkleConstraint = new btFixedConstraint(*rShinBody, *rFootBody, rAnkleShinFrame, rAnkleFootFrame);
	m_ownerWorld->addConstraint(rAnkleConstraint, true);
	m_joints.push_back(rAnkleConstraint);

	// Left Thigh
	btTransform lThighTransform;
	lThighTransform.setIdentity();
	btVector3 vLThighOrigin = vLThigh + btVector3(btScalar(0.0), btScalar(-0.175), btScalar(0.0));
	lThighTransform.setOrigin(vLThighOrigin);
	btRigidBody *lThighBody = localCreateRigidBody(btScalar(1.0), offset*lThighTransform, lThighShape);
	m_bodies.push_back(lThighBody);
	btTransform lHipPivot;
	lHipPivot.setIdentity();
	lHipPivot.setOrigin(vLThigh);
	lHipPivot = offset * lHipPivot;
	btTransform lHipThighFrame = lThighBody->getWorldTransform().inverse() * lHipPivot;
	btTransform lHipButtFrame = buttBody->getWorldTransform().inverse() * lHipPivot;
	btConeTwistConstraint *lHipConstraint = new btConeTwistConstraint(*buttBody, *lThighBody, lHipButtFrame, lHipThighFrame);
	lHipConstraint->setLimit(btScalar(M_PI_2), btScalar(M_PI_2), btScalar(0.0));
	m_ownerWorld->addConstraint(lHipConstraint, true);
	m_joints.push_back(lHipConstraint);

	// Left Shin
	btTransform lShinTransform;
	lShinTransform.setIdentity();
	btVector3 vLShinOrigin = vLShin + btVector3(btScalar(0.0), btScalar(-0.15), btScalar(0.0));
	lShinTransform.setOrigin(vLShinOrigin);
	btRigidBody *lShinBody = localCreateRigidBody(btScalar(1.0), offset*lShinTransform, lShinShape);
	m_bodies.push_back(lShinBody);
	btVector3 lKneePivot = offset * vLShin;
	btVector3 lKneeThighFrame = lThighBody->getWorldTransform().inverse() * lKneePivot;
	btVector3 lKneeShinFrame = lShinBody->getWorldTransform().inverse() * lKneePivot;
	btVector3 lKneeAxis = btVector3(btScalar(0.0), btScalar(0.0), btScalar(1.0));
	btHingeConstraint *lKneeConstraint = new btHingeConstraint(*lThighBody, *lShinBody, lKneeThighFrame, lKneeShinFrame, lKneeAxis, lKneeAxis);
	lKneeConstraint->setLimit(btScalar(-0.05), btScalar(M_PI_2));
	m_ownerWorld->addConstraint(lKneeConstraint, true);
	m_joints.push_back(lKneeConstraint);

	// Left Foot
	btTransform lFootTransform;
	lFootTransform.setIdentity();
	btVector3 vLFootOrigin = vLFoot + btVector3(btScalar(0.0), btScalar(0.1), btScalar(0.0));
	lFootTransform.setOrigin(vLFootOrigin);
	btRigidBody *lFootBody = localCreateRigidBody(btScalar(1.0), offset*lFootTransform, lFootShape);
	m_bodies.push_back(lFootBody);
	btTransform lAnklePivot;
	lAnklePivot.setIdentity();
	lAnklePivot.setOrigin(vLFoot);
	lAnklePivot = offset * lAnklePivot;
	btTransform lAnkleShinFrame = lShinBody->getWorldTransform().inverse() * lAnklePivot;
	btTransform lAnkleFootFrame = lFootBody->getWorldTransform().inverse() * lAnklePivot;
	btTypedConstraint *lAnkleConstraint = new btFixedConstraint(*lShinBody, *lFootBody, lAnkleShinFrame, lAnkleFootFrame);
	m_ownerWorld->addConstraint(lAnkleConstraint, true);
	m_joints.push_back(lAnkleConstraint);

	// Right Upper Arm
	btTransform rUpArmTransform;
	rUpArmTransform.setIdentity();
	btVector3 vRUpArmOrigin = vRUpArm + btVector3(btScalar(0.0), btScalar(0.0), btScalar(-armLength / 2));
	rUpArmTransform.setOrigin(vRUpArmOrigin);
	rUpArmTransform.setRotation(btQuaternion(vAxis, M_PI_2));
	btRigidBody *rUpArmBody = localCreateRigidBody(btScalar(1.0), offset*rUpArmTransform, rUpArmShape);
	m_bodies.push_back(rUpArmBody);
	btTransform rShldrPivot;
	rShldrPivot.setIdentity();
	rShldrPivot.setOrigin(vRUpArm);
	rShldrPivot = offset * rShldrPivot;
	btTransform rShldrArmFrame = rUpArmBody->getWorldTransform().inverse() * rShldrPivot;
	btTransform rShldrWaistFrame = upWaistBody->getWorldTransform().inverse() * rShldrPivot;
	btConeTwistConstraint *rShldrConstraint = new btConeTwistConstraint(*upWaistBody, *rUpArmBody, rShldrWaistFrame, rShldrArmFrame);
	rShldrConstraint->setLimit(btScalar(M_PI), btScalar(M_PI), btScalar(M_PI));
	m_ownerWorld->addConstraint(rShldrConstraint, true);
	m_joints.push_back(rShldrConstraint);

	// Right Lower Arm
	btTransform rLoArmTransform;
	rLoArmTransform.setIdentity();
	btVector3 vRLoArmOrigin = vRLoArm + btVector3(btScalar(0.0), btScalar(0.0), btScalar(-armLength / 2));
	rLoArmTransform.setOrigin(vRLoArmOrigin);
	rLoArmTransform.setRotation(btQuaternion(vAxis, M_PI_2));
	btRigidBody *rLoArmBody = localCreateRigidBody(btScalar(1.0), offset*rLoArmTransform, rLoArmShape);
	m_bodies.push_back(rLoArmBody);
	btVector3 rElbowAxis = btVector3(btScalar(0.0), btScalar(0.0), btScalar(1.0));
	btVector3 rElbowPivot = offset * vRLoArm;
	btVector3 rElbowUpFrame = rUpArmBody->getWorldTransform().inverse() * rElbowPivot;
	btVector3 rElbowLoFrame = rLoArmBody->getWorldTransform().inverse() * rElbowPivot;
	btHingeConstraint *rElbowConstraint = new btHingeConstraint(*rUpArmBody, *rLoArmBody, rElbowUpFrame, rElbowLoFrame, rElbowAxis, rElbowAxis);
	rElbowConstraint->setLimit(btScalar(-M_PI_2), btScalar(0.05));
	m_ownerWorld->addConstraint(rElbowConstraint, true);
	m_joints.push_back(rElbowConstraint);

	// Right Hand
	btTransform rHandTransform;
	rHandTransform.setIdentity();
	btVector3 vRHandOrigin = vRHand + btVector3(btScalar(0.0), btScalar(0.0), btScalar(0.0));
	rHandTransform.setOrigin(vRHandOrigin);
	btRigidBody *rHandBody = localCreateRigidBody(btScalar(1.0), offset*rHandTransform, rHandShape);
	m_bodies.push_back(rHandBody);
	btTransform rWristPivot;
	rWristPivot.setIdentity();
	rWristPivot.setOrigin(vRHand);
	rWristPivot = offset * rWristPivot;
	btTransform rWristArmFrame = rLoArmBody->getWorldTransform().inverse() * rWristPivot;
	btTransform rWristHandFrame = rHandBody->getWorldTransform().inverse() * rWristPivot;
	btTypedConstraint *rWristConstraint = new btFixedConstraint(*rLoArmBody, *rHandBody, rWristArmFrame, rWristHandFrame);
	m_ownerWorld->addConstraint(rWristConstraint, true);
        m_joints.push_back(rWristConstraint);

	// Left Upper Arm
	btTransform lUpArmTransform;
	lUpArmTransform.setIdentity();
	btVector3 vLUpArmOrigin = vLUpArm + btVector3(btScalar(0.0), btScalar(0.0), btScalar(armLength / 2));
	lUpArmTransform.setOrigin(vLUpArmOrigin);
	lUpArmTransform.setRotation(btQuaternion(vAxis, M_PI_2));
	btRigidBody *lUpArmBody = localCreateRigidBody(btScalar(1.0), offset*lUpArmTransform, lUpArmShape);
	m_bodies.push_back(lUpArmBody);
	btTransform lShldrPivot;
	lShldrPivot.setIdentity();
	lShldrPivot.setOrigin(vLUpArm);
	lShldrPivot = offset * lShldrPivot;
	btTransform lShldrArmFrame = lUpArmBody->getWorldTransform().inverse() * lShldrPivot;
	btTransform lShldrWaistFrame = upWaistBody->getWorldTransform().inverse() * lShldrPivot;
	btConeTwistConstraint *lShldrConstraint = new btConeTwistConstraint(*upWaistBody, *lUpArmBody, lShldrWaistFrame, lShldrArmFrame);
	lShldrConstraint->setLimit(btScalar(M_PI), btScalar(M_PI), btScalar(M_PI));
	m_ownerWorld->addConstraint(lShldrConstraint, true);
	m_joints.push_back(lShldrConstraint);

	// Left Lower Arm
	btTransform lLoArmTransform;
	lLoArmTransform.setIdentity();
	btVector3 vLLoArmOrigin = vLLoArm + btVector3(btScalar(0.0), btScalar(0.0), btScalar(armLength / 2));
	lLoArmTransform.setOrigin(vLLoArmOrigin);
	lLoArmTransform.setRotation(btQuaternion(vAxis, M_PI_2));
	btRigidBody *lLoArmBody = localCreateRigidBody(btScalar(1.0), offset*lLoArmTransform, lLoArmShape);
	m_bodies.push_back(lLoArmBody);
	btVector3 lElbowAxis = btVector3(btScalar(0.0), btScalar(0.0), btScalar(-1.0));
	btVector3 lElbowPivot = offset * vLLoArm;
	btVector3 lElbowUpFrame = lUpArmBody->getWorldTransform().inverse() * lElbowPivot;
	btVector3 lElbowLoFrame = lLoArmBody->getWorldTransform().inverse() * lElbowPivot;
	btHingeConstraint *lElbowConstraint = new btHingeConstraint(*lUpArmBody, *lLoArmBody, lElbowUpFrame, lElbowLoFrame, lElbowAxis, lElbowAxis);
	lElbowConstraint->setLimit(btScalar(-M_PI_2), btScalar(0.05));
	m_ownerWorld->addConstraint(lElbowConstraint, true);
	m_joints.push_back(lElbowConstraint);

	// Left Hand
	btTransform lHandTransform;
	lHandTransform.setIdentity();
	btVector3 vLHandOrigin = vLHand + btVector3(btScalar(0.0), btScalar(0.0), btScalar(0.0));
	lHandTransform.setOrigin(vLHandOrigin);
	btRigidBody *lHandBody = localCreateRigidBody(btScalar(1.0), offset*lHandTransform, lHandShape);
	m_bodies.push_back(lHandBody);
	btTransform lWristPivot;
	lWristPivot.setIdentity();
	lWristPivot.setOrigin(vLHand);
	lWristPivot = offset * lWristPivot;
	btTransform lWristArmFrame = lLoArmBody->getWorldTransform().inverse() * lWristPivot;
	btTransform lWristHandFrame = lHandBody->getWorldTransform().inverse() * lWristPivot;
	btTypedConstraint *lWristConstraint = new btFixedConstraint(*lLoArmBody, *lHandBody, lWristArmFrame, lWristHandFrame);
	m_ownerWorld->addConstraint(lWristConstraint, true);
        m_joints.push_back(lWristConstraint);

	/*// legs
	for ( i=0; i<NUM_LEGS; i++)
	    {
		float fAngle = 2 * M_PI * i / NUM_LEGS;
		float fSin = sin(fAngle);
		float fCos = cos(fAngle);

		transform.setIdentity();
		btVector3 vBoneOrigin = btVector3(btScalar(fCos*(fBodySize+0.5*fLegLength)), btScalar(fHeight), btScalar(fSin*(fBodySize+0.5*fLegLength)));
		transform.setOrigin(vBoneOrigin);

		// thigh
		btVector3 vToBone = (vBoneOrigin - vRoot).normalize();
		btVector3 vAxis = vToBone.cross(vUp);			
		transform.setRotation(btQuaternion(vAxis, M_PI_2));
		m_bodies[1+2*i] = localCreateRigidBody(btScalar(1.), offset*transform, m_shapes[1+2*i]);

		// shin
		transform.setIdentity();
		transform.setOrigin(btVector3(btScalar(fCos*(fBodySize+fLegLength)), btScalar(fHeight-0.5*fForeLegLength), btScalar(fSin*(fBodySize+fLegLength))));
		m_bodies[2+2*i] = localCreateRigidBody(btScalar(1.), offset*transform, m_shapes[2+2*i]);
	    }

	// Setup some damping on the m_bodies
	for (i = 0; i < BODYPART_COUNT; ++i)
	    {
		m_bodies[i]->setDamping(0.05, 0.85);
		m_bodies[i]->setDeactivationTime(0.8);
		//m_bodies[i]->setSleepingThresholds(1.6, 2.5);
		m_bodies[i]->setSleepingThresholds(0.5f, 0.5f);
	    }


	//
	// Setup the constraints
	//
	btHingeConstraint* hingeC;
	//btConeTwistConstraint* coneC;

	btTransform localA, localB, localC;
	*/
	/*for ( i=0; i<NUM_LEGS; i++)
	  {
	  float fAngle = 2 * M_PI * i / NUM_LEGS;
	  float fSin = sin(fAngle);
	  float fCos = cos(fAngle);

	  // hip joints
	  localA.setIdentity(); localB.setIdentity();
	  localA.getBasis().setEulerZYX(0,-fAngle,0);	localA.setOrigin(btVector3(btScalar(fCos*fBodySize), btScalar(0.), btScalar(fSin*fBodySize)));
	  localB = m_bodies[1+2*i]->getWorldTransform().inverse() * m_bodies[0]->getWorldTransform() * localA;
	  hingeC = new btHingeConstraint(*m_bodies[0], *m_bodies[1+2*i], localA, localB);
	  hingeC->setLimit(btScalar(-0.75 * M_PI_4), btScalar(M_PI_8));
	  //hingeC->setLimit(btScalar(-0.1), btScalar(0.1));
	  m_joints[2*i] = hingeC;
	  m_ownerWorld->addConstraint(m_joints[2*i], true);

	  // knee joints
	  localA.setIdentity(); localB.setIdentity(); localC.setIdentity();
	  localA.getBasis().setEulerZYX(0,-fAngle,0);	localA.setOrigin(btVector3(btScalar(fCos*(fBodySize+fLegLength)), btScalar(0.), btScalar(fSin*(fBodySize+fLegLength))));
	  localB = m_bodies[1+2*i]->getWorldTransform().inverse() * m_bodies[0]->getWorldTransform() * localA;
	  localC = m_bodies[2+2*i]->getWorldTransform().inverse() * m_bodies[0]->getWorldTransform() * localA;
	  hingeC = new btHingeConstraint(*m_bodies[1+2*i], *m_bodies[2+2*i], localB, localC);
	  //hingeC->setLimit(btScalar(-0.01), btScalar(0.01));
	  hingeC->setLimit(btScalar(-M_PI_8), btScalar(0.2));
	  m_joints[1+2*i] = hingeC;
	  m_ownerWorld->addConstraint(m_joints[1+2*i], true);
	  }*/
    }

    virtual	~HumanRig ()
    {
	int i;

	// Remove all constraints
	for ( i = 0; i < m_joints.size(); ++i)
	    {
		m_ownerWorld->removeConstraint(m_joints[i]);
		delete m_joints[i]; m_joints[i] = 0;
	    }

	// Remove all bodies and shapes
	for ( i = 0; i < m_bodies.size(); ++i)
	    {
		m_ownerWorld->removeRigidBody(m_bodies[i]);
			
		delete m_bodies[i]->getMotionState();

		delete m_bodies[i]; m_bodies[i] = 0;
		delete m_shapes[i]; m_shapes[i] = 0;
	    }
    }

    btTypedConstraint** GetJoints() {return &m_joints[0];}

};



void humanPreTickCallback (btDynamicsWorld *world, btScalar timeStep)
{
    HumanDemo* humanDemo = (HumanDemo*)world->getWorldUserInfo();

    humanDemo->setMotorTargets(timeStep);
	
}



void HumanDemo::initPhysics()
{
    m_guiHelper->setUpAxis(1);

    // Setup the basic world

    m_Time = 0;
    m_fCyclePeriod = 2000.f; // in milliseconds

    //	m_fMuscleStrength = 0.05f;
    // new SIMD solver for joints clips accumulated impulse, so the new limits for the motor
    // should be (numberOfsolverIterations * oldLimits)
    // currently solver uses 10 iterations, so:
    m_fMuscleStrength = 0.5f;


    m_collisionConfiguration = new btDefaultCollisionConfiguration();

    m_dispatcher = new btCollisionDispatcher(m_collisionConfiguration);

    btVector3 worldAabbMin(-10000,-10000,-10000);
    btVector3 worldAabbMax(10000,10000,10000);
    m_broadphase = new btAxisSweep3 (worldAabbMin, worldAabbMax);

    m_solver = new btSequentialImpulseConstraintSolver;

    m_dynamicsWorld = new btDiscreteDynamicsWorld(m_dispatcher,m_broadphase,m_solver,m_collisionConfiguration);

    m_dynamicsWorld->setInternalTickCallback(humanPreTickCallback,this,true);
    m_guiHelper->createPhysicsDebugDrawer(m_dynamicsWorld);
	

    // Setup a big ground box
    {
	btCollisionShape* groundShape = new btBoxShape(btVector3(btScalar(200.),btScalar(10.),btScalar(200.)));
	m_collisionShapes.push_back(groundShape);
	btTransform groundTransform;
	groundTransform.setIdentity();
	groundTransform.setOrigin(btVector3(0,-10,0));
	createRigidBody(btScalar(0.),groundTransform,groundShape);
    }

    // Spawn one ragdoll
    btVector3 startOffset(1,0.5,0);
    spawnHumanRig(startOffset, false);
    startOffset.setValue(-2,0.5,0);
    spawnHumanRig(startOffset, true);

    m_guiHelper->autogenerateGraphicsObjects(m_dynamicsWorld);
}


void HumanDemo::spawnHumanRig(const btVector3& startOffset, bool bFixed)
{
    HumanRig* rig = new HumanRig(m_dynamicsWorld, startOffset, bFixed);
    m_rigs.push_back(rig);
}

void	HumanPreStep()
{

}




void HumanDemo::setMotorTargets(btScalar deltaTime)
{

    float ms = deltaTime*1000000.;
    float minFPS = 1000000.f/60.f;
    if (ms > minFPS)
	ms = minFPS;

    m_Time += ms;

    //
    // set per-frame sinusoidal position targets using angular motor (hacky?)
    //	
    for (int r=0; r<m_rigs.size(); r++)
	{
	    for (int i=0; i<2*NUM_LEGS; i++)
		{
		    /*btHingeConstraint* hingeC = static_cast<btHingeConstraint*>(m_rigs[r]->GetJoints()[i]);
		    btScalar fCurAngle      = hingeC->getHingeAngle();
			
		    btScalar fTargetPercent = (int(m_Time / 1000) % int(m_fCyclePeriod)) / m_fCyclePeriod;
		    btScalar fTargetAngle   = 0.5 * (1 + sin(2 * M_PI * fTargetPercent));
		    btScalar fTargetLimitAngle = hingeC->getLowerLimit() + fTargetAngle * (hingeC->getUpperLimit() - hingeC->getLowerLimit());
		    btScalar fAngleError  = fTargetLimitAngle - fCurAngle;
		    btScalar fDesiredAngularVel = 1000000.f * fAngleError/ms;
		    hingeC->enableAngularMotor(true, fDesiredAngularVel, m_fMuscleStrength);*/
		}
	}

	
}

#if 0
void HumanDemo::keyboardCallback(unsigned char key, int x, int y)
{
    switch (key)
	{
	case '+': case '=':
	    m_fCyclePeriod /= 1.1f;
	    if (m_fCyclePeriod < 1.f)
		m_fCyclePeriod = 1.f;
	    break;
	case '-': case '_':
	    m_fCyclePeriod *= 1.1f;
	    break;
	case '[':
	    m_fMuscleStrength /= 1.1f;
	    break;
	case ']':
	    m_fMuscleStrength *= 1.1f;
	    break;
	default:
	    DemoApplication::keyboardCallback(key, x, y);
	}	
}
#endif



void HumanDemo::exitPhysics()
{

    int i;

    for (i=0;i<m_rigs.size();i++)
	{
	    HumanRig* rig = m_rigs[i];
	    delete rig;
	}

    //cleanup in the reverse order of creation/initialization

    //remove the rigidbodies from the dynamics world and delete them
	
    for (i=m_dynamicsWorld->getNumCollisionObjects()-1; i>=0 ;i--)
	{
	    btCollisionObject* obj = m_dynamicsWorld->getCollisionObjectArray()[i];
	    btRigidBody* body = btRigidBody::upcast(obj);
	    if (body && body->getMotionState())
		{
		    delete body->getMotionState();
		}
	    m_dynamicsWorld->removeCollisionObject( obj );
	    delete obj;
	}

    //delete collision shapes
    for (int j=0;j<m_collisionShapes.size();j++)
	{
	    btCollisionShape* shape = m_collisionShapes[j];
	    delete shape;
	}

    //delete dynamics world
    delete m_dynamicsWorld;

    //delete solver
    delete m_solver;

    //delete broadphase
    delete m_broadphase;

    //delete dispatcher
    delete m_dispatcher;

    delete m_collisionConfiguration;	
}


class CommonExampleInterface*    HumanControlCreateFunc(struct CommonExampleOptions& options)
{
    return new HumanDemo(options.m_guiHelper);
}
