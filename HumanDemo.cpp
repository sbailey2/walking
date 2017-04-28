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
#include <iomanip>

#include <vector>

#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <errno.h>

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

class JointEvaluator
{
private:
    std::string name;
    
public:
    JointEvaluator(const std::string &n) : name(n) {}
    virtual ~JointEvaluator() {}
    virtual void evaluate(std::vector<btScalar> &r) = 0;
    std::string getName() {return name;}
};

class Joint3DOF : public JointEvaluator
{
private:
    btGeneric6DofSpring2Constraint *constraint;
    btTransform axisTransform;
    btTransform axisOffset;
    
public:
    Joint3DOF(const std::string &n, btGeneric6DofSpring2Constraint *c,
	      double ax, double ay, double az, const btTransform &ao) :
	JointEvaluator(n),
	constraint(c),
	axisOffset(ao)
    {
	axisTransform.setIdentity();
	btQuaternion x = btQuaternion(0.0,ax*M_PI/180,0.0);
	btQuaternion y = btQuaternion(ay*M_PI/180,0.0,0.0);
	btQuaternion z = btQuaternion(0.0,0.0,az*M_PI/180);
	btQuaternion trans = z*y*x;
	axisTransform.setRotation(trans);
    }
    ~Joint3DOF() {}
    
    virtual void evaluate(std::vector<btScalar> &r)
    {
	btTransform gtA = constraint->getCalculatedTransformA();
	btTransform gtB = constraint->getCalculatedTransformB();
	btTransform local = axisTransform.inverse() * gtA.inverse() * gtB * axisOffset * axisTransform;
	btMatrix3x3 m = local.getBasis();
	btScalar rx, ry, rz;
	m.getEulerZYX(rz, ry, rx);
	rx *= 180 / M_PI;
	ry *= 180 / M_PI;
	rz *= 180 / M_PI;
	
	r.clear();
	r.push_back(rx);
	r.push_back(ry);
	r.push_back(rz);
    }
};

class Joint1DOF : public JointEvaluator
{
private:
    btGeneric6DofSpring2Constraint *constraint;
    int index;
    double scale;
    
public:
    Joint1DOF(const std::string &n, btGeneric6DofSpring2Constraint *c, int i, double s=1.0) :
	JointEvaluator(n),
	constraint(c),
        index(i),
        scale(s)
    {}
    ~Joint1DOF() {}
    
    virtual void evaluate(std::vector<btScalar> &r)
    {
	r.clear();
        r.push_back(scale*constraint->getAngle(index) * 180 / M_PI);
    }
};

class JointOrientation : public JointEvaluator
{
private:
    btRigidBody *body;
    
public:
    JointOrientation(const std::string &n, btRigidBody *b) :
	JointEvaluator(n),
        body(b)
    {}
    ~JointOrientation() {}
    
    virtual void evaluate(std::vector<btScalar> &r)
    {
	btQuaternion rot = body->getOrientation();
	btTransform t;
	t.setIdentity();
	t.setRotation(rot);
	btMatrix3x3 m = t.getBasis();

	btScalar rx, ry, rz;
	m.getEulerZYX(rz, ry, rx);
	rx *= 180 / M_PI;
	ry *= 180 / M_PI;
	rz *= 180 / M_PI;
	btVector3 com = body->getCenterOfMassPosition();
	
	r.clear();
	r.push_back(10*com[0]);
	r.push_back(10*com[1]);
	r.push_back(10*com[2]);
	r.push_back(rx);
	r.push_back(ry);
	r.push_back(rz);
    }
};

class JointBlank : public JointEvaluator
{
private:
    int size;
    
public:
    JointBlank(const std::string &n, int s) :
	JointEvaluator(n),
	size(s)
    {}
    ~JointBlank() {}
    
    virtual void evaluate(std::vector<btScalar> &r)
    {
	r.clear();
	for (int i = 0; i < size; ++i) {
	    r.push_back(0.0);
	}
    }
};

class HumanDemo : public CommonRigidBodyBase
{
    float m_Time;
    float m_fCyclePeriod; // in milliseconds
    float m_fMuscleStrength;
    bool m_fullState;

    int serverSocket;
    int clientSocket;
    int receiveSocket;
	
    btAlignedObjectArray<class HumanRig*> m_rigs;
	
	
public:
    HumanDemo(struct GUIHelperInterface* helper)
	:CommonRigidBodyBase(helper),
	 m_fullState(false)
    {
	serverSocket = socket(AF_INET, SOCK_STREAM, 0);
	sockaddr_in serverAddr;
	serverAddr.sin_family = AF_INET;
	serverAddr.sin_port = 8888;
	serverAddr.sin_addr.s_addr = INADDR_ANY;
	if (bind(serverSocket, (struct sockaddr*)&serverAddr, sizeof(struct sockaddr)) != 0) {
	    std::cout << "Error binding socket\n";
	    exit(errno);
	}

	if (listen(serverSocket, 1) != 0) {
	    std::cout << "Error listening on the socket\n";
	    exit(errno);
	}
	sockaddr_in clientAddr;
	socklen_t sin_size=sizeof(struct sockaddr_in);
	std::cout << "Waiting for connection...\n";
	clientSocket = accept(serverSocket, (struct sockaddr*)&clientAddr, &sin_size);

	if (listen(serverSocket, 1) != 0) {
	    std::cout << "Error listening on the socket\n";
	    exit(errno);
	}

	std::cout << "Waiting for connection...\n";
	receiveSocket = accept(serverSocket, (struct sockaddr*)&clientAddr, &sin_size);

	// Receive a message
	//char buffer[1000];
	//bzero(buffer, 1000);
	//int n = read(clientSocket, buffer, 500);
	//std::cout << "Received " << buffer << "\n";
    }

    void initPhysics();
	
    void exitPhysics();
	
    virtual ~HumanDemo()
    {
	close(clientSocket);
	close(serverSocket);
    }
	
    void spawnHumanRig(const btVector3& startOffset, bool bFixed);
	
    //	virtual void keyboardCallback(unsigned char key, int x, int y);
	
    void setMotorTargets(btScalar deltaTime);

    virtual void preStepAction(float deltaTime);
    virtual void postStepAction(float deltaTime);
	
    void resetCamera()
    {
	float dist = 11;
	float pitch = 52;
	float yaw = 35;
	float targetPos[3]={0,0.46,0};
	if (m_guiHelper != 0) {
	    m_guiHelper->resetCamera(dist,pitch,yaw,targetPos[0],targetPos[1],targetPos[2]);
	}
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
public:
    btDynamicsWorld*	m_ownerWorld;
    std::vector<btCollisionShape*>   m_shapes;
    std::vector<btRigidBody*>        m_bodies;
    std::vector<btTypedConstraint*>  m_joints;
    std::vector<JointEvaluator*>     m_evaluators;
    std::vector<int>                 m_parents;
    std::vector<std::pair<btVector3, int> > m_controls;

    typedef std::pair<btGeneric6DofSpring2Constraint*, int> JointControl;
    std::vector<JointControl> m_jointControls;
    

    btRigidBody* localCreateRigidBody (btScalar mass, const btTransform& startTransform, btCollisionShape* shape)
    {
	bool isDynamic = (mass != 0.f);

	btVector3 localInertia(0,0,0);
	if (isDynamic)
	    shape->calculateLocalInertia(mass,localInertia);

	btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);
	btRigidBody::btRigidBodyConstructionInfo rbInfo(mass,myMotionState,shape,localInertia);
	rbInfo.m_friction = btScalar(0.8);
	rbInfo.m_linearDamping = btScalar(0.9);
	rbInfo.m_angularDamping = btScalar(0.9);
	
	btRigidBody* body = new btRigidBody(rbInfo);
	//std::cout << "Angular damping: " << body->getAngularDamping() << "\n";

	m_ownerWorld->addRigidBody(body);

	return body;
    }
    
    HumanRig (btDynamicsWorld* ownerWorld, const btVector3& positionOffset, bool bFixed)
	: m_ownerWorld (ownerWorld)
    {
	btVector3 vUp(0, 1, 0);
	btScalar damping(5.0);
	btScalar stiffness(5.0);

	// Set up the world and stuffs
	m_ownerWorld->getSolverInfo().m_solverMode = SOLVER_USE_WARMSTARTING | SOLVER_SIMD | SOLVER_USE_2_FRICTION_DIRECTIONS | SOLVER_ENABLE_FRICTION_DIRECTION_CACHING;
	m_ownerWorld->getSolverInfo().m_warmstartingFactor = 1.0;
	m_ownerWorld->getSolverInfo().m_minimumSolverBatchSize = 1;

	//Quality/stability
	m_ownerWorld->getSolverInfo().m_tau = 1.0;  //mass factor
	m_ownerWorld->getSolverInfo().m_erp = 0.5;  //constraint error reduction in one step
	m_ownerWorld->getSolverInfo().m_erp2 = 1.0; //constraint error reduction in one step for split impulse
	m_ownerWorld->getSolverInfo().m_numIterations = 300; //number of constraint iterations
	m_ownerWorld->getSolverInfo().m_sor = 1.0; //not used
	m_ownerWorld->getSolverInfo().m_maxErrorReduction = 0; //not used
   
	//Collision
	m_ownerWorld->getSolverInfo().m_splitImpulse = true; //avoid adding energy to the system
	m_ownerWorld->getSolverInfo().m_splitImpulsePenetrationThreshold = -0.000001; //low value needed for accurate friction
	//m_ownerWorld->getSolverInfo().m_splitImpulsePenetrationThreshold = -0.1; //low value needed for accurate friction
	m_ownerWorld->getSolverInfo().m_splitImpulseTurnErp = 1.0; //error reduction for rigid body angular velocity
	m_ownerWorld->getDispatchInfo().m_useContinuous = false;
	m_ownerWorld->getDispatchInfo().m_allowedCcdPenetration = -0.001;
	//m_ownerWorld->setApplySpeculativeContactRestitution(true);
	m_ownerWorld->getSolverInfo().m_restingContactRestitutionThreshold = 1e30; //not used
   
	//Special forces
	m_ownerWorld->getSolverInfo().m_maxGyroscopicForce = 1e30; //gyroscopic effect
   
	//Unrealistic components
	//m_ownerWorld->getSolverInfo().m_globalCfm = 0.0; //global constraint force mixing factor
	//m_ownerWorld->getSolverInfo().m_damping = 0.25; //global damping
	m_ownerWorld->getSolverInfo().m_friction = 1.0; //global friction
	m_ownerWorld->getSolverInfo().m_singleAxisRollingFrictionThreshold = 1e30; //single axis rolling velocity threshold
	m_ownerWorld->getSolverInfo().m_linearSlop = 0.0; //position bias

	//
	// Setup geometry
	//
	float fBodySize  = 0.25f;
	float fLegLength = 0.45f;
	float fForeLegLength = 0.75f;
	float armLength = 0.277128;
	//btCapsuleShape *torsoShape = new btCapsuleShapeX(btScalar(0.07), btScalar(0.14));
	btCapsuleShape *torsoShape = new btCapsuleShapeX(btScalar(0.07), btScalar(0.07));
	btSphereShape *headShape = new btSphereShape(btScalar(0.09));
	//btCapsuleShape *upWaistShape = new btCapsuleShapeX(btScalar(0.06), btScalar(0.12));
	//btCapsuleShape *loWaistShape = new btCapsuleShapeX(btScalar(0.06), btScalar(0.12));
	btCapsuleShape *upWaistShape = new btCapsuleShapeX(btScalar(0.06), btScalar(0.06));
	btCapsuleShape *loWaistShape = new btCapsuleShapeX(btScalar(0.06), btScalar(0.06));
	//btCapsuleShape *buttShape = new btCapsuleShapeX(btScalar(0.09), btScalar(0.14));
	btCapsuleShape *buttShape = new btCapsuleShapeX(btScalar(0.09), btScalar(0.09));
	btCapsuleShape *rThighShape = new btCapsuleShape(btScalar(0.06), btScalar(0.34));
	btCapsuleShape *rShinShape = new btCapsuleShape(btScalar(0.049), btScalar(0.3));
	btSphereShape *rFootShape = new btSphereShape(btScalar(0.075));
	//btBoxShape *rFootShape = new btBoxShape(btVector3(0.075,0.075,0.075));
	btCapsuleShape *lThighShape = new btCapsuleShape(btScalar(0.06), btScalar(0.34));
	btCapsuleShape *lShinShape = new btCapsuleShape(btScalar(0.049), btScalar(0.3));
	btSphereShape *lFootShape = new btSphereShape(btScalar(0.075));
	//btBoxShape *lFootShape = new btBoxShape(btVector3(0.075,0.075,0.075));
	//btCapsuleShape *rUpArmShape = new btCapsuleShapeX(btScalar(0.04), btScalar(armLength));
	//btCapsuleShape *rLoArmShape = new btCapsuleShapeX(btScalar(0.04), btScalar(armLength));
	btSphereShape *rHandShape = new btSphereShape(btScalar(0.04));
	//btCapsuleShape *lUpArmShape = new btCapsuleShapeX(btScalar(0.04), btScalar(armLength));
	//btCapsuleShape *lLoArmShape = new btCapsuleShapeX(btScalar(0.04), btScalar(armLength));
	btCapsuleShape *rUpArmShape = new btCapsuleShapeX(btScalar(0.035), btScalar(armLength));
	btCapsuleShape *rLoArmShape = new btCapsuleShapeX(btScalar(0.035), btScalar(armLength));
	btCapsuleShape *lUpArmShape = new btCapsuleShapeX(btScalar(0.035), btScalar(armLength));
	btCapsuleShape *lLoArmShape = new btCapsuleShapeX(btScalar(0.035), btScalar(armLength));
	btSphereShape *lHandShape = new btSphereShape(btScalar(0.04));
	
	m_shapes.push_back(torsoShape);
	m_shapes.push_back(headShape);
	m_shapes.push_back(upWaistShape);
	m_shapes.push_back(loWaistShape);
	m_shapes.push_back(buttShape);
	m_shapes.push_back(rThighShape);
	m_shapes.push_back(rShinShape);
	m_shapes.push_back(rFootShape);
	m_shapes.push_back(lThighShape);
	m_shapes.push_back(lShinShape);
	m_shapes.push_back(lFootShape);
	m_shapes.push_back(rUpArmShape);
	m_shapes.push_back(rLoArmShape);
	m_shapes.push_back(rHandShape);
	m_shapes.push_back(lUpArmShape);
	m_shapes.push_back(lLoArmShape);
	m_shapes.push_back(lHandShape);
	
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
	//offset.setRotation(btQuaternion(btVector3(0.0,1.0,0.0), -M_PI_4));
	//offset.setRotation(btQuaternion(btVector3(1.0,0.0,0.0), -M_PI_2));
	//offset.setRotation(btQuaternion(btVector3(0.0,0.0,1.0), -0.15));
	//offset.setRotation(btQuaternion(btVector3(0.0,0.0,1.0), -M_PI_2));
	offset.setOrigin(positionOffset);		

	// root
	btVector3 vAxis = btVector3(btScalar(1.0), btScalar(0.0), btScalar(0.0));
	btVector3 vRoot = btVector3(btScalar(0.0), btScalar(1.4), btScalar(0.0));
	btTransform transform;
	transform.setIdentity();
	transform.setOrigin(vRoot);
	//transform.setRotation(btQuaternion(vAxis, M_PI_2));

	btMatrix3x3 tempMat = offset.getBasis();
	//std::cout << "Root matrix:\n";
	for (int i = 0; i < 3; ++i) {
	    //std::cout << tempMat[i][0] << "\t" << tempMat[i][1] << "\t" << tempMat[i][2] << "\n";
	}
	btScalar y,p,r;
	tempMat.getEulerYPR(y,p,r);
	//std::cout << "YPR representation:\n";
	//std::cout << y << "\t" << p << "\t" << r << "\n";
	tempMat.getEulerZYX(y,p,r);
	//std::cout << "ZYX representation\n";
	//std::cout << y << "\t" << p << "\t" << r << "\n";

	btVector3 vLWaist = vRoot + btVector3(btScalar(0.0), btScalar(-0.26), btScalar(-0.01));
	btVector3 vPelvis = vLWaist + btVector3(btScalar(0.0), btScalar(-0.165), btScalar(0.0));
	btVector3 vRThigh = vPelvis + btVector3(btScalar(-0.1), btScalar(-0.04), btScalar(0.0));
	btVector3 vRShin = vRThigh + btVector3(btScalar(0.0), btScalar(-0.403), btScalar(0.01));
	btVector3 vRFoot = vRShin + btVector3(btScalar(0.0), btScalar(-0.45), btScalar(0.0));
	btVector3 vLThigh = vPelvis + btVector3(btScalar(0.1), btScalar(-0.04), btScalar(0.0));
	btVector3 vLShin = vLThigh + btVector3(btScalar(0.0), btScalar(-0.403), btScalar(-0.01));
	btVector3 vLFoot = vLShin + btVector3(btScalar(0.0), btScalar(-0.45), btScalar(0.0));
	btVector3 vRUpArm = vRoot + btVector3(btScalar(-0.17), btScalar(0.06), btScalar(0.0));
	btVector3 vRLoArm = vRUpArm + btVector3(btScalar(-0.311769), btScalar(0.0), btScalar(0.0));
	btVector3 vRHand = vRLoArm + btVector3(btScalar(-0.311769), btScalar(0.0), btScalar(0.0));
	btVector3 vLUpArm = vRoot + btVector3(btScalar(0.17), btScalar(0.06), btScalar(0.0));
	btVector3 vLLoArm = vLUpArm + btVector3(btScalar(0.311769), btScalar(0.0), btScalar(0.0));
	btVector3 vLHand = vLLoArm + btVector3(btScalar(0.311769), btScalar(0.0), btScalar(0.0));

	// Torso 0
	btRigidBody *torsoBody;
	if (bFixed) {
	    torsoBody = localCreateRigidBody(btScalar(0.0), offset*transform, torsoShape);
	}
	else {
	    torsoBody = localCreateRigidBody(btScalar(0.0), offset*transform, torsoShape);
	}
	m_bodies.push_back(torsoBody);
	m_parents.push_back(2);

	// Head 1
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
	m_joints.push_back(neckConstraint); // 0
	m_parents.push_back(0);

	// Upper Waist 2
	btTransform upWaistTransform;
	upWaistTransform.setIdentity();
	btVector3 vUpWaistOrigin = vRoot + btVector3(btScalar(0.0), btScalar(-0.12), btScalar(-0.01));
	upWaistTransform.setOrigin(vUpWaistOrigin);
	//upWaistTransform.setRotation(btQuaternion(vAxis, M_PI_2));
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
	m_joints.push_back(chestConstraint); // 1
	m_parents.push_back(3);

	// Lower Waist 3
	btTransform loWaistTransform;
	loWaistTransform.setIdentity();
	btVector3 vLoWaistOrigin = vLWaist + btVector3(btScalar(0.0), btScalar(0.0), btScalar(0.0));
	loWaistTransform.setOrigin(vLoWaistOrigin);
	//loWaistTransform.setRotation(btQuaternion(vAxis, M_PI_2));
	btRigidBody *loWaistBody = localCreateRigidBody(btScalar(1.0), offset*loWaistTransform, loWaistShape);
	m_bodies.push_back(loWaistBody);
	btTransform upWaistPivot;
	upWaistPivot.setIdentity();
	upWaistPivot.setOrigin(vLWaist);
	upWaistPivot = offset * upWaistPivot;
	btTransform upWaistUpFrame = upWaistBody->getWorldTransform().inverse() * upWaistPivot;
	btTransform upWaistLoFrame = loWaistBody->getWorldTransform().inverse() * upWaistPivot;
	btTypedConstraint *upWaistConstraint = new btFixedConstraint(*upWaistBody, *loWaistBody, upWaistUpFrame, upWaistLoFrame);
	//loWaistConstraint->setLimit(btScalar(0.0), btScalar(M_PI_4), btScalar(M_PI_4));
	m_ownerWorld->addConstraint(upWaistConstraint, true);
	m_joints.push_back(upWaistConstraint); // 2
	m_parents.push_back(4);
	//m_controls.push_back(std::pair<btVector3,int>(btVector3(btScalar(0.0), btScalar(1.0), btScalar(0.0)), 3)); // 0 ????
	//m_controls.push_back(std::pair<btVector3,int>(btVector3(btScalar(0.0), btScalar(0.0), btScalar(1.0)), 3)); // 1

	// Butt 4
	btTransform buttTransform;
	buttTransform.setIdentity();
	btVector3 vButtOrigin = vPelvis + btVector3(btScalar(0.0), btScalar(0.0), btScalar(-0.02));
	buttTransform.setOrigin(vButtOrigin);
	//buttTransform.setRotation(btQuaternion(vAxis, M_PI_2));
	//btRigidBody *buttBody = localCreateRigidBody(btScalar(5.322), offset*buttTransform, buttShape);
	btRigidBody *buttBody = localCreateRigidBody(btScalar(0.0), offset*buttTransform, buttShape);

	btTransform loWaistPivot;
	loWaistPivot.setIdentity();
	loWaistPivot.setOrigin(vLWaist);
	loWaistPivot = offset * loWaistPivot;
	btTransform loWaistButtFrame = buttBody->getWorldTransform().inverse() * loWaistPivot;
	btTransform loWaistLoFrame = loWaistBody->getWorldTransform().inverse() * loWaistPivot;
	//btConeTwistConstraint *loWaistConstraint = new btConeTwistConstraint(*buttBody, *loWaistBody, loWaistButtFrame, loWaistLoFrame);
	//btTypedConstraint *loWaistConstraint = new btFixedConstraint(*buttBody, *loWaistBody, loWaistButtFrame, loWaistLoFrame);
	//btVector3 loWaistLL(0.0, -M_PI_4/2, -M_PI_4/2);
	//btVector3 loWaistUL(0.0, M_PI_4/2, M_PI_4/2);
	btVector3 loWaistLL(0.0, 0.0, 0.0);
	btVector3 loWaistUL(0.0, 0.0, 0.0);
	btGeneric6DofSpring2Constraint *loWaistConstraint = new btGeneric6DofSpring2Constraint(*buttBody, *loWaistBody, loWaistButtFrame, loWaistLoFrame);
	loWaistConstraint->setLimit(btScalar(M_PI_4), btScalar(0), btScalar(M_PI_4));
	loWaistConstraint->setAngularLowerLimit(loWaistLL);
	loWaistConstraint->setAngularUpperLimit(loWaistUL);
	loWaistConstraint->enableSpring(3, true);
	loWaistConstraint->setDamping(3, 10*damping);
	loWaistConstraint->setStiffness(3, 10*stiffness);
	loWaistConstraint->enableSpring(4, true);
	loWaistConstraint->setDamping(4, 10*damping);
	loWaistConstraint->setStiffness(4, 10*stiffness);
	loWaistConstraint->enableSpring(5, true);
	loWaistConstraint->setDamping(5, 10*damping);
	loWaistConstraint->setStiffness(5, 10*stiffness);
	loWaistConstraint->setLimit(btScalar(0.0), btScalar(M_PI_4/2), btScalar(M_PI_4/2));
	loWaistConstraint->setLimit(btScalar(0.0), btScalar(0), btScalar(0));
	m_ownerWorld->addConstraint(loWaistConstraint, true);
	m_joints.push_back(loWaistConstraint); // 3
	//m_parents.push_back(4);
	//m_controls.push_back(std::pair<btVector3,int>(btVector3(btScalar(0.0), btScalar(1.0), btScalar(0.0)), 3)); // 0 ????
	//m_controls.push_back(std::pair<btVector3,int>(btVector3(btScalar(0.0), btScalar(0.0), btScalar(1.0)), 3)); // 1

	m_bodies.push_back(buttBody);
	//btVector3 pelvisPivot = offset * vPelvis;
	//btTransform pelvisPivot;
	//pelvisPivot.setIdentity();
	//pelvisPivot.setOrigin(vPelvis);
	//pelvisPivot = offset * pelvisPivot;
	////btVector3 pelvisWaistFrame = loWaistBody->getWorldTransform().inverse() * pelvisPivot;
	////btVector3 pelvisButtFrame = buttBody->getWorldTransform().inverse() * pelvisPivot;
	//btTransform pelvisWaistFrame = loWaistBody->getWorldTransform().inverse() * pelvisPivot;
	//btTransform pelvisButtFrame = buttBody->getWorldTransform().inverse() * pelvisPivot;
	////btVector3 pelvisAxis = btVector3(btScalar(1.0), btScalar(0.0), btScalar(0.0));
	////btHingeConstraint *pelvisConstraint = new btHingeConstraint(*loWaistBody, *buttBody, pelvisWaistFrame, pelvisButtFrame, pelvisAxis, pelvisAxis);
	//btTypedConstraint *pelvisConstraint = new btFixedConstraint(*loWaistBody, *buttBody, pelvisWaistFrame, pelvisButtFrame);
	////pelvisConstraint->setLimit(btScalar(-0.610865), btScalar(0.610865));
	//m_ownerWorld->addConstraint(pelvisConstraint, true);
	//m_joints.push_back(pelvisConstraint);
	m_parents.push_back(-1);
	//m_controls.push_back(std::pair<btVector3,int>(btVector3(btScalar(1.0), btScalar(0.0), btScalar(0.0)), 4)); // 2

	m_evaluators.push_back(new JointOrientation("root", buttBody));

	// Right Thigh 5
	btTransform rThighTransform;
	rThighTransform.setIdentity();
	btVector3 vRThighOrigin = vRThigh + btVector3(btScalar(0.0), btScalar(-0.175), btScalar(0.0));
	rThighTransform.setOrigin(vRThighOrigin);
	btRigidBody *rThighBody = localCreateRigidBody(btScalar(5.377), offset*rThighTransform, rThighShape);
	m_bodies.push_back(rThighBody);
	btTransform rHipPivot;
	rHipPivot.setIdentity();
	rHipPivot.setOrigin(vRThigh);
	rHipPivot = offset * rHipPivot;
	btTransform rHipOffset;
	rHipOffset.setIdentity();
	//rHipOffset.setRotation(btQuaternion(0, 0, -20.0 * M_PI / 180.0));
	btTransform rHipThighFrame = rThighBody->getWorldTransform().inverse() * rHipPivot * rHipOffset;
	btTransform rHipButtFrame = buttBody->getWorldTransform().inverse() * rHipPivot;
	//btConeTwistConstraint *rHipConstraint = new btConeTwistConstraint(*buttBody, *rThighBody, rHipButtFrame, rHipThighFrame);
	//btVector3 rHipLL(-M_PI_2, -M_PI_4, -M_PI_4);
	//btVector3 rHipUL(M_PI_4, M_PI_4, M_PI_4);
	btVector3 rHipLL(-M_PI_2, -M_PI_4, -M_PI_4);
	btVector3 rHipUL(M_PI_2, M_PI_4, M_PI_4);
	btGeneric6DofSpring2Constraint *rHipConstraint = new btGeneric6DofSpring2Constraint(*buttBody, *rThighBody, rHipButtFrame, rHipThighFrame);
	//rHipConstraint->setLimit(btScalar(M_PI_4), btScalar(0), btScalar(M_PI_4));
	rHipConstraint->setAngularLowerLimit(rHipLL);
	rHipConstraint->setAngularUpperLimit(rHipUL);
	rHipConstraint->enableSpring(3, true);
	rHipConstraint->setDamping(3, damping);
	rHipConstraint->setStiffness(3, stiffness);
	rHipConstraint->enableSpring(4, true);
	rHipConstraint->setDamping(4, damping);
	rHipConstraint->setStiffness(4, stiffness);
	rHipConstraint->enableSpring(5, true);
	rHipConstraint->setDamping(5, damping);
	rHipConstraint->setStiffness(5, stiffness);
	
	m_ownerWorld->addConstraint(rHipConstraint, true);
	m_joints.push_back(rHipConstraint); // 4
	m_parents.push_back(4);
	m_controls.push_back(std::pair<btVector3,int>(btVector3(btScalar(1.0), btScalar(0.0), btScalar(0.0)), 5)); // 2
	m_controls.push_back(std::pair<btVector3,int>(btVector3(btScalar(0.0), btScalar(0.0), btScalar(1.0)), 5)); // 3
	m_controls.push_back(std::pair<btVector3,int>(btVector3(btScalar(0.0), btScalar(1.0), btScalar(0.0)), 5)); // 4

	m_jointControls.push_back(JointControl(rHipConstraint,3));
	m_jointControls.push_back(JointControl(rHipConstraint,4));
	m_jointControls.push_back(JointControl(rHipConstraint,5));

	btTransform rThighAxisOffset;
	rThighAxisOffset.setIdentity();
	rThighAxisOffset.setRotation(btQuaternion(0,0,20*M_PI/180));
	m_evaluators.push_back(new Joint3DOF("rfemur", rHipConstraint, 0, 0, -20,
					     rThighAxisOffset));

	// Right Shin 6
	btTransform rShinTransform;
	rShinTransform.setIdentity();
	btVector3 vRShinOrigin = vRShin + btVector3(btScalar(0.0), btScalar(-0.15), btScalar(0.0));
	rShinTransform.setOrigin(vRShinOrigin);
	btRigidBody *rShinBody = localCreateRigidBody(btScalar(3.1), offset*rShinTransform, rShinShape);
	m_bodies.push_back(rShinBody);
	btTransform rKneePivot;
	rKneePivot.setIdentity();
	rKneePivot.setOrigin(vRShin);
	rKneePivot = offset * rKneePivot;
	//rShinTransform.setRotation(btQuaternion(btVector3(0,0,1), M_PI_4));
	rShinTransform.setOrigin(btVector3(0,0,0));
	btTransform rKneeThighFrame = rThighBody->getWorldTransform().inverse() * rKneePivot;
	btTransform rKneeShinFrame = rShinTransform * rShinBody->getWorldTransform().inverse() * rKneePivot;
	rKneeShinFrame.setOrigin(btVector3(0,0.3/2,0));
	//btHingeConstraint *rKneeConstraint = new btHingeConstraint(*rThighBody, *rShinBody, rKneeThighFrame, rKneeShinFrame, rKneeAxis, rKneeAxis);
	btVector3 rKneeLL(-M_PI_2, 0.0, 0.0);
	btVector3 rKneeUL(M_PI_4, 0.0, 0.0);
	btGeneric6DofSpring2Constraint *rKneeConstraint = new btGeneric6DofSpring2Constraint(*rThighBody, *rShinBody, rKneeThighFrame, rKneeShinFrame);
	rKneeConstraint->setAngularLowerLimit(rKneeLL);
	rKneeConstraint->setAngularUpperLimit(rKneeUL);
	rKneeConstraint->enableSpring(3, true);
	rKneeConstraint->setDamping(3, damping);
	rKneeConstraint->setStiffness(3, stiffness);
	rKneeConstraint->enableSpring(4, true);
	rKneeConstraint->setDamping(4, damping);
	rKneeConstraint->setStiffness(4, stiffness);
	rKneeConstraint->enableSpring(5, true);
	rKneeConstraint->setDamping(5, damping);
	rKneeConstraint->setStiffness(5, stiffness);
	//rKneeConstraint->setLimit(btScalar(-0.05), btScalar(M_PI_4));
	m_ownerWorld->addConstraint(rKneeConstraint, true);
	m_joints.push_back(rKneeConstraint);
	m_parents.push_back(5);
	m_controls.push_back(std::pair<btVector3,int>(btVector3(btScalar(-1.0), btScalar(0.0), btScalar(0.0)), 6)); // 5

	m_jointControls.push_back(JointControl(rKneeConstraint,3));

	m_evaluators.push_back(new Joint1DOF("rtibia", rKneeConstraint, 0, -1.0));

	// Right Foot 7
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
	m_parents.push_back(6);

	// Left Thigh 8
	btTransform lThighTransform;
	lThighTransform.setIdentity();
	btVector3 vLThighOrigin = vLThigh + btVector3(btScalar(0.0), btScalar(-0.175), btScalar(0.0));
	lThighTransform.setOrigin(vLThighOrigin);
	btRigidBody *lThighBody = localCreateRigidBody(btScalar(5.377), offset*lThighTransform, lThighShape);
	m_bodies.push_back(lThighBody);
	btTransform lHipPivot;
	lHipPivot.setIdentity();
	lHipPivot.setOrigin(vLThigh);
	lHipPivot = offset * lHipPivot;
	btTransform lHipThighFrame = lThighBody->getWorldTransform().inverse() * lHipPivot;
	btTransform lHipButtFrame = buttBody->getWorldTransform().inverse() * lHipPivot;
	//btVector3 lHipLL(-M_PI, -M_PI, -M_PI);
	//btVector3 lHipUL(M_PI, M_PI, M_PI);
	btVector3 lHipLL(-M_PI_2, -M_PI_4, -M_PI_4);
	btVector3 lHipUL(M_PI_2, M_PI_4, M_PI_4);
	btGeneric6DofSpring2Constraint *lHipConstraint = new btGeneric6DofSpring2Constraint(*buttBody, *lThighBody, lHipButtFrame, lHipThighFrame);
	lHipConstraint->setAngularLowerLimit(lHipLL);
	lHipConstraint->setAngularUpperLimit(lHipUL);
	lHipConstraint->enableSpring(3, true);
	lHipConstraint->setDamping(3, damping);
	lHipConstraint->setStiffness(3, stiffness);
	lHipConstraint->enableSpring(4, true);
	lHipConstraint->setDamping(4, damping);
	lHipConstraint->setStiffness(4, stiffness);
	lHipConstraint->enableSpring(5, true);
	lHipConstraint->setDamping(5, damping);
	lHipConstraint->setStiffness(5, stiffness);
	m_ownerWorld->addConstraint(lHipConstraint, true);
	m_joints.push_back(lHipConstraint);
	m_parents.push_back(4);
	m_controls.push_back(std::pair<btVector3,int>(btVector3(btScalar(-1.0), btScalar(0.0), btScalar(0.0)), 8)); // 6
	m_controls.push_back(std::pair<btVector3,int>(btVector3(btScalar(0.0), btScalar(0.0), btScalar(1.0)), 8)); // 7
	m_controls.push_back(std::pair<btVector3,int>(btVector3(btScalar(0.0), btScalar(1.0), btScalar(0.0)), 8)); // 8

	m_jointControls.push_back(JointControl(lHipConstraint,3));
	m_jointControls.push_back(JointControl(lHipConstraint,4));
	m_jointControls.push_back(JointControl(lHipConstraint,5));

	btTransform lThighAxisOffset;
	lThighAxisOffset.setIdentity();
	lThighAxisOffset.setRotation(btQuaternion(0,0,-20*M_PI/180));
	m_evaluators.push_back(new Joint3DOF("lfemur", lHipConstraint, 0, 0, 20,
					     lThighAxisOffset));

	// Left Shin 9
	btTransform lShinTransform;
	lShinTransform.setIdentity();
	btVector3 vLShinOrigin = vLShin + btVector3(btScalar(0.0), btScalar(-0.15), btScalar(0.0));
	lShinTransform.setOrigin(vLShinOrigin);
	btRigidBody *lShinBody = localCreateRigidBody(btScalar(3.1), offset*lShinTransform, lShinShape);
	m_bodies.push_back(lShinBody);
	btTransform lKneePivot;
	lKneePivot.setIdentity();
	lKneePivot.setOrigin(vLShin);
	lKneePivot = offset * lKneePivot;
	//lShinTransform.setRotation(btQuaternion(btVector3(0,0,1), M_PI_4));
	lShinTransform.setOrigin(btVector3(0,0,0));
	btTransform lKneeThighFrame = lThighBody->getWorldTransform().inverse() * lKneePivot;
	btTransform lKneeShinFrame = lShinTransform * lShinBody->getWorldTransform().inverse() * lKneePivot;
	lKneeShinFrame.setOrigin(btVector3(0,0.3/2,0));
	//btHingeConstraint *lKneeConstraint = new btHingeConstraint(*lThighBody, *lShinBody, lKneeThighFrame, lKneeShinFrame, lKneeAxis, lKneeAxis);
	btVector3 lKneeLL(-M_PI_2, 0.0, 0.0);
	btVector3 lKneeUL(M_PI_4, 0.0, 0.0);
	btGeneric6DofSpring2Constraint *lKneeConstraint = new btGeneric6DofSpring2Constraint(*lThighBody, *lShinBody, lKneeThighFrame, lKneeShinFrame);
	lKneeConstraint->setAngularLowerLimit(lKneeLL);
	lKneeConstraint->setAngularUpperLimit(lKneeUL);
	lKneeConstraint->enableSpring(3, true);
	lKneeConstraint->setDamping(3, damping);
	lKneeConstraint->setStiffness(3, stiffness);
	lKneeConstraint->enableSpring(4, true);
	lKneeConstraint->setDamping(4, damping);
	lKneeConstraint->setStiffness(4, stiffness);
	lKneeConstraint->enableSpring(5, true);
	lKneeConstraint->setDamping(5, damping);
	lKneeConstraint->setStiffness(5, stiffness);
	m_ownerWorld->addConstraint(lKneeConstraint, true);
	m_joints.push_back(lKneeConstraint);
	m_parents.push_back(8);
	m_controls.push_back(std::pair<btVector3,int>(btVector3(btScalar(-1.0), btScalar(0.0), btScalar(0.0)), 9)); // 9

	m_jointControls.push_back(JointControl(lKneeConstraint,3));

	m_evaluators.push_back(new Joint1DOF("ltibia", lKneeConstraint, 0, -1.0));

	// Left Foot 10
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
	m_parents.push_back(9);

	// Right Upper Arm 11
	btTransform rUpArmTransform;
	rUpArmTransform.setIdentity();
	btVector3 vRUpArmOrigin = vRUpArm + btVector3(btScalar(-armLength / 2), btScalar(0.0), btScalar(0.0));
	rUpArmTransform.setOrigin(vRUpArmOrigin);
	//rUpArmTransform.setRotation(btQuaternion(vAxis, M_PI_2));
	btRigidBody *rUpArmBody = localCreateRigidBody(btScalar(1.594), offset*rUpArmTransform, rUpArmShape);
	//btRigidBody *rUpArmBody = localCreateRigidBody(btScalar(0), offset*rUpArmTransform, rUpArmShape);
	m_bodies.push_back(rUpArmBody);
	btTransform rShldrPivot;
	rShldrPivot.setIdentity();
	rShldrPivot.setOrigin(vRUpArm);
	rShldrPivot = offset * rShldrPivot;
	//btTransform rShldrArmFrame = rUpArmTransform * rUpArmBody->getWorldTransform().inverse() * rShldrPivot;
	btTransform rShldrArmFrame = rUpArmBody->getWorldTransform().inverse() * rShldrPivot;
	rShldrArmFrame.setOrigin(btVector3(armLength/2,0,0));
	btTransform rShldrWaistFrame = upWaistBody->getWorldTransform().inverse() * rShldrPivot;
	btGeneric6DofSpring2Constraint *rShldrConstraint = new btGeneric6DofSpring2Constraint(*upWaistBody, *rUpArmBody, rShldrWaistFrame, rShldrArmFrame);
	btVector3 rShldrLL(-M_PI_2, -M_PI_2-M_PI_4, -M_PI_2-M_PI_4);
	btVector3 rShldrUL(M_PI_2, M_PI_2+M_PI_4, M_PI_2+M_PI_4);
	//rShldrConstraint->setAngularLowerLimit(rShldrLL);
	//rShldrConstraint->setAngularUpperLimit(rShldrUL);
	rShldrConstraint->enableSpring(3, true);
	rShldrConstraint->setDamping(3, damping);
	rShldrConstraint->setStiffness(3, stiffness);
	rShldrConstraint->enableSpring(4, true);
	rShldrConstraint->setDamping(4, damping);
	rShldrConstraint->setStiffness(4, stiffness);
	rShldrConstraint->enableSpring(5, true);
	rShldrConstraint->setDamping(5, damping);
	rShldrConstraint->setStiffness(5, stiffness);
	m_ownerWorld->addConstraint(rShldrConstraint, true);
	m_joints.push_back(rShldrConstraint);
	m_parents.push_back(2);
	m_controls.push_back(std::pair<btVector3,int>(btVector3(btScalar(1.0), btScalar(0.0), btScalar(0.0)), 11)); // 10
	m_controls.push_back(std::pair<btVector3,int>(btVector3(btScalar(0.0), btScalar(0.0), btScalar(1.0)), 11)); // 11
	m_controls.push_back(std::pair<btVector3,int>(btVector3(btScalar(0.0), btScalar(1.0), btScalar(0.0)), 11)); // 12

	m_jointControls.push_back(JointControl(rShldrConstraint,3));
	m_jointControls.push_back(JointControl(rShldrConstraint,4));
	m_jointControls.push_back(JointControl(rShldrConstraint,5));

	btTransform rShldrAxisOffset;
	rShldrAxisOffset.setIdentity();\
	rShldrAxisOffset.setRotation(btQuaternion(0,30*M_PI/180,0));
	m_evaluators.push_back(new Joint3DOF("rhumerus", rShldrConstraint, 180, 30, 90,
					     rShldrAxisOffset));

	rUpArmBody->setCollisionFlags(4);

	// Right Lower Arm 12
	btTransform rLoArmTransform;
	rLoArmTransform.setIdentity();
	btVector3 vRLoArmOrigin = vRLoArm + btVector3(btScalar(-armLength / 2), btScalar(0.0),btScalar(0.0));
	rLoArmTransform.setOrigin(vRLoArmOrigin);
	//rLoArmTransform.setRotation(btQuaternion(vAxis, M_PI_2));
	btRigidBody *rLoArmBody = localCreateRigidBody(btScalar(0.877), offset*rLoArmTransform, rLoArmShape);
	//btRigidBody *rLoArmBody = localCreateRigidBody(btScalar(0.0), offset*rLoArmTransform, rLoArmShape);
	m_bodies.push_back(rLoArmBody);
	btTransform rElbowPivot;
	rElbowPivot.setIdentity();
	rElbowPivot.setOrigin(vRLoArm);
	rElbowPivot = offset * rElbowPivot;
	btTransform rElbowUpFrame = rUpArmBody->getWorldTransform().inverse() * rElbowPivot;
	btTransform rElbowLoFrame = rLoArmBody->getWorldTransform().inverse() * rElbowPivot;
	//btHingeConstraint *rElbowConstraint = new btHingeConstraint(*rUpArmBody, *rLoArmBody, rElbowUpFrame, rElbowLoFrame, rElbowAxis, rElbowAxis);
	btGeneric6DofSpring2Constraint *rElbowConstraint = new btGeneric6DofSpring2Constraint(*rUpArmBody, *rLoArmBody, rElbowUpFrame, rElbowLoFrame);
	btVector3 rElbowLL(0.0, -M_PI_2, 0.0);
	btVector3 rElbowUL(0.0, M_PI_4, 0.0);
	rElbowConstraint->setAngularLowerLimit(rElbowLL);
	rElbowConstraint->setAngularUpperLimit(rElbowUL);
	rElbowConstraint->enableSpring(3, true);
	rElbowConstraint->setDamping(3, damping);
	rElbowConstraint->setStiffness(3, stiffness);
	rElbowConstraint->enableSpring(4, true);
	rElbowConstraint->setDamping(4, damping);
	rElbowConstraint->setStiffness(4, stiffness);
	rElbowConstraint->enableSpring(5, true);
	rElbowConstraint->setDamping(5, damping);
	rElbowConstraint->setStiffness(5, stiffness);
	//rElbowConstraint->setLimit(btScalar(-M_PI_2), btScalar(0.05));
	m_ownerWorld->addConstraint(rElbowConstraint, true);
	m_joints.push_back(rElbowConstraint);
	m_parents.push_back(11);
	m_controls.push_back(std::pair<btVector3,int>(btVector3(btScalar(0.0), btScalar(1.0), btScalar(0.0)), 12)); // 13
	m_evaluators.push_back(new Joint1DOF("rradius", rElbowConstraint, 1, -1.0));

	m_jointControls.push_back(JointControl(rElbowConstraint,4));

	rLoArmBody->setCollisionFlags(4);

	// Right Hand 13
	btTransform rHandTransform;
	rHandTransform.setIdentity();
	btVector3 vRHandOrigin = vRHand + btVector3(btScalar(0.0), btScalar(0.0), btScalar(0.0));
	rHandTransform.setOrigin(vRHandOrigin);
	btRigidBody *rHandBody = localCreateRigidBody(btScalar(0.25), offset*rHandTransform, rHandShape);
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
	m_parents.push_back(12);

	rHandBody->setCollisionFlags(4);

	// Left Upper Arm 14
	btTransform lUpArmTransform;
	lUpArmTransform.setIdentity();
	btVector3 vLUpArmOrigin = vLUpArm + btVector3(btScalar(armLength / 2), btScalar(0.0), btScalar(0.0));
	lUpArmTransform.setOrigin(vLUpArmOrigin);
	//lUpArmTransform.setRotation(btQuaternion(vAxis, M_PI_2));
	btRigidBody *lUpArmBody = localCreateRigidBody(btScalar(1.594), offset*lUpArmTransform, lUpArmShape);
	//btRigidBody *lUpArmBody = localCreateRigidBody(btScalar(0), offset*lUpArmTransform, lUpArmShape);
	m_bodies.push_back(lUpArmBody);
	btTransform lShldrPivot;
	lShldrPivot.setIdentity();
	lShldrPivot.setOrigin(vLUpArm);
	lShldrPivot = offset * lShldrPivot;
	//lUpArmTransform.setOrigin(btVector3(0,0,0));
	btTransform lShldrArmFrame = lUpArmTransform.inverse() * lUpArmBody->getWorldTransform().inverse() * lShldrPivot;
	lShldrArmFrame.setOrigin(btVector3(-armLength/2,0,0));
	//btTransform lShldrArmFrame = lShldrPivot;
	btTransform lShldrWaistFrame = upWaistBody->getWorldTransform().inverse() * lShldrPivot;
	//btConeTwistConstraint *lShldrConstraint = new btConeTwistConstraint(*upWaistBody, *lUpArmBody, lShldrWaistFrame, lShldrArmFrame);
	//lShldrConstraint->setLimit(btScalar(M_PI), btScalar(0.0), btScalar(M_PI));

	btGeneric6DofSpring2Constraint *lShldrConstraint = new btGeneric6DofSpring2Constraint(*upWaistBody, *lUpArmBody, lShldrWaistFrame, lShldrArmFrame);
	btVector3 lShldrLL(-M_PI_2, -M_PI_2-M_PI_4, -M_PI_2-M_PI_4);
	btVector3 lShldrUL(M_PI_2, M_PI_2+M_PI_4, M_PI_2+M_PI_4);
	//lShldrConstraint->setAngularLowerLimit(lShldrLL);
	//lShldrConstraint->setAngularUpperLimit(lShldrUL);
	lShldrConstraint->enableSpring(3, true);
	lShldrConstraint->setDamping(3, damping);
	lShldrConstraint->setStiffness(3, stiffness);
	lShldrConstraint->enableSpring(4, true);
	lShldrConstraint->setDamping(4, damping);
	lShldrConstraint->setStiffness(4, stiffness);
	lShldrConstraint->enableSpring(5, true);
	lShldrConstraint->setDamping(5, damping);
	lShldrConstraint->setStiffness(5, stiffness);

	m_ownerWorld->addConstraint(lShldrConstraint, true);
	m_joints.push_back(lShldrConstraint);
	m_parents.push_back(2);
	m_controls.push_back(std::pair<btVector3,int>(btVector3(btScalar(-1.0), btScalar(0.0), btScalar(0.0)), 14)); // 14
	m_controls.push_back(std::pair<btVector3,int>(btVector3(btScalar(0.0), btScalar(0.0), btScalar(-1.0)), 14)); // 15
	m_controls.push_back(std::pair<btVector3,int>(btVector3(btScalar(0.0), btScalar(-1.0), btScalar(0.0)), 14)); // 16

	m_jointControls.push_back(JointControl(lShldrConstraint,3));
	m_jointControls.push_back(JointControl(lShldrConstraint,4));
	m_jointControls.push_back(JointControl(lShldrConstraint,5));

	btTransform lShldrAxisOffset;
	lShldrAxisOffset.setIdentity();
	lShldrAxisOffset.setRotation(btQuaternion(0,30*M_PI/180,0));
	m_evaluators.push_back(new Joint3DOF("lhumerus", lShldrConstraint, 180, -30, -90,
					     lShldrAxisOffset));

	lUpArmBody->setCollisionFlags(4);

	// Left Lower Arm 15
	btTransform lLoArmTransform;
	lLoArmTransform.setIdentity();
	btVector3 vLLoArmOrigin = vLLoArm + btVector3(btScalar(armLength / 2), btScalar(0.0), btScalar(0.0));
	lLoArmTransform.setOrigin(vLLoArmOrigin);
	//lLoArmTransform.setRotation(btQuaternion(vAxis, M_PI_2));
	btRigidBody *lLoArmBody = localCreateRigidBody(btScalar(0.877), offset*lLoArmTransform, lLoArmShape);
	m_bodies.push_back(lLoArmBody);
	btTransform lElbowPivot;
	lElbowPivot.setIdentity();
	lElbowPivot.setOrigin(vLLoArm);
	lElbowPivot = offset * lElbowPivot;
	btTransform lElbowUpFrame = lUpArmBody->getWorldTransform().inverse() * lElbowPivot;
	btTransform lElbowLoFrame = lLoArmBody->getWorldTransform().inverse() * lElbowPivot;
	//btHingeConstraint *lElbowConstraint = new btHingeConstraint(*lUpArmBody, *lLoArmBody, lElbowUpFrame, lElbowLoFrame, lElbowAxis, lElbowAxis);
	btGeneric6DofSpring2Constraint *lElbowConstraint = new btGeneric6DofSpring2Constraint(*lUpArmBody, *lLoArmBody, lElbowUpFrame, lElbowLoFrame);
	btVector3 lElbowLL(0.0, -M_PI_4, 0.0);
	btVector3 lElbowUL(0.0, M_PI_2, 0.0);
	lElbowConstraint->setAngularLowerLimit(lElbowLL);
	lElbowConstraint->setAngularUpperLimit(lElbowUL);
	lElbowConstraint->enableSpring(3, true);
	lElbowConstraint->setDamping(3, damping);
	lElbowConstraint->setStiffness(3, stiffness);
	lElbowConstraint->enableSpring(4, true);
	lElbowConstraint->setDamping(4, damping);
	lElbowConstraint->setStiffness(4, stiffness);
	lElbowConstraint->enableSpring(5, true);
	lElbowConstraint->setDamping(5, damping);
	lElbowConstraint->setStiffness(5, stiffness);
	//lElbowConstraint->setLimit(btScalar(-M_PI_2), btScalar(0.05));
	m_ownerWorld->addConstraint(lElbowConstraint, true);
	m_joints.push_back(lElbowConstraint);
	m_parents.push_back(14);
	m_controls.push_back(std::pair<btVector3,int>(btVector3(btScalar(0.0), btScalar(-1.0), btScalar(0.0)), 15)); // 17

	m_jointControls.push_back(JointControl(lElbowConstraint,4));
	
	m_evaluators.push_back(new Joint1DOF("lradius", lElbowConstraint, 1));

	lLoArmBody->setCollisionFlags(4);

	// Left Hand 16
	btTransform lHandTransform;
	lHandTransform.setIdentity();
	btVector3 vLHandOrigin = vLHand + btVector3(btScalar(0.0), btScalar(0.0), btScalar(0.0));
	lHandTransform.setOrigin(vLHandOrigin);
	btRigidBody *lHandBody = localCreateRigidBody(btScalar(0.25), offset*lHandTransform, lHandShape);
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
	m_parents.push_back(15);

	lHandBody->setCollisionFlags(4);

	for (int i = 0; i < m_jointControls.size(); ++i) {
	    btGeneric6DofSpring2Constraint* c = m_jointControls[i].first;
	    int index = m_jointControls[i].second;
	    c->enableMotor(index, true);
	    c->setMaxMotorForce(index, 5.0);
	}

	// Add blank evaluators so that each frame is complete for the mocap conversion
	/*m_evaluators.push_back(new JointBlank("rhand", 2));
	m_evaluators.push_back(new JointBlank("lhand", 2));
	m_evaluators.push_back(new JointBlank("rthumb", 2));
	m_evaluators.push_back(new JointBlank("lthumb", 2));
	m_evaluators.push_back(new JointBlank("thorax", 3));
	m_evaluators.push_back(new JointBlank("upperback", 3));
	m_evaluators.push_back(new JointBlank("lowerback", 3));
	m_evaluators.push_back(new JointBlank("upperneck", 3));
	m_evaluators.push_back(new JointBlank("lowerneck", 3));
	m_evaluators.push_back(new JointBlank("head", 3));
	m_evaluators.push_back(new JointBlank("rfoot", 2));
	m_evaluators.push_back(new JointBlank("lfoot", 2));
	m_evaluators.push_back(new JointBlank("rwrist", 1));
	m_evaluators.push_back(new JointBlank("lwrist", 1));
	m_evaluators.push_back(new JointBlank("rtoes", 1));
	m_evaluators.push_back(new JointBlank("ltoes", 1));
	m_evaluators.push_back(new JointBlank("rfingers", 1));
	m_evaluators.push_back(new JointBlank("lfingers", 1));
	m_evaluators.push_back(new JointBlank("rclavicle", 2));
	m_evaluators.push_back(new JointBlank("lclavicle", 2));*/
    }

    virtual	~HumanRig ()
    {
	int i;

	// Remove all constraints
	//std::cout << "Removing constraints\n";
	for ( i = 0; i < m_joints.size(); ++i)
	    {
		m_ownerWorld->removeConstraint(m_joints[i]);
		delete m_joints[i]; m_joints[i] = 0;
	    }

	for (i = 0; i < m_evaluators.size(); ++i) {
	    delete m_evaluators[i];
	}
	m_evaluators.clear();

	// Remove all bodies and shapes
	//std::cout << "Removing bodies\n";
	for ( i = 0; i < m_bodies.size(); ++i)
	    {
		//std::cout << "Removing rigid body from world\n";
		m_ownerWorld->removeRigidBody(m_bodies[i]);
			
		//std::cout << "Deleting motion state\n";
		delete m_bodies[i]->getMotionState();

		//std::cout << "Deleting bodies and shapes\n";
		delete m_bodies[i]; m_bodies[i] = 0;
		delete m_shapes[i]; m_shapes[i] = 0;
	    }
    }

    btTypedConstraint** GetJoints() {return &m_joints[0];}
    int GetJointSize() {return m_joints.size();}
    btRigidBody **GetBodies() {return &m_bodies[0];}
    int GetBodySize() {return m_bodies.size();}
    int *GetParents() {return &m_parents[0];}
    std::pair<btVector3, int> *GetControls() {return &m_controls[0];}
    int GetControlSize() {return m_controls.size();}

    void ApplyControl(const std::vector<double> &controls)
    {
	/*const double MAX_IMPULSE = 1.5;
	std::vector<double> inputs(controls);
	for (int i = 0; i < inputs.size(); ++i) {
	    if (inputs[i] < -MAX_IMPULSE) {
		inputs[i] = -MAX_IMPULSE;
	    }
	    else if (inputs[i] > MAX_IMPULSE) {
		inputs[i] = MAX_IMPULSE;
	    }
	}
	inputs.resize(GetControlSize());
	for (int i = 0; i < inputs.size(); ++i) {
	    int body = m_controls[i].second;
	    btRigidBody *b = m_bodies[body];
	    btRigidBody *p = m_bodies[m_parents[body]];
	    btVector3 t = inputs[i] * m_controls[i].first;
	    btTransform trans = b->getWorldTransform();
	    trans.setOrigin(btVector3(btScalar(0.0), btScalar(0.0), btScalar(0.0)));
	    t = trans * t;
	    b->applyTorqueImpulse(t);
	    p->applyTorqueImpulse(-t);

	    //if (inputs[i] > 0.01 || inputs[i] < -0.01) {
	    //std::cout << "Index " << i << ": " << inputs[i] << "\n";
	    //std::cout << "Applying torque to bodies " << body << " and " << m_parents[body] << "\n";
	    //std::cout << "Torque: " << t[0] << " " << t[1] << " " << t[2] << "\n";
	    //}
	}
	//btRigidBody *foot = m_bodies[6];
	//btVector3 imp(10.0,0,0);
	//foot->applyCentralImpulse(imp);*/

	for (int i = 0; i < m_jointControls.size() && i < controls.size(); ++i) {
	    btGeneric6DofSpring2Constraint* c = m_jointControls[i].first;
	    int index = m_jointControls[i].second;
	    c->setTargetVelocity(index, controls[i]);
	}
    }
};



void humanPreTickCallback (btDynamicsWorld *world, btScalar timeStep)
{
    HumanDemo* humanDemo = (HumanDemo*)world->getWorldUserInfo();

    humanDemo->setMotorTargets(timeStep);
	
}

void HumanDemo::preStepAction(float deltaTime)
{
    // Read the control from the socket
    float buffer[1000];
    char *newBuff = (char*)buffer;
    int controlSize;
    read(receiveSocket, (char*)&controlSize, sizeof(int));
    //std::cout << "Reading " << controlSize * sizeof(float) << " bytes of control data\n";
    //char *temp = (char*)&controlSize;
    //std::cout << "Bytes read:";
    //for (int i = 0; i < 4; ++i) {
	//std::cout << " " << (int)temp[i];
    //}
    //std::cout << "\n";
    std::vector<double> controls(controlSize);
    char *buffPtr = newBuff;
    int tries = 0;
    while (buffPtr - newBuff < controlSize * sizeof(float)) {
	int n = read(receiveSocket, buffPtr, controlSize * sizeof(float) - (buffPtr - newBuff));
	//int n = read(clientSocket, buffPtr, 1000);
	buffPtr += n;
	++tries;
    }
    //std::cout << "Read " << controlSize * sizeof(float) << " bytes of control data\n";
    //std::cout << "Read in " << tries << " iterations\n";

    // Check if we want to reset
    std::string response(newBuff);
    if (response == "RESET") {
	m_reset = true;
	return;
    }

    // If not, then apply the controls
    for (int i=0; i<controlSize; ++i) {
	controls[i] = buffer[i];
    }

    for (int r=0; r<m_rigs.size(); r++) {
	m_rigs[r]->ApplyControl(controls);
    }
}

void HumanDemo::postStepAction(float deltaTime)
{
    // Send the state of the system
    float buffer[1000];
    char *newBuff = (char*)buffer;

    // Get the state of the system
    int buffIdx = 0;
    for (int i = 0; i < m_rigs[0]->m_evaluators.size(); ++i) {
	std::vector<btScalar> r;
	m_rigs[0]->m_evaluators[i]->evaluate(r);
	//std::string name = m_rigs[0]->m_evaluators[i]->getName();
	for (int i = 0; i < r.size(); ++i) {
	    //std::cout <<  " " << std::setprecision(5) << r[i];
	    buffer[buffIdx++] = r[i];
	}
	//std::cout << "\n";
     }

    //std::vector<btScalar> r;
    //m_rigs[0]->m_evaluators[3]->evaluate(r);
    //std::cout << "Joint 3:";
    //for (int i = 0; i < r.size(); ++i) {
    //std::cout << " " << r[i];
    //}
    //std::cout << "\n";

     int size = buffIdx * sizeof(float);
    ((int*)newBuff)[0] = size;
    //std::cout << "Writing " << size << " bytes of state data\n";
    int n = write(clientSocket, &size, sizeof(int));
    n = write(clientSocket, newBuff, buffIdx * sizeof(float));
    //std::cout << "Wrote " << size << " bytes of state data\n";
}


void HumanDemo::initPhysics()
{
    if (m_guiHelper != 0) {
	m_guiHelper->setUpAxis(1);
    }

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

    //m_dynamicsWorld->setInternalTickCallback(humanPreTickCallback,this,true);
    if (m_guiHelper != 0) {
	m_guiHelper->createPhysicsDebugDrawer(m_dynamicsWorld);
    }
	

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
    //btVector3 startOffset(0,-0.1,0.0);
    btVector3 startOffset(0,1.5,0.0);
    //btVector3 startOffset(0,0.5,0.0);
    spawnHumanRig(startOffset, false);

    if (m_guiHelper != 0) { 
	m_guiHelper->autogenerateGraphicsObjects(m_dynamicsWorld);
    }
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
    
    // Send the state of the system
    float buffer[1000];
    char *newBuff = (char*)buffer;

    // Get the state of the system
    int buffIdx = 0;
    for (int i = 0; i < m_rigs[0]->m_evaluators.size(); ++i) {
	std::vector<btScalar> r;
	m_rigs[0]->m_evaluators[i]->evaluate(r);
	//std::string name = m_rigs[0]->m_evaluators[i]->getName();
	for (int i = 0; i < r.size(); ++i) {
	    //std::cout <<  " " << std::setprecision(5) << r[i];
	    buffer[buffIdx++] = r[i];
	}
	//std::cout << "\n";
     }

    std::vector<btScalar> r;
    m_rigs[0]->m_evaluators[1]->evaluate(r);
    std::cout << "Joint 1:";
    for (int i = 0; i < r.size(); ++i) {
	std::cout << " " << r[i];
    }
    std::cout << "\n";

     int size = buffIdx * sizeof(float);
    ((int*)newBuff)[0] = size;
    //std::cout << "Writing " << size << " bytes of state data\n";
    int n = write(clientSocket, &size, sizeof(int));
    n = write(clientSocket, newBuff, buffIdx * sizeof(float));
    //std::cout << "Wrote " << size << " bytes of state data\n";


    // Read the control from the socket
    int controlSize;
    read(clientSocket, (char*)&controlSize, sizeof(int));
    //std::cout << "Reading " << controlSize * sizeof(float) << " bytes of control data\n";
    char *temp = (char*)&controlSize;
    //std::cout << "Bytes read:";
    //for (int i = 0; i < 4; ++i) {
	//std::cout << " " << (int)temp[i];
    //}
    //std::cout << "\n";
    std::vector<double> controls(controlSize);
    char *buffPtr = newBuff;
    while (buffPtr - newBuff < controlSize * sizeof(float)) {
	n = read(clientSocket, buffPtr, controlSize*sizeof(float) - (buffPtr - newBuff));
	buffPtr += n;
    }
    //std::cout << "Read " << controlSize * sizeof(float) << " bytes of control data\n";

    /*// Check if we want to reset
    std::string response(newBuff);
    if (response == "RESET") {
	m_reset = true;
	return;
    }

    // If not, then apply the controls
    for (int i=0; i<controlSize; ++i) {
	controls[i] = buffer[i];
    }

    for (int r=0; r<m_rigs.size(); r++) {
	m_rigs[r]->ApplyControl(controls);
	}*/
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
    m_rigs.clear();

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
    m_collisionShapes.clear();

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
