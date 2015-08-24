#include "stdafx.h"
#include <vector>
#include <btBulletDynamicsCommon.h>
#include <btBulletCollisionCommon.h>

class Physics {

public:
	Physics(void);
	virtual ~Physics(void);
	virtual btDiscreteDynamicsWorld* getDynamicsWorld();
	virtual void addRigidBody(btRigidBody* body);
	virtual std::vector<btRigidBody*> getRigidBodies();
	
	virtual void addCan(btRigidBody* body);
	virtual std::vector<btRigidBody*> getCans();

	virtual void addCube(btRigidBody* body);
	virtual std::vector<btRigidBody*> getCubes();
	virtual int getCubeCount();
	virtual void removeCubes();

protected:
    btBroadphaseInterface* broadphase;
    btDefaultCollisionConfiguration* collisionConfiguration;
    btCollisionDispatcher* dispatcher;
    btSequentialImpulseConstraintSolver* solver;
    btDiscreteDynamicsWorld* dynamicsWorld;

	btRigidBody* rigidBody;

	unsigned int cubeCount;
	std::vector<btRigidBody*> rigidBodies;	//all other rigid bodies (table plate, table legs)
	std::vector<btRigidBody*> cubes;	//all throwable cubes
	std::vector<btRigidBody*> cans;		//all cans

};