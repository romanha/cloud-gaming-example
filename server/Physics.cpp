#include "Physics.h"

//constructor sets up a new DiscreteDynamicsWorld
Physics::Physics(void) {  
	// Build the broadphase
    this->broadphase = new btDbvtBroadphase();

    // Set up the collision configuration and dispatcher
    this->collisionConfiguration = new btDefaultCollisionConfiguration();
    this->dispatcher = new btCollisionDispatcher(collisionConfiguration);

    // The actual physics solver
    this->solver = new btSequentialImpulseConstraintSolver;

    // The world.
    this->dynamicsWorld = new btDiscreteDynamicsWorld(dispatcher, broadphase, solver, collisionConfiguration);
    this->dynamicsWorld->setGravity(btVector3(0, -9.81f, 0));

	this->rigidBody = NULL;
	this->rigidBodies;
	this->cubeCount = 0;

	this->cans;
}

Physics::~Physics(void) {

}

//getter for DiscreteDynamicsWorld
btDiscreteDynamicsWorld* Physics::getDynamicsWorld(void) {
	return this->dynamicsWorld;
}

void Physics::addRigidBody(btRigidBody* body) {
	this->rigidBodies.push_back(body);
}

std::vector<btRigidBody*> Physics::getRigidBodies(void) {
	return this->rigidBodies;
}

void Physics::addCan(btRigidBody* body) {
	this->cans.push_back(body);
}

std::vector<btRigidBody*> Physics::getCans(void) {
	return this->cans;
}

void Physics::addCube(btRigidBody* body) {
	this->cubeCount++;
	this->cubes.push_back(body);
}

std::vector<btRigidBody*> Physics::getCubes(void) {
	return this->cubes;
}

int Physics::getCubeCount(void) {
	return this->cubeCount;
}

void Physics::removeCubes(void) {
	while(cubes.size() > 0) {
		this->cubes.pop_back();
	}
	this->cubeCount = 0;
}