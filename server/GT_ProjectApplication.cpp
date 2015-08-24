/*
-----------------------------------------------------------------------------
Filename:    GT_OgreApplication.cpp
-----------------------------------------------------------------------------                          
	  adapted from the Ogre Tutorial Framework
	  http://www.ogre3d.org/tikiwiki/
-----------------------------------------------------------------------------
*/
#include "GT_ProjectApplication.h"
#include <string>
 
#if OGRE_PLATFORM == OGRE_PLATFORM_APPLE
#include <macUtils.h>
#endif

//-------------------------------------------------------------------------------------
GT_ProjectApplication::GT_ProjectApplication(void)
	: mRoot(0),
	mCamera(0),
	mSceneMgr(0),
	mWindow(0),
	mResourcesCfg(Ogre::StringUtil::BLANK),
	mPluginsCfg(Ogre::StringUtil::BLANK),
	mShutDown(false),
	mInputManager(0),
	mMouse(0),
	mKeyboard(0),
	mRenderTexture(0)
{
#if OGRE_PLATFORM == OGRE_PLATFORM_APPLE
    m_ResourcePath = Ogre::macBundlePath() + "/Contents/Resources/";
#else
    m_ResourcePath = "";
#endif

	// Initialize Variables for Speed of Camera Movement
    m_MoveSpeed			= 0.0f;	//30.0f when enabled (cycle per TAB through camera positions -> position 4 = free movement)
	m_RotateSpeed       = 50.0f;

	// Create Physics Engine
	mPhysics = new Physics();

	//UDP and Video Codec
	xvidEncoder = new EncodeVideoXVID();	//create encoder
	UDPSender = new UDPSend();	//create UDP sender
	UDPReceiver = new UDPReceive();

	//pointer for render texture
	pDest = NULL;

	//cube type and camera spot
	mCubeType = 1;
	mCameraSpot = 1;

	//throwing force
	mThrowingForce = 0.0f;

	mReset = false;
	mUpPressed = false;
	mDownPressed = false;
	mLeftPressed = false;
	mRightPressed = false;
	mAPressed = false;
	mDPressed = false;
	mWPressed = false;
	mSPressed = false;
	mQPressed = false;
	mEPressed = false;
}
 
//-------------------------------------------------------------------------------------
GT_ProjectApplication::~GT_ProjectApplication(void)
{ 
	//Remove ourself as a Window listener
	Ogre::WindowEventUtilities::removeWindowEventListener(mWindow, this);
	windowClosed(mWindow);
	
	m_rtt_texture.setNull();	//free Ogre pointer before mRoot

	delete mRoot;

	delete xvidEncoder;	//delete encoder pointer
	delete pDest;	//delete memory pointer for render texture
}

//-------------------------------------------------------------------------------------
void GT_ProjectApplication::go(void)
{
#ifdef _DEBUG
	#ifndef OGRE_STATIC_LIB	
		mResourcesCfg = m_ResourcePath + "resources_d.cfg";
		mPluginsCfg = m_ResourcePath + "plugins_d.cfg";
	#else
		mResourcesCfg = "resources_d.cfg";
		mPluginsCfg = "plugins_d.cfg";
	#endif
#else
	#ifndef OGRE_STATIC_LIB
		mResourcesCfg = m_ResourcePath + "resources.cfg";
		mPluginsCfg = m_ResourcePath + "plugins.cfg";
	#else
		mResourcesCfg = "resources.cfg";
		mPluginsCfg = "plugins.cfg";
	#endif
#endif
 
	if (!setup())
		return;
 
	mRoot->startRendering();
 
	// clean up
	destroyScene();
}

//-------------------------------------------------------------------------------------
bool GT_ProjectApplication::setup(void)
{
	mRoot = new Ogre::Root(mPluginsCfg);
 
	setupResources();
 
	bool carryOn = configure();
	if (!carryOn) return false;
 
	chooseSceneManager();
	createCamera();
	createViewports();
 
	// Load resources
	loadResources();
 
	// Create the scene
	createScene();

	createOIS();
	createWindowEventListener();

	createFrameListener();
 
	return true;
}
 
//-------------------------------------------------------------------------------------
bool GT_ProjectApplication::configure(void)
{
	/// Config Dialog to select settings
	if(mRoot->showConfigDialog())
	{
		mWindow = mRoot->initialise(true, "AMT_NetworkGame Server");
		return true;
	} return false;
}
//-------------------------------------------------------------------------------------
void GT_ProjectApplication::chooseSceneManager(void)
{
	/// Manages organisation and rendering of a scene
	mSceneMgr = mRoot->createSceneManager(Ogre::ST_GENERIC);
}
//-------------------------------------------------------------------------------------
void GT_ProjectApplication::createCamera(void)
{
	/// The main camera
	mCamera = mSceneMgr->createCamera("PlayerCam");
	mCamera->setPosition(Ogre::Vector3(0,15,50));
	mCamera->lookAt(Ogre::Vector3(0,8,0));
	mCamera->setNearClipDistance(0.1f);
}
//-------------------------------------------------------------------------------------
void GT_ProjectApplication::createFrameListener(void)
{
	/// Framelistener
	mRoot->addFrameListener(this);
}
//-------------------------------------------------------------------------------------
void GT_ProjectApplication::destroyScene(void)
{

}
//-------------------------------------------------------------------------------------
void GT_ProjectApplication::createViewports(void)
{
	/// Create a viewport, a rendering region for a render target
	Ogre::Viewport* vp = mWindow->addViewport(mCamera);
	vp->setBackgroundColour(Ogre::ColourValue(0,0,0));
 	mCamera->setAspectRatio(
		Ogre::Real(vp->getActualWidth()) / Ogre::Real(vp->getActualHeight()));
}
//-------------------------------------------------------------------------------------
void GT_ProjectApplication::setupResources(void)
{
	// Load resource paths from config file
	Ogre::ConfigFile cf;
	cf.load(mResourcesCfg);
 
	// Go through all sections & settings in the file
	Ogre::ConfigFile::SectionIterator seci = cf.getSectionIterator();
 
	Ogre::String secName, typeName, archName;
	while (seci.hasMoreElements())
	{
		secName = seci.peekNextKey();
		Ogre::ConfigFile::SettingsMultiMap *settings = seci.getNext();
		Ogre::ConfigFile::SettingsMultiMap::iterator i;
		for (i = settings->begin(); i != settings->end(); ++i)
		{
			typeName = i->first;
			archName = i->second;
			Ogre::ResourceGroupManager::getSingleton().addResourceLocation(
				archName, typeName, secName);
		}
	}
}

//-------------------------------------------------------------------------------------
void GT_ProjectApplication::loadResources(void)
{
	Ogre::ResourceGroupManager::getSingleton().initialiseAllResourceGroups();
}



void GT_ProjectApplication::createOIS(void)
{
	Ogre::LogManager::getSingletonPtr()->logMessage("*** Initializing OIS ***");
	OIS::ParamList pl;
	size_t windowHnd = 0;
	std::ostringstream windowHndStr;
	mWindow->getCustomAttribute("WINDOW", &windowHnd);
	windowHndStr << windowHnd;
	pl.insert(std::make_pair(std::string("WINDOW"), windowHndStr.str()));
	mInputManager = OIS::InputManager::createInputSystem( pl );
	mKeyboard = static_cast<OIS::Keyboard*>(mInputManager->createInputObject( OIS::OISKeyboard, true ));
	mMouse = static_cast<OIS::Mouse*>(mInputManager->createInputObject( OIS::OISMouse, true ));
	mMouse->setEventCallback(this);
	mKeyboard->setEventCallback(this);
}

void GT_ProjectApplication::createWindowEventListener(void)
{
	windowResized(mWindow);
	Ogre::WindowEventUtilities::addWindowEventListener(mWindow, this);
}

//-------------------------------------------------------------------------------------
bool GT_ProjectApplication::frameRenderingQueued(const Ogre::FrameEvent& evt)
{
	m_MoveScale = m_MoveSpeed   * (float)evt.timeSinceLastFrame;
	m_RotScale  = m_RotateSpeed * (float)evt.timeSinceLastFrame;

	m_YawAngle = Ogre::Degree(0.0);
	m_PitchAngle = Ogre::Degree(0.0);
	m_TranslateVector = Ogre::Vector3::ZERO;


	/*
	 * PHYSICS ENGINE
	 */
	if (this->mPhysics != NULL) {

		mPhysics->getDynamicsWorld()->stepSimulation((float)evt.timeSinceLastFrame, 10);

		btRigidBody* rigidBody = NULL;
		std::vector<btRigidBody*> rigidBodies = mPhysics->getCubes();
		
		//loop through all cubes and update their physical behaviour
		for (unsigned i = 0; i<rigidBodies.size(); i++) {
			rigidBody = rigidBodies.at(i);
			if (rigidBody && rigidBody->getMotionState()) {

				btTransform trans;
				rigidBody->getMotionState()->getWorldTransform(trans);

				void* userPointer = rigidBody->getUserPointer();
				if (userPointer) {
					btQuaternion orientation = trans.getRotation();
					Ogre::SceneNode* sceneNode = static_cast<Ogre::SceneNode*>(userPointer);
					sceneNode->setPosition(Ogre::Vector3(trans.getOrigin().getX(), trans.getOrigin().getY(), trans.getOrigin().getZ()));
					sceneNode->setOrientation(Ogre::Quaternion(orientation.getW(), orientation.getX(), orientation.getY(), orientation.getZ()));
				}
			}
		}


		rigidBodies = mPhysics->getCans();
		
		//loop through all cans and update their physical behaviour (they are saved seperately to respawn them easily)
		for (unsigned i = 0; i<rigidBodies.size(); i++) {
			rigidBody = rigidBodies.at(i);
		
			if (rigidBody && rigidBody->getMotionState()) {

				btTransform trans;
				rigidBody->getMotionState()->getWorldTransform(trans);

				void* userPointer = rigidBody->getUserPointer();
				if (userPointer) {
					btQuaternion orientation = trans.getRotation();
					Ogre::SceneNode* sceneNode = static_cast<Ogre::SceneNode*>(userPointer);
					sceneNode->setPosition(Ogre::Vector3(trans.getOrigin().getX(), trans.getOrigin().getY(), trans.getOrigin().getZ()));
					sceneNode->setOrientation(Ogre::Quaternion(orientation.getW(), orientation.getX(), orientation.getY(), orientation.getZ()));
				}
			}
		}


		rigidBodies = mPhysics->getRigidBodies();
		
		//loop through all other rigid bodies (like table) and update their physical behaviour (they are saved seperately to respawn them easily)
		for (unsigned i = 0; i<rigidBodies.size(); i++) {
			rigidBody = rigidBodies.at(i);
		
			if (rigidBody && rigidBody->getMotionState()) {

				btTransform trans;
				rigidBody->getMotionState()->getWorldTransform(trans);

				void* userPointer = rigidBody->getUserPointer();
				if (userPointer) {
					btQuaternion orientation = trans.getRotation();
					Ogre::SceneNode* sceneNode = static_cast<Ogre::SceneNode*>(userPointer);
					sceneNode->setPosition(Ogre::Vector3(trans.getOrigin().getX(), trans.getOrigin().getY(), trans.getOrigin().getZ()));
					sceneNode->setOrientation(Ogre::Quaternion(orientation.getW(), orientation.getX(), orientation.getY(), orientation.getZ()));
				}
			}
		}

	}
	/*
	 * END OF PHYSICS ENGINE
	 */

	/*
	 * INPUT ACTIONS
	 */
	//PRESS SPACE TO SHOOT CUBES (HOLD SPACE TO SHOOT STRONGER)
	static bool buttonWasDownSpace = false;
	bool buttonIsDownSpace = mKeyboard->isKeyDown(OIS::KC_SPACE);

	//get aspect ratio
	float x = (float)mWindow->getViewport(0)->getActualHeight();
	float y = (float)mWindow->getViewport(0)->getActualWidth();
	float aspectRatio = x/y;
	//calculate percentage of maximum throwing force
	float percentageForce = 0.0f;

	if(buttonIsDownSpace) {
		if(mThrowingForce < 3.0f)	//maximum throwing force
			mThrowingForce += 1.0f * (float)evt.timeSinceLastFrame;
		else
			mThrowingForce = 3.0f;

		percentageForce = mThrowingForce / 3.0f;
		mSceneMgr->getSceneNode("forceChargingBox")->setScale(0.03f * aspectRatio, 0.1f * percentageForce, 1); //(0.03f * aspectRatio, 0.0f, 1);
	}

	if(!buttonIsDownSpace && buttonWasDownSpace) {

		int cubeCount = mPhysics->getCubeCount();

		float cubeScale = 0.01f;
		if(mCubeType == 1)
			cubeScale = 0.01f;
		else if(mCubeType == 2)
			cubeScale = 0.02f;
		else if(mCubeType == 3)
			cubeScale = 0.03f;

		//create cubes manually for testing
		std::string cubeName = "Cube" + std::to_string(static_cast<long long>(cubeCount));
		Ogre::Entity* cube = mSceneMgr->createEntity(cubeName, "cube.mesh");
		cube->setMaterialName("Examples/BumpyMetal");
		std::string cubeNodeName = "CubeNode" + std::to_string(static_cast<long long>(cubeCount));
		Ogre::SceneNode* cubeNode = mSceneMgr->getRootSceneNode()->createChildSceneNode(cubeNodeName);
		cubeNode->attachObject(cube);
		cubeNode->setPosition(Ogre::Vector3(0.0f,-999.0f,0.0f));	//far below the ground, so that no cube can be seen in the time between creating the Ogre object and Bullet settings its position
		cubeNode->scale(Ogre::Vector3(cubeScale,cubeScale,cubeScale));	//makes cube size either 10x10x10, 20x20x20 or 30x30x30 (original mesh size is 100x100x100)

		//cube physics
		btCollisionShape* newRigidShape = new btBoxShape(btVector3(cubeScale*100/2, cubeScale*100/2, cubeScale*100/2));	//cube sizes are either 10x10x10, 20x20x20 or 30x30x30
		btTransform startTransform;
		startTransform.setIdentity();
		startTransform.setRotation(btQuaternion(1.0f, 1.0f, 1.0f, 0));
		btScalar mass = cubeScale * 100;	//1kg, 2kg or 3kg
		btVector3 localInertia(0.0f,0.0f,0.0f);
		
		float xPos = mCamera->getPosition().x + mCamera->getDirection().x * 2;
		float yPos = mCamera->getPosition().y + mCamera->getDirection().y * 2;
		float zPos = mCamera->getPosition().z + mCamera->getDirection().z * 2;

		startTransform.setOrigin(btVector3(xPos,yPos,zPos));
		newRigidShape->calculateLocalInertia(mass, localInertia);	//calculates local inertia by using the mass and body shape and saves it in localInertia

		btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);
		btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, myMotionState, newRigidShape, localInertia);
		rbInfo.m_friction = 0.8f;
		btRigidBody* body = new btRigidBody(rbInfo);
		body->setRestitution(0.0f);
		body->setUserPointer(cubeNode);

		//apply rotation (notize: bigger objects rotate slower)
		btVector3 torque(0.5f,0.5f,0.2f);
		body->applyTorqueImpulse(torque);
		
		float xImpulse = mCamera->getDirection().x * 50 * mThrowingForce;
		float yImpulse = mCamera->getDirection().y * 50 * mThrowingForce;
		float zImpulse = mCamera->getDirection().z * 50 * mThrowingForce;

		btVector3 impulse(xImpulse,yImpulse,zImpulse);
		body->applyCentralImpulse(impulse);

		body->setRestitution(0.0f);	//degree to which a collision is elastic or inelastic (1 = fully elastic)

		mPhysics->addCube(body);
		mPhysics->getDynamicsWorld()->addRigidBody(body);

		//reset throwing force
		mThrowingForce = 0.0f;
		mSceneMgr->getSceneNode("forceChargingBox")->setScale(0.03f * aspectRatio, 0.0f, 1);
	
	}
	buttonWasDownSpace = buttonIsDownSpace;


	//PRESS R TO RESPAWN CANS AND REMOVE ALL CUBES
	static bool buttonWasDownR = false;
	bool buttonIsDownR = mKeyboard->isKeyDown(OIS::KC_R);

	if(buttonIsDownR && !buttonWasDownR) {
		despawnCubes();
		respawnCans();
	}
	buttonWasDownR = buttonIsDownR;

	if(mReset) {
		despawnCubes();
		respawnCans();
		mReset = false;
	}


	//PRESS 1 TO SET CUBETYPE 1
	static bool buttonWasDown1 = false;
	bool buttonIsDown1 = mKeyboard->isKeyDown(OIS::KC_1);

	if(buttonIsDown1 && !buttonWasDown1) {
		mCubeType = 1;
		mSceneMgr->getSceneNode("smallCube")->setVisible(false);
		mSceneMgr->getSceneNode("smallCubeSelected")->setVisible(true);
		mSceneMgr->getSceneNode("mediumCube")->setVisible(true);
		mSceneMgr->getSceneNode("mediumCubeSelected")->setVisible(false);
		mSceneMgr->getSceneNode("bigCube")->setVisible(true);
		mSceneMgr->getSceneNode("bigCubeSelected")->setVisible(false);
	}
	buttonWasDown1 = buttonIsDown1;

	//PRESS 2 TO SET CUBETYPE 2
	static bool buttonWasDown2 = false;
	bool buttonIsDown2 = mKeyboard->isKeyDown(OIS::KC_2);

	if(buttonIsDown2 && !buttonWasDown2) {
		mCubeType = 2;
		mSceneMgr->getSceneNode("smallCube")->setVisible(true);
		mSceneMgr->getSceneNode("smallCubeSelected")->setVisible(false);
		mSceneMgr->getSceneNode("mediumCube")->setVisible(false);
		mSceneMgr->getSceneNode("mediumCubeSelected")->setVisible(true);
		mSceneMgr->getSceneNode("bigCube")->setVisible(true);
		mSceneMgr->getSceneNode("bigCubeSelected")->setVisible(false);
	}
	buttonWasDown2 = buttonIsDown2;

	//PRESS 3 TO SET CUBETYPE 3
	static bool buttonWasDown3 = false;
	bool buttonIsDown3 = mKeyboard->isKeyDown(OIS::KC_3);

	if(buttonIsDown3 && !buttonWasDown3) {
		mCubeType = 3;
		mSceneMgr->getSceneNode("smallCube")->setVisible(true);
		mSceneMgr->getSceneNode("smallCubeSelected")->setVisible(false);
		mSceneMgr->getSceneNode("mediumCube")->setVisible(true);
		mSceneMgr->getSceneNode("mediumCubeSelected")->setVisible(false);
		mSceneMgr->getSceneNode("bigCube")->setVisible(false);
		mSceneMgr->getSceneNode("bigCubeSelected")->setVisible(true);
	}
	buttonWasDown3 = buttonIsDown3;

	//PRESS TAB TO SWITCH BETWEEN CAMERA POSITIONS (1, 2, 3 = fixed positions & 4 = free movement)
	static bool buttonWasDownTab = false;
	bool buttonIsDownTab = mKeyboard->isKeyDown(OIS::KC_TAB);

	if(buttonIsDownTab && !buttonWasDownTab) {
		mCameraSpot++;
		if(mCameraSpot > 4) {
			mCameraSpot = 1;
			m_MoveSpeed = 0.0f;
			mSceneMgr->getSceneNode("disabledMovementBox")->setVisible(true);
		}

		if(mCameraSpot == 1) {
			mCamera->setPosition(Ogre::Vector3(0,10,50));
			mCamera->lookAt(Ogre::Vector3(0,8,0));
		}
		else if(mCameraSpot == 2) {
			mCamera->setPosition(Ogre::Vector3(50,10,30));
			mCamera->lookAt(Ogre::Vector3(0,8,0));
		}
		else if(mCameraSpot == 3) {
			mCamera->setPosition(Ogre::Vector3(-30,10,110));
			mCamera->lookAt(Ogre::Vector3(0,8,0));
		}
		else if(mCameraSpot == 4) {
			mCamera->setPosition(Ogre::Vector3(0,10,50));
			mCamera->lookAt(Ogre::Vector3(0,8,0));
			m_MoveSpeed = 30.0f;
			mSceneMgr->getSceneNode("disabledMovementBox")->setVisible(false);
		}
	}
	buttonWasDownTab = buttonIsDownTab;

	
	//W,A,S,D,Q,E FOR CAMERA MOVEMENT (ONLY IF ENABLED)
	if(mKeyboard->isKeyDown(OIS::KC_A))
		m_TranslateVector.x = -m_MoveScale;
	if(mKeyboard->isKeyDown(OIS::KC_D))
		m_TranslateVector.x = m_MoveScale;
	if(mKeyboard->isKeyDown(OIS::KC_W))
		m_TranslateVector.z = -m_MoveScale;
	if(mKeyboard->isKeyDown(OIS::KC_S))
		m_TranslateVector.z = m_MoveScale;
	if(mKeyboard->isKeyDown(OIS::KC_Q))
		m_TranslateVector.y = m_MoveScale;
	if(mKeyboard->isKeyDown(OIS::KC_E))
		m_TranslateVector.y = -m_MoveScale;

	
	//REMOTE INPUT CONTROLS
	if(mAPressed) {
		m_TranslateVector.x = -m_MoveScale;
		mAPressed = false;
	}
	if(mDPressed) {
		m_TranslateVector.x = m_MoveScale;
		mDPressed = false;
	}
	if(mWPressed) {
		m_TranslateVector.z = -m_MoveScale;
		mWPressed = false;
	}
	if(mSPressed) {
		m_TranslateVector.z = m_MoveScale;
		mSPressed = false;
	}
	if(mQPressed) {
		m_TranslateVector.y = m_MoveScale;
		mQPressed = false;
	}
	if(mEPressed) {
		m_TranslateVector.y = -m_MoveScale;
		mEPressed = false;
	}

	if(mUpPressed) {
		mCamera->pitch(Ogre::Degree(0.5f));
	}
	mUpPressed = false;
	
	if(mDownPressed) {
		mCamera->pitch(Ogre::Degree(-0.5f));
	}
	mDownPressed = false;
	
	if(mLeftPressed) {
		mCamera->yaw(Ogre::Degree(0.5f));
	}
	mLeftPressed = false;
	
	if(mRightPressed) {
		mCamera->yaw(Ogre::Degree(-0.5f));
	}
	mRightPressed = false;
	/*
	 * END OF INPUT ACTIONS
	 */


	OIS::MouseState state = mMouse->getMouseState();
	m_YawAngle = Ogre::Degree(state.X.rel * -m_RotScale);
	m_PitchAngle = Ogre::Degree(state.Y.rel * -m_RotScale);


	if(mWindow->isClosed())
		return false;
 
	if(mShutDown)
		return false;
 
	mKeyboard->capture();
	mMouse->capture();

	//apply camera movement / rotation
	mCamera->moveRelative(m_TranslateVector);
	mCamera->yaw(m_YawAngle);
	mCamera->pitch(m_PitchAngle);


	/*
	 * SAVING TO RENDER TEXTURE AND UDP STREAMING
	 */
	//save texture to main memory
	m_rtt_texture->getBuffer()->blitToMemory(m_encodeDest);
	

	//encode texture
	int memory = (mWindow->getWidth() * mWindow->getHeight() * 24) / 8;	//this should be way more than enough (raw size without encoding)
	char* encodeDestination = new char[memory];	//allocate memory for encoded data as char pointer (for UDP sending)
	int encodeSize = xvidEncoder->enc_main(pDest, encodeDestination, mWindow->getWidth(), mWindow->getHeight());	//size of the encoded data

	//Ogre::LogManager::getSingleton().logMessage(Ogre::LML_NORMAL, "Encoded: " + std::to_string(static_cast<long long>(encodeSize)));
	
	//send encoded bitstream per UDP
	UDPSender->send(encodeDestination, encodeSize);

	//free local pointers
	delete[] encodeDestination;


	return true;
}

//-------------------------------------------------------------------------------------
bool GT_ProjectApplication::keyPressed( const OIS::KeyEvent &arg )
{ 
	if (arg.key == OIS::KC_ESCAPE) mShutDown=true;
	return true;
}
 
bool GT_ProjectApplication::keyReleased( const OIS::KeyEvent &arg )
{
	return true;
}
 
bool GT_ProjectApplication::mouseMoved( const OIS::MouseEvent &arg )
{
	return true;
}
 
bool GT_ProjectApplication::mousePressed( const OIS::MouseEvent &arg, OIS::MouseButtonID id )
{
	return true;
}
 
bool GT_ProjectApplication::mouseReleased( const OIS::MouseEvent &arg, OIS::MouseButtonID id )
{
	return true;
}

void GT_ProjectApplication::createScene(void) {

	mSceneMgr->setSkyBox(true, "Examples/CloudyNoonSkyBox");
	mSceneMgr->setAmbientLight(Ogre::ColourValue(0.4f, 0.4f, 0.4f));
	mSceneMgr->setShadowTechnique(Ogre::SHADOWTYPE_STENCIL_ADDITIVE);
	
	Ogre::Light* directionalLight = mSceneMgr->createLight("directionalLight");
    directionalLight->setType(Ogre::Light::LT_DIRECTIONAL);
    directionalLight->setDiffuseColour(Ogre::ColourValue(1.0, 1.0, 1.0));
    directionalLight->setSpecularColour(Ogre::ColourValue(1.0, 1.0, 1.0));
	directionalLight->setDirection(Ogre::Vector3( -1, -1, -1 ));

	//create background scenery
	Ogre::Entity* tudorhouse = mSceneMgr->createEntity("Tudorhouse", "tudorhouse.mesh");
	tudorhouse->setCastShadows(true);
	Ogre::SceneNode* tudorhouseNode = mSceneMgr->getRootSceneNode()->createChildSceneNode("TudorhouseNode");
	tudorhouseNode->attachObject(tudorhouse);
	tudorhouseNode->yaw(Ogre::Degree(30));
	tudorhouseNode->scale(Ogre::Vector3(0.1f,0.1f,0.1f));
	tudorhouseNode->setPosition(-100,54,-100);
	tudorhouse = mSceneMgr->createEntity("Tudorhouse2", "tudorhouse.mesh");
	tudorhouse->setCastShadows(true);
	tudorhouseNode = mSceneMgr->getRootSceneNode()->createChildSceneNode("Tudorhouse2Node");
	tudorhouseNode->attachObject(tudorhouse);
	tudorhouseNode->yaw(Ogre::Degree(10));
	tudorhouseNode->scale(Ogre::Vector3(0.1f,0.1f,0.1f));
	tudorhouseNode->setPosition(130,54,-60);
	tudorhouse = mSceneMgr->createEntity("Tudorhouse3", "tudorhouse.mesh");
	tudorhouse->setCastShadows(true);
	tudorhouseNode = mSceneMgr->getRootSceneNode()->createChildSceneNode("Tudorhouse3Node");
	tudorhouseNode->attachObject(tudorhouse);
	tudorhouseNode->yaw(Ogre::Degree(320));
	tudorhouseNode->scale(Ogre::Vector3(0.1f,0.1f,0.1f));
	tudorhouseNode->setPosition(60,54,-200);
	tudorhouse = mSceneMgr->createEntity("Tudorhouse4", "tudorhouse.mesh");
	tudorhouse->setCastShadows(true);
	tudorhouseNode = mSceneMgr->getRootSceneNode()->createChildSceneNode("Tudorhouse4Node");
	tudorhouseNode->attachObject(tudorhouse);
	tudorhouseNode->yaw(Ogre::Degree(50));
	tudorhouseNode->scale(Ogre::Vector3(0.1f,0.1f,0.1f));
	tudorhouseNode->setPosition(-60,54,-300);

	//GROUND PLANE
	//create Ogre ground plane
	Ogre::Plane plane(Ogre::Vector3::UNIT_Y, 0);
    Ogre::MeshManager::getSingleton().createPlane("ground", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
                                                  plane, 1000, 1000, 20, 20, true, 1, 5, 5, Ogre::Vector3::UNIT_Z);
    
    Ogre::Entity* entGround = mSceneMgr->createEntity("GroundEntity", "ground");
    Ogre::SceneNode* ground = mSceneMgr->getRootSceneNode()->createChildSceneNode("GroundNode");
    ground->attachObject(entGround);
    
    entGround->setMaterialName("Examples/GrassFloor");
    entGround->setCastShadows(false);
    
    ground->setPosition(0,0,0);

	//create ground in physics engine
	btCollisionShape* groundShape = new btStaticPlaneShape(btVector3(0,1,0), 0);

	btTransform groundTransform;
	groundTransform.setIdentity();	//ground is at origin
	btDefaultMotionState* groundMotionState = new btDefaultMotionState(groundTransform);
	btScalar groundMass(0.);	//mass is infinite
	btVector3 localGroundInertia(0, 0, 0);	//ground has no inertia
	groundShape->calculateLocalInertia(groundMass, localGroundInertia);
	btRigidBody::btRigidBodyConstructionInfo groundRBInfo(groundMass, groundMotionState, groundShape, localGroundInertia);
	btScalar friction (0.5);
	groundRBInfo.m_friction = friction;
	btRigidBody* groundBody = new btRigidBody(groundRBInfo);

	mPhysics->getDynamicsWorld()->addRigidBody(groundBody);


	//OTHER OBJECTS AND HUD
	//spawn a wooden table
	spawnTable();

	//spawn the tin cans
	spawnCans();

	//draw HUD
	drawHUDCrosshair();
	drawHUDCubeTypes();
	drawHUDMovementStatus();
	drawHUDForceMeter();
	

	//save render texture to main memory
	m_rtt_texture = Ogre::TextureManager::getSingleton()
		.createManual("RttTex", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME, Ogre::TEX_TYPE_2D,
						mWindow->getWidth(), mWindow->getHeight(), 0,Ogre::PF_R8G8B8,Ogre::TU_RENDERTARGET);	//create render texture
	

	int memory = (mWindow->getWidth() * mWindow->getHeight() * 24) / 8;	//number of pixels with 24 bit per pixel (RGB24)
	pDest = new char[memory];	//allocate memory 

	m_encodeDest = Ogre::PixelBox(mWindow->getWidth(), mWindow->getHeight(), 1, Ogre::PF_B8G8R8, (void*)( pDest ) );
	
	//initialize XVID global and encoder
	xvidEncoder->global_init(true);
	xvidEncoder->enc_init(mWindow->getWidth(), mWindow->getHeight(), 128, 1, 20, 3);
	
	mRenderTexture = m_rtt_texture->getBuffer()->getRenderTarget();
	mRenderTexture->addViewport(mCamera);
	mRenderTexture->getViewport(0)->setClearEveryFrame(true);
	mRenderTexture->getViewport(0)->setOverlaysEnabled(true);
	mRenderTexture->setAutoUpdated(true);
	mRenderTexture->getViewport(0)->setBackgroundColour(Ogre::ColourValue::Black);

	//init UDP sender
	UDPSender->startWinsock();
	UDPSender->init("127.0.0.1", 8088);	//stream texture to localhost at port 8088

	//init UDP receiver
	UDPReceiver->startWinsock();
	UDPReceiver->init(8089);	//wait for key input at port 8089

	assert(!recThread);
	recThread = boost::shared_ptr<boost::thread>(new boost::thread(boost::bind(&GT_ProjectApplication::receiveCommands, this)));

	recThread.get()->start_thread();	
}

void GT_ProjectApplication::receiveCommands() {
	while(true) {
		char* buffer = new char[65000];
		double* ptime = new double(105);
		int receivedBytes = UDPReceiver->receive(buffer, 1, ptime);
		
		//Ogre::LogManager::getSingleton().logMessage(Ogre::LML_NORMAL, "Key pressed, Bytes received: " + std::to_string(static_cast<long long>(receivedBytes)));
		//Ogre::LogManager::getSingleton().logMessage(Ogre::LML_NORMAL, buffer);

		if(receivedBytes == -1) {
			continue;
		}
		else {
			//get aspect ratio
			float x = (float)mWindow->getViewport(0)->getActualHeight();
			float y = (float)mWindow->getViewport(0)->getActualWidth();
			float aspectRatio = x/y;
			//calculate percentage of maximum throwing force
			float percentageForce = 0.0f;

			if(strcmp(buffer, "SPACE") == 0) {
				if(mThrowingForce < 3.0f)	//maximum throwing force
					mThrowingForce += 1.0f * (1.0f/30.0f);	//assume 30 FPS
				else
					mThrowingForce = 3.0f;

				percentageForce = mThrowingForce / 3.0f;
				mSceneMgr->getSceneNode("forceChargingBox")->setScale(0.03f * aspectRatio, 0.1f * percentageForce, 1);
			}
			else if(strcmp(buffer,"SPACEEND") == 0) {
				
				int cubeCount = mPhysics->getCubeCount();

				float cubeScale = 0.01f;
				if(mCubeType == 1)
					cubeScale = 0.01f;
				else if(mCubeType == 2)
					cubeScale = 0.02f;
				else if(mCubeType == 3)
					cubeScale = 0.03f;

				//create cubes manually for testing
				std::string cubeName = "Cube" + std::to_string(static_cast<long long>(cubeCount));
				Ogre::Entity* cube = mSceneMgr->createEntity(cubeName, "cube.mesh");
				cube->setMaterialName("Examples/BumpyMetal");
				std::string cubeNodeName = "CubeNode" + std::to_string(static_cast<long long>(cubeCount));
				Ogre::SceneNode* cubeNode = mSceneMgr->getRootSceneNode()->createChildSceneNode(cubeNodeName);
				cubeNode->attachObject(cube);
				cubeNode->setPosition(Ogre::Vector3(0.0f,-999.0f,0.0f));	//far below the ground, so that no cube can be seen in the time between creating the Ogre object and Bullet settings its position
				cubeNode->scale(Ogre::Vector3(cubeScale,cubeScale,cubeScale));	//makes cube size either 10x10x10, 20x20x20 or 30x30x30 (original mesh size is 100x100x100)

				//cube physics
				btCollisionShape* newRigidShape = new btBoxShape(btVector3(cubeScale*100/2, cubeScale*100/2, cubeScale*100/2));	//cube sizes are either 10x10x10, 20x20x20 or 30x30x30
				btTransform startTransform;
				startTransform.setIdentity();
				startTransform.setRotation(btQuaternion(1.0f, 1.0f, 1.0f, 0));
				btScalar mass = cubeScale * 100;	//1kg, 2kg or 3kg
				btVector3 localInertia(0.0f,0.0f,0.0f);
		
				float xPos = mCamera->getPosition().x + mCamera->getDirection().x * 2;
				float yPos = mCamera->getPosition().y + mCamera->getDirection().y * 2;
				float zPos = mCamera->getPosition().z + mCamera->getDirection().z * 2;

				startTransform.setOrigin(btVector3(xPos,yPos,zPos));
				newRigidShape->calculateLocalInertia(mass, localInertia);	//calculates local inertia by using the mass and body shape and saves it in localInertia

				btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);
				btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, myMotionState, newRigidShape, localInertia);
				rbInfo.m_friction = 0.8f;
				btRigidBody* body = new btRigidBody(rbInfo);
				body->setRestitution(0.0f);
				body->setUserPointer(cubeNode);

				//apply rotation (notize: bigger objects rotate slower)
				btVector3 torque(0.5f,0.5f,0.2f);
				body->applyTorqueImpulse(torque);
		
				float xImpulse = mCamera->getDirection().x * 50 * mThrowingForce;
				float yImpulse = mCamera->getDirection().y * 50 * mThrowingForce;
				float zImpulse = mCamera->getDirection().z * 50 * mThrowingForce;

				btVector3 impulse(xImpulse,yImpulse,zImpulse);
				body->applyCentralImpulse(impulse);

				body->setRestitution(0.0f);	//degree to which a collision is elastic or inelastic (1 = fully elastic)

				mPhysics->addCube(body);
				mPhysics->getDynamicsWorld()->addRigidBody(body);

				//reset throwing force
				mThrowingForce = 0.0f;
				mSceneMgr->getSceneNode("forceChargingBox")->setScale(0.03f * aspectRatio, 0.0f, 1);
			}
			else if(strcmp(buffer, "R") == 0) {
				mReset = true;
				//if this thread would call despawnCubes there could be problems when frameRenderingQueued tries simultaneously to update physics
			}
			else if(strcmp(buffer, "1") == 0) {
				mCubeType = 1;
				mSceneMgr->getSceneNode("smallCube")->setVisible(false);
				mSceneMgr->getSceneNode("smallCubeSelected")->setVisible(true);
				mSceneMgr->getSceneNode("mediumCube")->setVisible(true);
				mSceneMgr->getSceneNode("mediumCubeSelected")->setVisible(false);
				mSceneMgr->getSceneNode("bigCube")->setVisible(true);
				mSceneMgr->getSceneNode("bigCubeSelected")->setVisible(false);
			}
			else if(strcmp(buffer, "2") == 0) {
				mCubeType = 2;
				mSceneMgr->getSceneNode("smallCube")->setVisible(true);
				mSceneMgr->getSceneNode("smallCubeSelected")->setVisible(false);
				mSceneMgr->getSceneNode("mediumCube")->setVisible(false);
				mSceneMgr->getSceneNode("mediumCubeSelected")->setVisible(true);
				mSceneMgr->getSceneNode("bigCube")->setVisible(true);
				mSceneMgr->getSceneNode("bigCubeSelected")->setVisible(false);
			}
			else if(strcmp(buffer, "3") == 0) {
				mCubeType = 3;
				mSceneMgr->getSceneNode("smallCube")->setVisible(true);
				mSceneMgr->getSceneNode("smallCubeSelected")->setVisible(false);
				mSceneMgr->getSceneNode("mediumCube")->setVisible(true);
				mSceneMgr->getSceneNode("mediumCubeSelected")->setVisible(false);
				mSceneMgr->getSceneNode("bigCube")->setVisible(false);
				mSceneMgr->getSceneNode("bigCubeSelected")->setVisible(true);
			}
			else if(strcmp(buffer, "TAB") == 0) {
				mCameraSpot++;
				if(mCameraSpot > 4) {
					mCameraSpot = 1;
					m_MoveSpeed = 0.0f;
					mSceneMgr->getSceneNode("disabledMovementBox")->setVisible(true);
				}

				if(mCameraSpot == 1) {
					mCamera->setPosition(Ogre::Vector3(0,10,50));
					mCamera->lookAt(Ogre::Vector3(0,8,0));
				}
				else if(mCameraSpot == 2) {
					mCamera->setPosition(Ogre::Vector3(50,10,30));
					mCamera->lookAt(Ogre::Vector3(0,8,0));
				}
				else if(mCameraSpot == 3) {
					mCamera->setPosition(Ogre::Vector3(-30,10,110));
					mCamera->lookAt(Ogre::Vector3(0,8,0));
				}
				else if(mCameraSpot == 4) {
					mCamera->setPosition(Ogre::Vector3(0,10,50));
					mCamera->lookAt(Ogre::Vector3(0,8,0));
					m_MoveSpeed = 30.0f;
					mSceneMgr->getSceneNode("disabledMovementBox")->setVisible(false);
				}
			}
			else if(strcmp(buffer, "UP") == 0) {
				mUpPressed = true;
			}
			else if(strcmp(buffer, "DOWN") == 0) {
				mDownPressed = true;
			}
			else if(strcmp(buffer, "LEFT") == 0) {
				mLeftPressed = true;
			}
			else if(strcmp(buffer, "RIGHT") == 0) {
				mRightPressed = true;
			}
			else if(strcmp(buffer, "A") == 0) {
				mAPressed = true;
			}
			else if(strcmp(buffer, "D") == 0) {
				mDPressed = true;
			}
			else if(strcmp(buffer, "W") == 0) {
				mWPressed = true;
			}
			else if(strcmp(buffer, "S") == 0) {
				mSPressed = true;
			}
			else if(strcmp(buffer, "Q") == 0) {
				mQPressed = true;
			}
			else if(strcmp(buffer, "E") == 0) {
				mEPressed = true;
			}

		}

		delete ptime;
		delete[] buffer;
	}
}
 
//Adjust mouse clipping area
void GT_ProjectApplication::windowResized(Ogre::RenderWindow* rw)
{
	unsigned int width, height, depth;
	int left, top;
	rw->getMetrics(width, height, depth, left, top);
 
	const OIS::MouseState &ms = mMouse->getMouseState();
	ms.width = width;
	ms.height = height;
}
void GT_ProjectApplication::windowClosed(Ogre::RenderWindow* rw)
{	if( rw == mWindow )
	{
		if( mInputManager )
		{
			mInputManager->destroyInputObject( mMouse );
			mInputManager->destroyInputObject( mKeyboard );
 
			OIS::InputManager::destroyInputSystem(mInputManager);
			mInputManager = 0;
		}
	}
}




void GT_ProjectApplication::spawnTable(void) {

	//set material scale to a custom value
	Ogre::MaterialPtr material = Ogre::MaterialManager::getSingleton().getByName("WoodPallet");
	material->getTechnique(0)->getPass(0)->getTextureUnitState(0)->setTextureScale(0.1f, 0.1f);

	
	//create tablePlate
	Ogre::Entity* tablePlate = mSceneMgr->createEntity("TablePlate", "cube.mesh");
	tablePlate->setMaterialName("WoodPallet");
	Ogre::SceneNode* tablePlateNode = mSceneMgr->getRootSceneNode()->createChildSceneNode("TablePlateNode");
	tablePlateNode->attachObject(tablePlate);
	tablePlateNode->setPosition(Ogre::Vector3(0.0f,-999.0f,0.0f));	//far below the ground, so that no mesh can be seen in the time between creating the Ogre object and Bullet setting its position
	tablePlateNode->scale(Ogre::Vector3(0.2f,0.005f,0.1f));	//makes tablePlate size 20x0.5x10 (original mesh size is 100x100x100)

	//tablePlate physics
	btCollisionShape* newRigidShape = new btBoxShape(btVector3(20/2, 0.5/2, 10/2));
	btTransform startTransform;
	startTransform.setIdentity();
	startTransform.setOrigin(btVector3(0,5,0));

	btScalar mass = 0.0f;	// 0 mass = static object
	btVector3 localInertia(0.0f,0.0f,0.0f);
		
	newRigidShape->calculateLocalInertia(mass, localInertia);	//calculates local inertia by using the mass and body shape and saves it in localInertia

	btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);
	btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, myMotionState, newRigidShape, localInertia);
	rbInfo.m_friction = 0.8f;
	btRigidBody* tablePlateRigid = new btRigidBody(rbInfo);
	tablePlateRigid->setRestitution(0.0f);
	tablePlateRigid->setUserPointer(tablePlateNode);

	mPhysics->addRigidBody(tablePlateRigid);
	mPhysics->getDynamicsWorld()->addRigidBody(tablePlateRigid);

	
	//change material scale for table legs
	//material->getTechnique(0)->getPass(0)->getTextureUnitState(0)->setTextureScale(1.0f, 0.1f);
	
	//spawn 4 table legs
	float posX = 0.0f;
	float posY = 0.0f;
	float posZ = 0.0f;

	for (int i = 1; i<5; i++) {

		//change position for each leg
		if(i == 1) {
			posX = 10-0.5-0.5;
			posY = 2.5f;
			posZ = 5-0.5-0.5;
		}
		else if(i == 2) {
			posX = 10-0.5-0.5;
			posY = 2.5f;
			posZ = -5+0.5+0.5;
		}
		else if(i == 3) {
			posX = -10+0.5+0.5;
			posY = 2.5f;
			posZ = 5-0.5-0.5;
		}
		else if(i == 4) {
			posX = -10+0.5+0.5;
			posY = 2.5f;
			posZ = -5+0.5+0.5;
		}

	
		Ogre::Entity* tableLeg = mSceneMgr->createEntity("TableLeg" + std::to_string(static_cast<long long>(i)), "cube.mesh");
		tableLeg->setMaterialName("WoodPallet");
		Ogre::SceneNode* tableLegNode = mSceneMgr->getRootSceneNode()->createChildSceneNode("TableLegNode" + std::to_string(static_cast<long long>(i)));
		tableLegNode->attachObject(tableLeg);
		tableLegNode->setPosition(Ogre::Vector3(0.0f,-999.0f,0.0f));	//far below the ground, so that no mesh can be seen in the time between creating the Ogre object and Bullet setting its position
		tableLegNode->scale(Ogre::Vector3(0.005f,0.05f,0.005f));	//makes tableLeg size 0.5x5x0.5 (original mesh size is 100x100x100)

		//tableLeg physics
		newRigidShape = new btBoxShape(btVector3(0.5/2, 5/2, 0.5/2));
		startTransform.setIdentity();
		startTransform.setOrigin(btVector3(posX,posY,posZ));	//set table leg position

		mass = 0.0f;	// 0 mass = static object
		localInertia = btVector3(0.0f,0.0f,0.0f);
		
		newRigidShape->calculateLocalInertia(mass, localInertia);	//calculates local inertia by using the mass and body shape and saves it in localInertia

		myMotionState = new btDefaultMotionState(startTransform);
		rbInfo = btRigidBody::btRigidBodyConstructionInfo (mass, myMotionState, newRigidShape, localInertia);
		rbInfo.m_friction = 0.8f;
		btRigidBody* tableLegRigid = new btRigidBody(rbInfo);
		tableLegRigid->setRestitution(0.0f);
		tableLegRigid->setUserPointer(tableLegNode);

		mPhysics->addRigidBody(tableLegRigid);
		mPhysics->getDynamicsWorld()->addRigidBody(tableLegRigid);
	}

}


void GT_ProjectApplication::spawnCans(void) {

	//prepare a manual cylinder object
	Ogre::ManualObject manualObject("ManualObject");
    manualObject.setQueryFlags(0);
    manualObject.clear();
    manualObject.setDynamic(false);
    manualObject.estimateVertexCount(Cylinder::vertexCount());
    manualObject.estimateIndexCount(Cylinder::indexCount());
    Cylinder::add(manualObject, "Examples/BumpyMetal", Ogre::ColourValue(1,0,0), Ogre::Quaternion());

	//create the cans
	Ogre::Entity* can;
	Ogre::SceneNode* canNode;
	btCollisionShape* newRigidShape = new btCylinderShape(btVector3(2/2, 4/2, 0));
	btTransform startTransform;
	startTransform.setIdentity();
	btScalar mass = 0.8f;	//can mass
	btVector3 localInertia = btVector3(0.0f,0.0f,0.0f);
	btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);
	btRigidBody::btRigidBodyConstructionInfo rbInfo (mass, myMotionState, newRigidShape, localInertia);
	btRigidBody* body;

	//bottom row
	for (int i = 0; i<8 ; i++) {

		std::string objectName = "CanBottom" + std::to_string(static_cast<long long>(i));
		std::string nodeName = "CanBottomNode" + std::to_string(static_cast<long long>(i));

		float posX = -9 + 0.25f + i*2.5f;	//position + margin + offset (can width + 0.5 space between cans)
		float posY = 7.25f;
		float posZ = 0.0f;


		manualObject.convertToMesh(objectName);

		//Ogre
		can = mSceneMgr->createEntity(objectName);
		canNode = mSceneMgr->getRootSceneNode()->createChildSceneNode(nodeName);
		canNode->attachObject(can);
		canNode->setPosition(Ogre::Vector3(0.0f,-999.0f,0.0f));	//far below the ground, so that no mesh can be seen in the time between creating the Ogre object and Bullet setting its position
	
		//Bullet
		newRigidShape = new btCylinderShape(btVector3(2/2, 4/2, 0));	//radius, height, zup (Bullet extends its models from a central point and therefore needs the half of the actual values)
		startTransform.setIdentity();
		startTransform.setOrigin(btVector3(posX, posY, posZ));
	
		newRigidShape->calculateLocalInertia(mass, localInertia);	//calculates local inertia by using the mass and body shape and saves it in localInertia

		myMotionState = new btDefaultMotionState(startTransform);
		rbInfo = btRigidBody::btRigidBodyConstructionInfo (mass, myMotionState, newRigidShape, localInertia);
		rbInfo.m_friction = 0.8f;
		body = new btRigidBody(rbInfo);
		body->setRestitution(0.0f);
		body->setUserPointer(canNode);

		mPhysics->addCan(body);
		mPhysics->getDynamicsWorld()->addRigidBody(body);
	}

	//middle row
	for (int i = 0; i<7 ; i++) {

		if (i != 3) {

			std::string objectName = "CanMiddle" + std::to_string(static_cast<long long>(i));
			std::string nodeName = "CanMiddleNode" + std::to_string(static_cast<long long>(i));

			float posX = -9 + 1.5f + i*2.5f;
			float posY = 11.25f;
			float posZ = 0.0f;


			manualObject.convertToMesh(objectName);

			//Ogre
			can = mSceneMgr->createEntity(objectName);
			canNode = mSceneMgr->getRootSceneNode()->createChildSceneNode(nodeName);
			canNode->attachObject(can);
			canNode->setPosition(Ogre::Vector3(0.0f,-999.0f,0.0f));	//far below the ground, so that no mesh can be seen in the time between creating the Ogre object and Bullet setting its position
	
			//Bullet
			newRigidShape = new btCylinderShape(btVector3(2/2, 4/2, 0));	//radius, height, zup (Bullet extends its models from a central point and therefore needs the half of the actual values)
			startTransform.setIdentity();
			startTransform.setOrigin(btVector3(posX, posY, posZ));
	
			newRigidShape->calculateLocalInertia(mass, localInertia);	//calculates local inertia by using the mass and body shape and saves it in localInertia

			myMotionState = new btDefaultMotionState(startTransform);
			rbInfo = btRigidBody::btRigidBodyConstructionInfo (mass, myMotionState, newRigidShape, localInertia);
			rbInfo.m_friction = 0.8f;
			body = new btRigidBody(rbInfo);
			body->setRestitution(0.0f);
			body->setUserPointer(canNode);

			mPhysics->addCan(body);
			mPhysics->getDynamicsWorld()->addRigidBody(body);
		}
	}

	//top row
	for (int i = 0; i<6 ; i++) {

		if (i != 2 && i != 3) {

			std::string objectName = "CanTop" + std::to_string(static_cast<long long>(i));
			std::string nodeName = "CanTopNode" + std::to_string(static_cast<long long>(i));

			float posX = -9 + 2.75f + i*2.5f;
			float posY = 15.25f;
			float posZ = 0.0f;


			manualObject.convertToMesh(objectName);

			//Ogre
			can = mSceneMgr->createEntity(objectName);
			canNode = mSceneMgr->getRootSceneNode()->createChildSceneNode(nodeName);
			canNode->attachObject(can);
			canNode->setPosition(Ogre::Vector3(0.0f,-999.0f,0.0f));	//far below the ground, so that no mesh can be seen in the time between creating the Ogre object and Bullet setting its position
	
			//Bullet
			newRigidShape = new btCylinderShape(btVector3(2/2, 4/2, 0));	//radius, height, zup (Bullet extends its models from a central point and therefore needs the half of the actual values)
			startTransform.setIdentity();
			startTransform.setOrigin(btVector3(posX, posY, posZ));
	
			newRigidShape->calculateLocalInertia(mass, localInertia);	//calculates local inertia by using the mass and body shape and saves it in localInertia

			myMotionState = new btDefaultMotionState(startTransform);
			rbInfo = btRigidBody::btRigidBodyConstructionInfo (mass, myMotionState, newRigidShape, localInertia);
			rbInfo.m_friction = 0.8f;
			body = new btRigidBody(rbInfo);
			body->setRestitution(0.0f);
			body->setUserPointer(canNode);

			mPhysics->addCan(body);
			mPhysics->getDynamicsWorld()->addRigidBody(body);
		}
	}
}


void GT_ProjectApplication::respawnCans(void) {

	btRigidBody* can = NULL;
	std::vector<btRigidBody*> cans = mPhysics->getCans();

	int bodyCount = -1;

	//bottom row
	for (int i = 0; i<8 ; i++) {
		bodyCount++;

		float posX = -9 + 0.25f + i*2.5f;	//position + margin + offset (can width + 0.5 space between cans)
		float posY = 7.25f;
		float posZ = 0.0f;

		can = cans.at(bodyCount);
			
		can->setAngularVelocity(btVector3(0,0,0));
		can->setLinearVelocity(btVector3(0,0,0));

		btTransform startTransform;
		startTransform.setIdentity();
		startTransform.setOrigin(btVector3(posX, posY, posZ));

		btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);
		can->setMotionState(myMotionState);
	}
	//middle row
	for (int i = 0; i<7 ; i++) {
		if (i != 3) {
			bodyCount++;

			float posX = -9 + 1.5f + i*2.5f;
			float posY = 11.25f;
			float posZ = 0.0f;

			can = cans.at(bodyCount);
				
			can->setAngularVelocity(btVector3(0,0,0));
			can->setLinearVelocity(btVector3(0,0,0));

			btTransform startTransform;
			startTransform.setIdentity();
			startTransform.setOrigin(btVector3(posX, posY, posZ));

			btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);
			can->setMotionState(myMotionState);
		}
	}
	//top row
	for (int i = 0; i<6 ; i++) {
		if (i != 2 && i != 3) {
			bodyCount++;

			float posX = -9 + 2.75f + i*2.5f;
			float posY = 15.25f;
			float posZ = 0.0f;

			can = cans.at(bodyCount);

			can->setAngularVelocity(btVector3(0,0,0));
			can->setLinearVelocity(btVector3(0,0,0));

			btTransform startTransform;
			startTransform.setIdentity();
			startTransform.setOrigin(btVector3(posX, posY, posZ));

			btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);
			can->setMotionState(myMotionState);
		}
	}
}


void GT_ProjectApplication::despawnCubes(void) {

	//remove cubes from physics engine
	std::vector<btRigidBody*> cubes = mPhysics->getCubes();
	btRigidBody* body;
	for (unsigned i = 0; i<cubes.size(); i++) {
		body = cubes.at(i);
		mPhysics->getDynamicsWorld()->removeRigidBody(body);
	}

	//remove cubes from Ogre
	for (int i = 0; i<mPhysics->getCubeCount(); i++) {	
		//get the name of the SceneNode
		std::string cubeNodeName = "CubeNode" + std::to_string(static_cast<long long>(i));
		Ogre::SceneNode* node = mSceneMgr->getSceneNode(cubeNodeName);
		if (node) {
			//destroy all of the node's attached objects (in this case there should only be one cube entity entached anyway)
			Ogre::SceneNode::ObjectIterator itObject = node->getAttachedObjectIterator();
			while(itObject.hasMoreElements()) {
				Ogre::MovableObject* pObject = static_cast<Ogre::MovableObject*>(itObject.getNext());
				node->getCreator()->destroyMovableObject(pObject);
			}
			//destroy the SceneNode
			mSceneMgr->destroySceneNode(cubeNodeName);
		}
	}
	mPhysics->removeCubes();
}


void GT_ProjectApplication::drawHUDCrosshair(void) {

	//get aspect ratio
	float x = (float)mWindow->getViewport(0)->getActualHeight();
	float y = (float)mWindow->getViewport(0)->getActualWidth();
	float aspectRatio = x/y;

	//create the crosshair
	Ogre::ManualObject* crosshairX = create2DBox("crossHairX", Ogre::ColourValue(0,0,0));
    prepManualObjectForHud(crosshairX);

	// Attach to scene
	Ogre::SceneNode* crossHairNodeX = mSceneMgr->getRootSceneNode()->createChildSceneNode(crosshairX->getName());
	crossHairNodeX->attachObject(crosshairX);
	crossHairNodeX->setScale(0.05f * aspectRatio, .001f, 1);	//multiply with aspect ratio to make crosshairX the same length as crosshairY
	crossHairNodeX->setVisible(true);

	//create the crosshair
	Ogre::ManualObject* crosshairY = create2DBox("crossHairY", Ogre::ColourValue(0,0,0));
    prepManualObjectForHud(crosshairY);

	// Attach to scene
	Ogre::SceneNode* crossHairNodeY = mSceneMgr->getRootSceneNode()->createChildSceneNode(crosshairY->getName());
	crossHairNodeY->attachObject(crosshairY);
	crossHairNodeY->setScale(0.001f, 0.05f, 1);
	crossHairNodeY->setVisible(true);
}



void GT_ProjectApplication::drawHUDCubeTypes(void) {

	//get aspect ratio
	float x = (float)mWindow->getViewport(0)->getActualHeight();
	float y = (float)mWindow->getViewport(0)->getActualWidth();
	float aspectRatio = x/y;

	//create the small cube type
	Ogre::ManualObject* smallCube = create2DBox("smallCube", Ogre::ColourValue(0.0f, 0.0f, 0.0f));
    prepManualObjectForHud(smallCube);
	Ogre::SceneNode* smallCubeNode = mSceneMgr->getRootSceneNode()->createChildSceneNode(smallCube->getName());
	smallCubeNode->attachObject(smallCube);
	smallCubeNode->setScale(0.01f * aspectRatio, 0.01f, 1);
	smallCubeNode->setVisible(false);
	smallCubeNode->setPosition(Ogre::Vector3(-0.9f, 0.9f, 0));	//upper left corner

	//create the selected small cube type
	Ogre::ManualObject* smallCubeSelected = create2DBox("smallCubeSelected", Ogre::ColourValue(1.0f, 0.0f, 0.0f));
    prepManualObjectForHud(smallCubeSelected);
	Ogre::SceneNode* smallCubeSelectedNode = mSceneMgr->getRootSceneNode()->createChildSceneNode(smallCubeSelected->getName());
	smallCubeSelectedNode->attachObject(smallCubeSelected);
	smallCubeSelectedNode->setScale(0.01f * aspectRatio, 0.01f, 1);
	smallCubeSelectedNode->setVisible(true);
	smallCubeSelectedNode->setPosition(Ogre::Vector3(-0.9f, 0.9f, 0));	//upper left corner
	
	//create the medium cube type
	Ogre::ManualObject* mediumCube = create2DBox("mediumCube", Ogre::ColourValue(0.0f, 0.0f, 0.0f));
    prepManualObjectForHud(mediumCube);
	Ogre::SceneNode* mediumCubeNode = mSceneMgr->getRootSceneNode()->createChildSceneNode(mediumCube->getName());
	mediumCubeNode->attachObject(mediumCube);
	mediumCubeNode->setScale(0.02f * aspectRatio, 0.02f, 1);
	mediumCubeNode->setVisible(true);
	mediumCubeNode->setPosition(Ogre::Vector3(-0.85f, 0.91f, 0));	//upper left corner
	
	//create the selected medium cube type
	Ogre::ManualObject* mediumCubeSelected = create2DBox("mediumCubeSelected", Ogre::ColourValue(1.0f, 0.0f, 0.0f));
    prepManualObjectForHud(mediumCubeSelected);
	Ogre::SceneNode* mediumCubeSelectedNode = mSceneMgr->getRootSceneNode()->createChildSceneNode(mediumCubeSelected->getName());
	mediumCubeSelectedNode->attachObject(mediumCubeSelected);
	mediumCubeSelectedNode->setScale(0.02f * aspectRatio, 0.02f, 1);
	mediumCubeSelectedNode->setVisible(false);
	mediumCubeSelectedNode->setPosition(Ogre::Vector3(-0.85f, 0.91f, 0));	//upper left corner

	//create the big cube type
	Ogre::ManualObject* bigCube = create2DBox("bigCube", Ogre::ColourValue(0.0f, 0.0f, 0.0f, 1.0f));
    prepManualObjectForHud(bigCube);
	Ogre::SceneNode* bigCubeNode = mSceneMgr->getRootSceneNode()->createChildSceneNode(bigCube->getName());
	bigCubeNode->attachObject(bigCube);
	bigCubeNode->setScale(0.03f * aspectRatio, 0.03f, 1);
	bigCubeNode->setVisible(true);
	bigCubeNode->setPosition(Ogre::Vector3(-0.78f, 0.92f, 0));	//upper left corner
	
	//create the big medium cube type
	Ogre::ManualObject* bigCubeSelected = create2DBox("bigCubeSelected", Ogre::ColourValue(1.0f, 0.0f, 0.0f));
    prepManualObjectForHud(bigCubeSelected);
	Ogre::SceneNode* bigCubeSelectedNode = mSceneMgr->getRootSceneNode()->createChildSceneNode(bigCubeSelected->getName());
	bigCubeSelectedNode->attachObject(bigCubeSelected);
	bigCubeSelectedNode->setScale(0.03f * aspectRatio, 0.03f, 1);
	bigCubeSelectedNode->setVisible(false);
	bigCubeSelectedNode->setPosition(Ogre::Vector3(-0.78f, 0.92f, 0));	//upper left corner

}

void GT_ProjectApplication::drawHUDMovementStatus(void) {

	//get aspect ratio
	float x = (float)mWindow->getViewport(0)->getActualHeight();
	float y = (float)mWindow->getViewport(0)->getActualWidth();
	float aspectRatio = x/y;

	//create the arrow box part
	Ogre::ManualObject* arrowBox = create2DBox("arrowBox", Ogre::ColourValue(0.0f, 0.0f, 0.0f));
    prepManualObjectForHud(arrowBox);
	Ogre::SceneNode* arrowBoxNode = mSceneMgr->getRootSceneNode()->createChildSceneNode(arrowBox->getName());
	arrowBoxNode->attachObject(arrowBox);
	arrowBoxNode->setScale(0.02f * aspectRatio, 0.004f, 1);
	arrowBoxNode->setVisible(true);
	arrowBoxNode->setPosition(Ogre::Vector3(0.9f, 0.9f, 0));	//upper right corner

	//create the arrow triangle part
	Ogre::ManualObject* arrowTriangle = create2DTriangle("arrowTriangle", Ogre::ColourValue(0.0f, 0.0f, 0.0f));
    prepManualObjectForHud(arrowTriangle);
	Ogre::SceneNode* arrowTriangleNode = mSceneMgr->getRootSceneNode()->createChildSceneNode(arrowTriangle->getName());
	arrowTriangleNode->attachObject(arrowTriangle);
	arrowTriangleNode->setScale(0.02f * aspectRatio, 0.02f, 1);
	arrowTriangleNode->setVisible(true);
	arrowTriangleNode->setPosition(Ogre::Vector3(0.92f, 0.9f, 0));	//upper right corner

	//create the "movement disabled" line
	Ogre::ManualObject* disabledBox = create2DBox("disabledMovementBox", Ogre::ColourValue(1.0f, 0.0f, 0.0f));
    prepManualObjectForHud(disabledBox);
	Ogre::SceneNode* disabledBoxNode = mSceneMgr->getRootSceneNode()->createChildSceneNode(disabledBox->getName());
	disabledBoxNode->attachObject(disabledBox);
	disabledBoxNode->setScale(0.05f * aspectRatio, 0.004f, 1);
	disabledBoxNode->roll(Ogre::Degree(315.0f));
	disabledBoxNode->setVisible(true);
	disabledBoxNode->setPosition(Ogre::Vector3(0.91f, 0.9f, 0));	//upper right corner
}

void GT_ProjectApplication::drawHUDForceMeter(void) {

	//get aspect ratio
	float x = (float)mWindow->getViewport(0)->getActualHeight();
	float y = (float)mWindow->getViewport(0)->getActualWidth();
	float aspectRatio = x/y;
	
	//create the empty box part
	Ogre::ManualObject* forceEmptyBox = create2DBox("forceEmptyBox", Ogre::ColourValue(0.0f, 0.0f, 0.0f));
    prepManualObjectForHud(forceEmptyBox);
	Ogre::SceneNode* forceEmptyBoxNode = mSceneMgr->getRootSceneNode()->createChildSceneNode(forceEmptyBox->getName());
	forceEmptyBoxNode->attachObject(forceEmptyBox);
	forceEmptyBoxNode->setScale(0.03f * aspectRatio, 0.1f, 1);
	forceEmptyBoxNode->setVisible(true);
	forceEmptyBoxNode->setPosition(Ogre::Vector3(-0.9f, 0.0f, 0));	//middle left side

	//create the empty box part
	Ogre::ManualObject* forceChargingBox = create2DBox("forceChargingBox", Ogre::ColourValue(1.0f, 0.0f, 0.0f));
    prepManualObjectForHud(forceChargingBox);
	Ogre::SceneNode* forceChargingBoxNode = mSceneMgr->getRootSceneNode()->createChildSceneNode(forceChargingBox->getName());
	forceChargingBoxNode->attachObject(forceChargingBox);
	forceChargingBoxNode->setScale(0.03f * aspectRatio, 0.0f, 1);
	forceChargingBoxNode->setVisible(true);
	forceChargingBoxNode->setPosition(Ogre::Vector3(-0.9f, 0.0f, 0));	//middle left side
}


Ogre::ManualObject* GT_ProjectApplication::create2DBox(std::string name, Ogre::ColourValue & colour) {

	// Create a manual object for 2D
	Ogre::ManualObject* manual = mSceneMgr->createManualObject(name);
	std::string materialName = "BaseWhiteNoLighting";	//use this material to display colors brightly

	manual->begin(materialName,Ogre::RenderOperation::OT_TRIANGLE_FAN); //2d square

	manual->position(-1, -1, 0); //lower left
	manual->colour(colour);
	//manual->textureCoord(0, 0, 0);
	manual->position(-1,  1, 0); //upper left
	manual->colour(colour);
	//manual->textureCoord(0, -1, 0);
	manual->position( 1,  1, 0); //upper right
	manual->colour(colour);
	//manual->textureCoord(1, -1, 0);
	manual->position( 1, -1, 0); //lower right
	manual->colour(colour);
	//manual->textureCoord(1, 0, 0);

	manual->index(3);
	manual->index(2);
	manual->index(1);
	manual->index(0);
	manual->end();

	return manual;
}


Ogre::ManualObject* GT_ProjectApplication::create2DTriangle(std::string name, Ogre::ColourValue & colour) {

	// Create a manual object for 2D
	Ogre::ManualObject* manual = mSceneMgr->createManualObject(name);
	std::string materialName = "BaseWhiteNoLighting";	//use this material to display colors brightly

	manual->begin(materialName,Ogre::RenderOperation::OT_TRIANGLE_LIST); //2d triangle

	manual->position(-1, -1, 0); //lower left
	manual->colour(colour);
	//manual->textureCoord(0, 0, 0);
	manual->position(-1,  1, 0); //upper left
	manual->colour(colour);
	//manual->textureCoord(0, -1, 0);
	manual->position( 1, 0.0f, 0); //middle right
	manual->colour(colour);
	//manual->textureCoord(1, -1, 0);

	manual->triangle(2,1,0);
	manual->end();

	return manual;
}



void GT_ProjectApplication::prepManualObjectForHud(Ogre::ManualObject* manual) {
   // Use identity view/projection matrices
   manual->setUseIdentityProjection(true);
   manual->setUseIdentityView(true);

   // Use infinite AAB to always stay visible
   Ogre::AxisAlignedBox aabInf;
   aabInf.setInfinite();
   manual->setBoundingBox(aabInf);

   // Render just before overlays
   manual->setRenderQueueGroup(Ogre::RENDER_QUEUE_OVERLAY - 1);
}



unsigned int Cylinder::vertexCount() { return n*4; }

unsigned int Cylinder::indexCount()  { return n*6; }

void Cylinder::add(Ogre::ManualObject & manualObject,
            const Ogre::String & material, 
            const Ogre::ColourValue & colour,
            const Ogre::Quaternion & orientation) {

    float thickness = 1;
    float length = 4;
      
    Ogre::Radian delta(Ogre::Degree(360/float(n)));
	Ogre::Quaternion rot(delta, Ogre::Vector3(0,1,0));

	Ogre::Vector3 originTop (0, -length/2, 0);
	Ogre::Vector3 originBottom (0, length/2, 0);
    Ogre::Vector3 p0 (thickness, -length/2, 0);
    Ogre::Vector3 p1 (thickness, length/2, 0);
    Ogre::Vector3 p2 = rot* p0;
    Ogre::Vector3 p3 = rot* p1;

    manualObject.begin(material, Ogre::RenderOperation::OT_TRIANGLE_LIST);
    for (int i = 0; i < n; ++i) {

		//set up vertex positions and colours for triangle drawing
        manualObject.position(orientation*originTop);
		manualObject.textureCoord(0,0);
        //manualObject.colour(Ogre::ColourValue(1,1,0));
        manualObject.position(orientation*p0);
		manualObject.textureCoord(0,0);
        //manualObject.colour(colour);
        manualObject.position(orientation*p1);
		manualObject.textureCoord(1,0);
        //manualObject.colour(colour);
        manualObject.position(orientation*p2);
		manualObject.textureCoord(0,1);
        //manualObject.colour(colour);
        manualObject.position(orientation*p3);
		manualObject.textureCoord(1,1);
        //manualObject.colour(colour);
        manualObject.position(orientation*originBottom);
		manualObject.textureCoord(1,1);
        //manualObject.colour(Ogre::ColourValue(1,1,0));

        int offset = i * 6;	//6 available vertices * position in array of all vertices (i)
        manualObject.triangle(offset+3, offset+1, offset);		//draw top area triangle
        manualObject.triangle(offset+1, offset+3, offset+4);	//draw side area triangle 1
        manualObject.triangle(offset+4, offset+2, offset+1);	//draw side area triangle 2
        manualObject.triangle(offset+2, offset+4, offset+5);	//draw bottom area triangle
		
		//rotate vertex positions to draw the next part of the cylinder
		originTop = rot * originTop;
		originBottom = rot * originBottom;
        p0 = rot * p0;
        p1 = rot * p1;
        p2 = rot * p2;
        p3 = rot * p3;
    }
    manualObject.end();
}