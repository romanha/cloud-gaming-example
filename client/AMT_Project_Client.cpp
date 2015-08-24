/*
-----------------------------------------------------------------------------
Filename:    GT_OgreApplication.cpp
-----------------------------------------------------------------------------                          
	  adapted from the Ogre Tutorial Framework
	  http://www.ogre3d.org/tikiwiki/
-----------------------------------------------------------------------------
*/
#include "AMT_Project_Client.h"
#include <string>
 
#if OGRE_PLATFORM == OGRE_PLATFORM_APPLE
#include <macUtils.h>
#endif

//-------------------------------------------------------------------------------------
AMT_Project_Client::AMT_Project_Client(void)
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
	mRenderTexture(0),
	mMiniScreen(0)
{
#if OGRE_PLATFORM == OGRE_PLATFORM_APPLE
    m_ResourcePath = Ogre::macBundlePath() + "/Contents/Resources/";
#else
    m_ResourcePath = "";
#endif
	
	//UDP and Video Codec
	xvidEncoder = new EncodeVideoXVID();	//create encoder
	xvidDecoder = new DecodeVideoXVID();	//create decoder
	UDPSender = new UDPSend();	//create UDP sender
	UDPReceiver = new UDPReceive();	//create UDP receiver

	//pointer for render texture
	pDest = NULL;

}
 
//-------------------------------------------------------------------------------------
AMT_Project_Client::~AMT_Project_Client(void)
{ 
	//Remove ourself as a Window listener
	Ogre::WindowEventUtilities::removeWindowEventListener(mWindow, this);
	windowClosed(mWindow);
	
	m_rtt_texture.setNull();	//free Ogre pointer before mRoot

	delete mRoot;

	delete xvidEncoder;	//delete encoder pointer
	delete xvidDecoder;	//delete decoder pointer
	//delete[] pDest;	//delete memory pointer for render texture
}

//-------------------------------------------------------------------------------------
void AMT_Project_Client::go(void)
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
bool AMT_Project_Client::setup(void)
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
bool AMT_Project_Client::configure(void)
{
	/// Config Dialog to select settings
	if(mRoot->showConfigDialog())
	{
		mWindow = mRoot->initialise(true, "AMT_NetworkGame Client");
		return true;
	} return false;
}
//-------------------------------------------------------------------------------------
void AMT_Project_Client::chooseSceneManager(void)
{
	/// Manages organisation and rendering of a scene
	mSceneMgr = mRoot->createSceneManager(Ogre::ST_GENERIC);
}
//-------------------------------------------------------------------------------------
void AMT_Project_Client::createCamera(void)
{
	/// The main camera
	mCamera = mSceneMgr->createCamera("PlayerCam");
	mCamera->setPosition(Ogre::Vector3(0,0,0));
	mCamera->lookAt(Ogre::Vector3(0,0,0));
	mCamera->setNearClipDistance(0.1f);
}
//-------------------------------------------------------------------------------------
void AMT_Project_Client::createFrameListener(void)
{
	/// Framelistener
	mRoot->addFrameListener(this);
}
//-------------------------------------------------------------------------------------
void AMT_Project_Client::destroyScene(void)
{

}
//-------------------------------------------------------------------------------------
void AMT_Project_Client::createViewports(void)
{
	/// Create a viewport, a rendering region for a render target
	Ogre::Viewport* vp = mWindow->addViewport(mCamera);
	vp->setBackgroundColour(Ogre::ColourValue(0,0,0));
 	mCamera->setAspectRatio(
		Ogre::Real(vp->getActualWidth()) / Ogre::Real(vp->getActualHeight()));
}
//-------------------------------------------------------------------------------------
void AMT_Project_Client::setupResources(void)
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
void AMT_Project_Client::loadResources(void)
{
	Ogre::ResourceGroupManager::getSingleton().initialiseAllResourceGroups();
}



void AMT_Project_Client::createOIS(void)
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

void AMT_Project_Client::createWindowEventListener(void)
{
	windowResized(mWindow);
	Ogre::WindowEventUtilities::addWindowEventListener(mWindow, this);
}

//-------------------------------------------------------------------------------------
bool AMT_Project_Client::frameRenderingQueued(const Ogre::FrameEvent& evt)
{

	m_YawAngle = Ogre::Degree(0.0);
	m_PitchAngle = Ogre::Degree(0.0);
	m_TranslateVector = Ogre::Vector3::ZERO;


	/*
	 * INPUT ACTIONS
	 */
	//PRESS SPACE TO SHOOT CUBES (HOLD SPACE TO SHOOT STRONGER)
	static bool buttonWasDownSpace = false;
	bool buttonIsDownSpace = mKeyboard->isKeyDown(OIS::KC_SPACE);

	if(buttonIsDownSpace) {
		sendKey("SPACE", 6);
	}

	if(!buttonIsDownSpace && buttonWasDownSpace) {
		sendKey("SPACEEND", 9);
	}
	buttonWasDownSpace = buttonIsDownSpace;


	//PRESS R TO RESPAWN CANS AND REMOVE ALL CUBES
	static bool buttonWasDownR = false;
	bool buttonIsDownR = mKeyboard->isKeyDown(OIS::KC_R);

	if(buttonIsDownR && !buttonWasDownR) {
		sendKey("R", 2);
	}
	buttonWasDownR = buttonIsDownR;


	//PRESS 1 TO SET CUBETYPE 1
	static bool buttonWasDown1 = false;
	bool buttonIsDown1 = mKeyboard->isKeyDown(OIS::KC_1);

	if(buttonIsDown1 && !buttonWasDown1) {
		sendKey("1", 2);
	}
	buttonWasDown1 = buttonIsDown1;

	//PRESS 2 TO SET CUBETYPE 2
	static bool buttonWasDown2 = false;
	bool buttonIsDown2 = mKeyboard->isKeyDown(OIS::KC_2);

	if(buttonIsDown2 && !buttonWasDown2) {
		sendKey("2", 2);
	}
	buttonWasDown2 = buttonIsDown2;

	//PRESS 3 TO SET CUBETYPE 3
	static bool buttonWasDown3 = false;
	bool buttonIsDown3 = mKeyboard->isKeyDown(OIS::KC_3);

	if(buttonIsDown3 && !buttonWasDown3) {
		sendKey("3", 2);
	}
	buttonWasDown3 = buttonIsDown3;

	//PRESS TAB TO SWITCH BETWEEN CAMERA POSITIONS (1, 2, 3 = fixed positions & 4 = free movement)
	static bool buttonWasDownTab = false;
	bool buttonIsDownTab = mKeyboard->isKeyDown(OIS::KC_TAB);

	if(buttonIsDownTab && !buttonWasDownTab) {
		sendKey("TAB", 4);
	}
	buttonWasDownTab = buttonIsDownTab;


	//ARROW KEYS FOR CAMERA ANGLE
	if(mKeyboard->isKeyDown(OIS::KC_UP))
		sendKey("UP", 3);
	if(mKeyboard->isKeyDown(OIS::KC_DOWN))
		sendKey("DOWN", 5);
	if(mKeyboard->isKeyDown(OIS::KC_LEFT))
		sendKey("LEFT", 5);
	if(mKeyboard->isKeyDown(OIS::KC_RIGHT))
		sendKey("RIGHT", 6);


	
	//W,A,S,D,Q,E FOR CAMERA MOVEMENT (ONLY IF ENABLED)
	if(mKeyboard->isKeyDown(OIS::KC_A))
		sendKey("A", 2);
	if(mKeyboard->isKeyDown(OIS::KC_D))
		sendKey("D", 2);
	if(mKeyboard->isKeyDown(OIS::KC_W))
		sendKey("W", 2);
	if(mKeyboard->isKeyDown(OIS::KC_S))
		sendKey("S", 2);
	if(mKeyboard->isKeyDown(OIS::KC_Q))
		sendKey("Q", 2);
	if(mKeyboard->isKeyDown(OIS::KC_E))
		sendKey("E", 2);


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
	 * RECEIVE TEXTURE, DECODE AND DRAWN TO MINISCREEN
	 */
	
	char* encodeDestination = new char[65000];
	double* ptime = new double(105);

	int receivedBytes = UDPReceiver->receive(encodeDestination, 1, ptime);


	int memory = (mWindow->getWidth() * mWindow->getHeight() * 24) / 8;	//number of pixels with 24 bit per pixel (RGB24)
	pDest = new char[memory];	//allocate memory 



	if (receivedBytes >= 0) {
		int decodeValue = xvidDecoder->dec_main(encodeDestination, receivedBytes, pDest, mWindow->getWidth());
		
		if (decodeValue >= 0) {
			m_decodeDest = Ogre::PixelBox(mWindow->getWidth(), mWindow->getHeight(), 1, Ogre::PF_B8G8R8, (void*)( pDest ) );

			m_rtt_texture->getBuffer()->blitFromMemory(m_decodeDest);
			mMiniScreen->setVisible(true);
		}
	}

	delete[] pDest;
	delete[] encodeDestination;
	delete ptime;


	return true;
}

//-------------------------------------------------------------------------------------
bool AMT_Project_Client::keyPressed( const OIS::KeyEvent &arg )
{ 
	if (arg.key == OIS::KC_ESCAPE) mShutDown=true;
	return true;
}
 
bool AMT_Project_Client::keyReleased( const OIS::KeyEvent &arg )
{
	return true;
}
 
bool AMT_Project_Client::mouseMoved( const OIS::MouseEvent &arg )
{
	return true;
}
 
bool AMT_Project_Client::mousePressed( const OIS::MouseEvent &arg, OIS::MouseButtonID id )
{
	return true;
}
 
bool AMT_Project_Client::mouseReleased( const OIS::MouseEvent &arg, OIS::MouseButtonID id )
{
	return true;
}


void AMT_Project_Client::sendKey(char* button, int length) {
	UDPSender->send(button, length);
}

void AMT_Project_Client::createScene(void) {
	
	//MINI SCREEN
	//create the mini screen
	mMiniScreen = new Ogre::Rectangle2D(true);
	mMiniScreen->setCorners(-1.0f, 1.0f, 1.0f, -1.0f);	//full screen
	mMiniScreen->setBoundingBox(Ogre::AxisAlignedBox::BOX_INFINITE);

	Ogre::SceneNode* miniScreenNode = mSceneMgr->getRootSceneNode()->createChildSceneNode();
	miniScreenNode->attachObject(mMiniScreen);
	
	//save render texture to main memory
	m_rtt_texture = Ogre::TextureManager::getSingleton()
		.createManual("RttTex", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME, Ogre::TEX_TYPE_2D,
						mWindow->getWidth(), mWindow->getHeight(), 0, Ogre::PF_R8G8B8, Ogre::TU_RENDERTARGET);	//create render texture

	mRenderTexture = m_rtt_texture->getBuffer()->getRenderTarget();

	//create material for miniScreen
	Ogre::MaterialPtr renderMaterial = Ogre::MaterialManager::getSingleton().create("RttMat", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
	renderMaterial->getTechnique(0)->getPass(0)->setLightingEnabled(false);
	renderMaterial->getTechnique(0)->getPass(0)->createTextureUnitState("RttTex");
	mMiniScreen->setMaterial("RttMat");


	//set size of buffer for decoding the texture
	int memory = (mWindow->getWidth() * mWindow->getHeight() * 24) / 8;	//number of pixels with 24 bit per pixel (RGB24)
	pDest = new char[memory];	//allocate memory 

	m_encodeDest = Ogre::PixelBox(mWindow->getWidth(), mWindow->getHeight(), 1, Ogre::PF_B8G8R8, (void*)( pDest ) );
	

	//initialize XVID global and encoder
	xvidEncoder->global_init(true);
	//xvidEncoder->enc_init(mWindow->getWidth(), mWindow->getHeight(), 128, 1, 20, 3);
	xvidDecoder->dec_init(mWindow->getWidth(), mWindow->getHeight());


	//init UDP sender
	UDPSender->startWinsock();
	UDPSender->init("127.0.0.1", 8089);	//stream input commands to localhost at port 8089

	//init UDP receiver
	UDPReceiver->startWinsock();
	UDPReceiver->init(8088);	//receive render texture at port 8088


}
 
//Adjust mouse clipping area
void AMT_Project_Client::windowResized(Ogre::RenderWindow* rw)
{
	unsigned int width, height, depth;
	int left, top;
	rw->getMetrics(width, height, depth, left, top);
 
	const OIS::MouseState &ms = mMouse->getMouseState();
	ms.width = width;
	ms.height = height;
}
void AMT_Project_Client::windowClosed(Ogre::RenderWindow* rw)
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

