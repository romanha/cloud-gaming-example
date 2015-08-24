/*
-----------------------------------------------------------------------------
Filename:    OgreApplication.cpp
-----------------------------------------------------------------------------                          
	  adapted from the Ogre Tutorial Framework
	  http://www.ogre3d.org/tikiwiki/
-----------------------------------------------------------------------------
*/

#ifndef __GT_ProjectApplication_h_
#define __GT_ProjectApplication_h_
 
#include <Overlay\OgreOverlaySystem.h>
#include <OgreCamera.h>
#include <OgreEntity.h>
#include <OgreLogManager.h>
#include <OgreRoot.h>
#include <OgreViewport.h>
#include <OgreSceneManager.h>
#include <OgreRenderWindow.h>
#include <OgreConfigFile.h>
#include <OgreLogManager.h>


 
#if OGRE_PLATFORM == OGRE_PLATFORM_APPLE
#  include <OIS/OISEvents.h>
#  include <OIS/OISInputManager.h>
#  include <OIS/OISKeyboard.h>
#  include <OIS/OISMouse.h>
 
#  include <OGRE/SdkTrays.h>
#  include <OGRE/SdkCameraMan.h>
#else
#  include <OISEvents.h>
#  include <OISInputManager.h>
#  include <OISKeyboard.h>
#  include <OISMouse.h>
 
#  include <SdkTrays.h>
#  include <SdkCameraMan.h>
#endif
 
#ifdef OGRE_STATIC_LIB
#  define OGRE_STATIC_GL
#  if OGRE_PLATFORM == OGRE_PLATFORM_WIN32
#    define OGRE_STATIC_Direct3D9
// dx10 will only work on vista, so be careful about statically linking
#    if OGRE_USE_D3D10
#      define OGRE_STATIC_Direct3D10
#    endif
#  endif
#  define OGRE_STATIC_BSPSceneManager
#  define OGRE_STATIC_ParticleFX
#  define OGRE_STATIC_CgProgramManager
#  ifdef OGRE_USE_PCZ
#    define OGRE_STATIC_PCZSceneManager
#    define OGRE_STATIC_OctreeZone
#  else
#    define OGRE_STATIC_OctreeSceneManager
#  endif
#  include "OgreStaticPluginLoader.h"
#endif
 

/*
 * Physics
 */
#include "Physics.h"

/*
 * UDP and Video Codec
 */
#include "UDPSend.h"
#include "UDPReceive.h"
#include "EncodeVideoXVID.h"


/*
 * Main Ogre Class
 */
class GT_ProjectApplication : public Ogre::FrameListener, public Ogre::WindowEventListener, public OIS::KeyListener, public OIS::MouseListener
{
public:
	GT_ProjectApplication(void);
	virtual ~GT_ProjectApplication(void);
 
	virtual void go(void);
 
protected:

	/// Own Methods

	virtual bool setup();
	virtual bool configure(void);
	virtual void chooseSceneManager(void);
	virtual void createCamera(void);
	virtual void createFrameListener(void);
	virtual void createOIS(void);
	virtual void createWindowEventListener(void);
	virtual void createScene(void);
	virtual void destroyScene(void);
	virtual void createViewports(void);
	virtual void setupResources(void);
	virtual void loadResources(void);

	/// Ogre::FrameListener Method
	virtual bool frameRenderingQueued(const Ogre::FrameEvent& evt);
 
	/// OIS::Keylistener Method
	virtual bool keyPressed( const OIS::KeyEvent &arg );
	virtual bool keyReleased( const OIS::KeyEvent &arg );

	/// OIS::Mouselistener Methods
	virtual bool mouseMoved( const OIS::MouseEvent &arg );
	virtual bool mousePressed( const OIS::MouseEvent &arg, OIS::MouseButtonID id );
	virtual bool mouseReleased( const OIS::MouseEvent &arg, OIS::MouseButtonID id );
 
	/// Ogre::WindowEventListener
	virtual void windowResized(Ogre::RenderWindow* rw);
	virtual void windowClosed(Ogre::RenderWindow* rw);
 
	//spawning stuff
	virtual void spawnTable(void);
	virtual void spawnCans(void);
	virtual void respawnCans(void);
	virtual void despawnCubes(void);

	//HUD drawing
	virtual void drawHUDCrosshair(void);
	virtual void drawHUDCubeTypes(void);
	virtual void drawHUDMovementStatus(void);
	virtual void drawHUDForceMeter(void);

	//HUD general object creation and preparation
	virtual Ogre::ManualObject* create2DBox(std::string name, Ogre::ColourValue & colour);
	virtual Ogre::ManualObject* create2DTriangle(std::string name, Ogre::ColourValue & colour);
	virtual void prepManualObjectForHud(Ogre::ManualObject* manual);

	//receive key commands
	virtual void receiveCommands();
	boost::shared_ptr<boost::thread> recThread;	//receiving thread
	//boost::thread recThread;

	//RenderTexture
	void* pDest;
	Ogre::RenderTexture* mRenderTexture;
	Ogre::TexturePtr m_rtt_texture;
	Ogre::PixelBox m_encodeDest;

	//UDP Sender and XVID Encoder
	UDPSend* UDPSender;
	UDPReceive* UDPReceiver;
	EncodeVideoXVID* xvidEncoder;

	Ogre::Root* mRoot;
	Ogre::Camera* mCamera;
	Ogre::SceneManager* mSceneMgr;
	Ogre::RenderWindow* mWindow;
	Ogre::String mResourcesCfg;
	Ogre::String mPluginsCfg;
 
	bool mShutDown;
 
	//OIS Input devices
	OIS::InputManager* mInputManager;
	OIS::Mouse*    mMouse;
	OIS::Keyboard* mKeyboard;

	//Physics
	Physics* mPhysics;

	//variable to switch between cube sizes and camera positions
	int mCubeType;
	int mCameraSpot;

	//variable to store throwing force
	float mThrowingForce;

	bool mReset;
	bool mUpPressed;
	bool mDownPressed;
	bool mLeftPressed;
	bool mRightPressed;
	bool mAPressed;
	bool mDPressed;
	bool mWPressed;
	bool mSPressed;
	bool mQPressed;
	bool mEPressed;

	// Added for Mac compatibility
	Ogre::String                 m_ResourcePath;

	// Variables for Camera Movement
	Ogre::Vector3				m_TranslateVector;
	Ogre::Real                  m_MoveSpeed; 
	Ogre::Degree				m_RotateSpeed; 
	float                       m_MoveScale; 
	Ogre::Degree				m_RotScale;
	Ogre::Radian				m_YawAngle;
	Ogre::Radian				m_PitchAngle;

 
#ifdef OGRE_STATIC_LIB
	Ogre::StaticPluginLoader m_StaticPluginLoader;
#endif
};
 


/*
 * Class to create a manual cylinder object.
 */
class Cylinder
{
public:
   static const unsigned int n = 16;	//amount of side areas (more side areas result in a smoother cylinder)
   static unsigned int vertexCount();
   static unsigned int indexCount();
   static void add(Ogre::ManualObject & manualObject,
              const Ogre::String & material, 
              const Ogre::ColourValue & colour,
              const Ogre::Quaternion & orientation);
};



#endif // #ifndef __GT_OgreApplication_h_