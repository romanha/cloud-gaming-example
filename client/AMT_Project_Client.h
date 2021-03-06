/*
-----------------------------------------------------------------------------
Filename:    OgreApplication.cpp
-----------------------------------------------------------------------------                          
	  adapted from the Ogre Tutorial Framework
	  http://www.ogre3d.org/tikiwiki/
-----------------------------------------------------------------------------
*/

#ifndef __AMT_Project_Client_h_
#define __AMT_Project_Client_h_
 
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
 * UDP and Video Codec
 */
#include "UDPSend.h"
#include "UDPReceive.h"
#include "EncodeVideoXVID.h"
#include "DecodeVideoXVID.h"


/*
 * Main Ogre Class
 */
class AMT_Project_Client : public Ogre::FrameListener, public Ogre::WindowEventListener, public OIS::KeyListener, public OIS::MouseListener
{
public:
	AMT_Project_Client(void);
	virtual ~AMT_Project_Client(void);
 
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
	
	//key sending
	virtual void sendKey(char* key, int length);


	//Mini Screen
	Ogre::Rectangle2D* mMiniScreen;

	
	//RenderTexture
	void* pDest;
	Ogre::RenderTexture* mRenderTexture;
	Ogre::TexturePtr m_rtt_texture;
	Ogre::PixelBox m_encodeDest;
	Ogre::PixelBox m_decodeDest;

	//UDP and XVID
	UDPSend* UDPSender;
	UDPReceive* UDPReceiver;
	EncodeVideoXVID* xvidEncoder;
	DecodeVideoXVID* xvidDecoder;

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



#endif // #ifndef __AMT_Project_Client_h_