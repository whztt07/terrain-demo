#include "application.hpp"

#include <OgreRoot.h>
#include <OgreCamera.h>

#include <OgreResourceBackgroundQueue.h>
#include <OgreTimer.h>

#include "sdl4ogre/sdlwindowhelper.hpp"
#include "sdl4ogre/sdlinputwrapper.hpp"

#if TERRAIN_USE_SHADER
#include <extern/shiny/Main/Factory.hpp>
#include <extern/shiny/Platforms/Ogre/OgrePlatform.hpp>
#endif

#include "../terrain/world.hpp"
#include "terrainstorage.hpp"

Application::Application()
    : mInputWrapper(NULL)
    , mRoot(NULL)
    , mSDLWindow(NULL)
    , mOgreWindow(NULL)
    , mShutdown(false)
    , mSceneMgr(NULL)
    , mCamera(NULL)
    , mTerrain(NULL)
    , mFreeze(false)
{

}

Application::~Application()
{
    delete mInputWrapper;
}

void Application::run()
{
    Uint32 flags = SDL_INIT_VIDEO|SDL_INIT_NOPARACHUTE;
    if(SDL_Init(flags) != 0)
    {
        throw std::runtime_error("Could not initialize SDL! " + std::string(SDL_GetError()));
    }

    // Construct Ogre::Root
    // We handle our stuff manually, so don't want to use any of the files
    mRoot = OGRE_NEW Ogre::Root(
        /* plugins.cfg file*/	"",
        /* config file */ 		"",
        /* log file */ 			""
    );

    // Set render system
    mRoot->loadPlugin(OGRE_PLUGIN_DIR_REL + std::string("/RenderSystem_GL"));
    Ogre::RenderSystem* rs = mRoot->getRenderSystemByName("OpenGL Rendering Subsystem");
    mRoot->setRenderSystem(rs);

    // Fire up Ogre::Root
    mRoot->initialise(false);

    bool fullscreen=false;
    int w = 1024;
    int h = 768;
    const std::string title = "Large Terrain Demo";

    mSDLWindow = SDL_CreateWindow(title.c_str(), SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, w, h,
                     SDL_WINDOW_SHOWN | SDL_WINDOW_RESIZABLE);

    Ogre::NameValuePairList params;
    params.insert(std::make_pair("title", title));
    params.insert(std::make_pair("FSAA", "0"));
    params.insert(std::make_pair("vsync", "true"));
    SFO::SDLWindowHelper helper (mSDLWindow, w, h, title, fullscreen, params);
    mOgreWindow = helper.getWindow();

    Ogre::ResourceGroupManager::getSingleton().addResourceLocation("../media", "FileSystem");

    Ogre::ResourceGroupManager::getSingleton().declareResource("lava.png", "Texture", "General");

    Ogre::ResourceGroupManager::getSingleton().initialiseAllResourceGroups();

    mInputWrapper = new SFO::InputWrapper(mSDLWindow, mOgreWindow, true);
    mInputWrapper->setWindowEventCallback(this);
    mInputWrapper->setMouseEventCallback(this);
    mInputWrapper->setKeyboardEventCallback(this);
    mInputWrapper->setGrabPointer(true);
    mInputWrapper->setMouseRelative(true);

    mSceneMgr = mRoot->createSceneManager(Ogre::ST_GENERIC);
    mSceneMgr->setAmbientLight(Ogre::ColourValue(1,1,1,1));
    mCamera = mSceneMgr->createCamera("");
    Ogre::Viewport* vp = mOgreWindow->addViewport(mCamera);
    vp->setBackgroundColour(Ogre::ColourValue(0.1,0.1,0.1,1.0));

    Ogre::TextureManager::getSingleton().setDefaultNumMipmaps(8);
    Ogre::MaterialManager::getSingleton().setDefaultAnisotropy(8);
    Ogre::MaterialManager::getSingleton().setDefaultTextureFiltering(Ogre::TFO_ANISOTROPIC);

    // Create material system
#if TERRAIN_USE_SHADER
    sh::OgrePlatform* platform = new sh::OgrePlatform("General", "../media");
    sh::Factory* factory = new sh::Factory(platform);
    factory->setCurrentLanguage(sh::Language_GLSL);
    factory->loadAllFiles();
    factory->setGlobalSetting("fog", "false");
    factory->setGlobalSetting("shadows", "false");
    factory->setGlobalSetting("shadows_pssm", "false");
    factory->setGlobalSetting("viewproj_fix", "false");
    factory->setGlobalSetting("render_refraction", "false");
    factory->setGlobalSetting("render_composite_map", "false");
    factory->setGlobalSetting("num_lights", "1");
#endif

    mCamera->setPosition(0,400,0);

    // Create the terrain
    TerrainStorage* storage = new TerrainStorage();
    mTerrain = new Terrain::World(mSceneMgr, storage, 1, true, true, Terrain::Align_XZ, 1, 128);
    mTerrain->syncLoad();

    // Start the rendering loop
    mRoot->addFrameListener(this);
    mRoot->startRendering();

    // For the sake of simplicity, delete everything here.
    // Not very exception friendly, of course, so don't do this at home kids ;)
    delete mTerrain;
#if TERRAIN_USE_SHADER
    delete factory;
#endif

    mRoot->removeFrameListener(this);
    OGRE_DELETE mRoot;

    SDL_DestroyWindow(mSDLWindow);
    SDL_Quit();
}

void Application::windowResized(int x, int y)
{

}

void Application::windowClosed()
{
    mShutdown = true;
}

bool Application::frameRenderingQueued(const Ogre::FrameEvent &evt)
{
    mInputWrapper->capture(false);

    if (!mFreeze)
        mTerrain->update(mCamera->getRealPosition());

    // Camera movement
    Ogre::Vector3 movementVector (0,0,0);
    if (mInputWrapper->isKeyDown(SDL_GetScancodeFromKey(SDLK_w)))
        movementVector.z = -1;
    else if (mInputWrapper->isKeyDown(SDL_GetScancodeFromKey(SDLK_s)))
        movementVector.z = 1;
    if (mInputWrapper->isKeyDown(SDL_GetScancodeFromKey(SDLK_a)))
        movementVector.x = -1;
    else if (mInputWrapper->isKeyDown(SDL_GetScancodeFromKey(SDLK_d)))
        movementVector.x = 1;

    movementVector *= evt.timeSinceLastFrame * 200;

    if (mInputWrapper->isModifierHeld(KMOD_LSHIFT))
        movementVector *= 5;

    mCamera->moveRelative(movementVector);

    return !mShutdown;
}

void Application::keyPressed(const SDL_KeyboardEvent &arg)
{
    if (arg.keysym.sym == SDLK_c)
        mCamera->setPolygonMode(mCamera->getPolygonMode() == Ogre::PM_SOLID ? Ogre::PM_WIREFRAME : Ogre::PM_SOLID);
    if (arg.keysym.sym == SDLK_f)
        mFreeze = !mFreeze;
    if (arg.keysym.sym == SDLK_ESCAPE)
        mShutdown = true;
}

void Application::keyReleased(const SDL_KeyboardEvent &arg)
{

}

void Application::mouseMoved(const SFO::MouseMotionEvent &arg)
{
    mCamera->yaw(Ogre::Radian(-arg.xrel*0.01));
    mCamera->pitch(Ogre::Radian(-arg.yrel*0.01));
}

void Application::mousePressed(const SDL_MouseButtonEvent &arg, Uint8 id)
{

}

void Application::mouseReleased(const SDL_MouseButtonEvent &arg, Uint8 id)
{

}
