#ifndef LTMDEMO_APPLICATION_HPP
#define LTMDEMO_APPLICATION_HPP

#include <OgreWindowEventUtilities.h>
#include <OgreFrameListener.h>

#include "sdl4ogre/events.h"

namespace Ogre
{
    class Root;
    class RenderWindow;
    class SceneManager;
    class Camera;
}

namespace SFO
{
    class InputWrapper;
}

namespace Terrain
{
    class World;
}

class Application : public Ogre::WindowEventListener, public Ogre::FrameListener, public SFO::KeyListener, public SFO::MouseListener, public SFO::WindowListener
{
public:
    Application();
    ~Application();

    void run();

protected:
    virtual bool frameRenderingQueued(const Ogre::FrameEvent& evt);

    virtual void keyPressed(const SDL_KeyboardEvent &arg);
    virtual void keyReleased(const SDL_KeyboardEvent &arg);
    virtual void mouseMoved( const SFO::MouseMotionEvent &arg );
    virtual void mousePressed( const SDL_MouseButtonEvent &arg, Uint8 id );
    virtual void mouseReleased( const SDL_MouseButtonEvent &arg, Uint8 id );

    virtual void windowClosed ();
    virtual void windowResized (int x, int y);

    bool mShutdown;

    float mTimerPrintFPS;

    bool mFreeze;

    Ogre::Root* mRoot;

    Ogre::SceneManager* mSceneMgr;
    Ogre::Camera* mCamera;
    Terrain::World* mTerrain;

    SFO::InputWrapper* mInputWrapper;

    SDL_Window* mSDLWindow;
    Ogre::RenderWindow* mOgreWindow;
};

#endif
