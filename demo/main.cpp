#include "application.hpp"

#include <OgreException.h>
#include <stdexcept>

int main(int argc, char* argv[])
{
    Application app;

    try
    {
        app.run();
    }
    catch (Ogre::Exception& e)
    {
        std::cerr << e.getFullDescription().c_str() << std::endl;
    }
    catch (std::exception& e)
    {
        std::cerr << e.what() << std::endl;
    }

    return 0;
}
