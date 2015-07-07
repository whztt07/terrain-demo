#include "terrainstorage.hpp"

#include <OgreVector2.h>
#include <OgreRoot.h>

#include "Simplex.hpp"

namespace Terrain
{

Terrain::LayerInfo TerrainStorage::getDefaultLayer()
{
    Terrain::LayerInfo info;
    info.mDiffuseMap = "lava.png";
    info.mNormalMap = "lava_nh.png";
    info.mParallax = true;
    info.mSpecular = false;
    return info;
}

float TerrainStorage::getCellWorldSize()
{
    return 160;
}

int TerrainStorage::getCellVertices()
{
    return 17;
}

float TerrainStorage::getHeightAt(const Ogre::Vector3 &worldPos)
{
    return 0;
}

void TerrainStorage::getBounds(float& minX, float& maxX, float& minY, float& maxY)
{
    minX = -16;
    maxX = 16;
    minY = -16;
    maxY = 16;
}

bool TerrainStorage::getMinMaxHeights (float size, const Ogre::Vector2& center, float& min, float& max)
{
    min = -500;
    max = 500;
    return true;
}

void TerrainStorage::fillVertexBuffers (int lodLevel, float size, const Ogre::Vector2& center, Terrain::Alignment align,
                                        std::vector<float>& positions,
                                        std::vector<float>& normals,
                                        std::vector<Ogre::uint8>& colours)
{
    // LOD level n means every 2^n-th vertex is kept
    size_t increment = 1 << lodLevel;

    Ogre::Vector2 origin = center - Ogre::Vector2(size/2.f, size/2.f);
    assert(origin.x == (int) origin.x);
    assert(origin.y == (int) origin.y);

    int startX = origin.x;
    int startY = origin.y;

    size_t numVerts = size*(getCellVertices()-1)/increment + 1;

    colours.resize(numVerts*numVerts*4);
    positions.resize(numVerts*numVerts*3);
    normals.resize(numVerts*numVerts*3);

    Ogre::ColourValue color (1,1,1,1);

    /*
     *
[ ter_gen ]
freq = 0.914330
mul = 1.000000
oct = 4
ofsh = 0.000000
ofsx = 0.140625
ofsy = -1.546875
persist = 0.347266
pow = 1.000000
roadsm = 0.000000
scale = 52.041016
terMaxA = 90.000000
terMaxH = 300.000000
terMinA = 0.000000
terMinH = -300.000000
terSmA = 9.000000
terSmH = 20.000000
*/

    const float gen_ofsx = 0.140625;
    const float gen_ofsy = -1.546875;
    const float gen_freq = 0.914330;
    const float gen_oct = 4;
    const float gen_persist = 0.347266;
    const float gen_pow = 1;
    const float gen_terMinH = -300;
    const float gen_terMaxH = 300;
    const float gen_terSmH = 20;
    const float gen_mul = 1;
    const float gen_ofsh = 0;

    for (int y = 0; y < numVerts; ++y)
    {
        for (int x = 0; x < numVerts; ++x)
        {
            float h = sin( (startX + (x/float(numVerts-1)*size))*0.1 + cos( 0.1*(startY + (y/float(numVerts-1)*size)))) * 4;

            float inX = startX + (x/float(numVerts-1)*size);
            float inY = startY + (y/float(numVerts-1)*size);
            inY *= 0.1;
            inX *= 0.1;
            float c = Noise(inY - gen_ofsy, inX - gen_ofsx, gen_freq, gen_oct, gen_persist) * 0.8f;
            c = c >= 0.f ? powf(c, gen_pow) : -powf(-c, gen_pow);
            h += c * gen_mul;

            positions[x*numVerts*3 + y*3] = ((x/float(numVerts-1)-0.5) * size * getCellWorldSize());
            positions[x*numVerts*3 + y*3 + 1] = ((y/float(numVerts-1)-0.5) * size * getCellWorldSize());
            positions[x*numVerts*3 + y*3 + 2] = h * 100;
            // Adjust for wanted alignment
            convertPosition(align, positions[x*numVerts*3 + y*3], positions[x*numVerts*3 + y*3 + 1], positions[x*numVerts*3 + y*3 + 2]);

            normals[x*numVerts*3 + y*3] = 0;
            normals[x*numVerts*3 + y*3 + 1] = 1;
            normals[x*numVerts*3 + y*3 + 2] = 0;

            float noise = SimplexNoise2D(startX + (x/float(numVerts-1)*size), startY + (y/float(numVerts-1)*size));
            noise = 1 - ((1 - (noise+1)/2) * 0.6);
            color = Ogre::ColourValue(noise,noise,noise,1);

            Ogre::uint32 rsColor;
            Ogre::Root::getSingleton().getRenderSystem()->convertColourValue(color, &rsColor);
            memcpy(&colours[x*numVerts*4 + y*4], &rsColor, sizeof(Ogre::uint32));
        }
    }
}

void TerrainStorage::getBlendmaps (float chunkSize, const Ogre::Vector2& chunkCenter, bool pack,
                   std::vector<Ogre::PixelBox>& blendmaps,
                   std::vector<Terrain::LayerInfo>& layerList)
{
    Terrain::LayerInfo layer;
    layer.mDiffuseMap = "lava.png";
    layer.mParallax = false;
    layer.mSpecular = false;
    layerList.push_back(layer);
}

}


