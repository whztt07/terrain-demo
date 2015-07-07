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

TerrainStorage::TerrainStorage()
    : mMinHeight(std::numeric_limits<float>::max())
    , mMaxHeight(std::numeric_limits<float>::min())
{

}

void TerrainStorage::getBounds(float& minX, float& maxX, float& minY, float& maxY)
{
    int numCells = getHeightmapSize() / (getCellVertices()-1);

    minX = -numCells/2;
    maxX = numCells/2;
    minY = -numCells/2;
    maxY = numCells/2;
}

bool TerrainStorage::getMinMaxHeights (float size, const Ogre::Vector2& center, float& min, float& max)
{
    ensureHeightmapLoaded();

    // TODO: get min/max heights of that particular chunk

    min = mMinHeight;
    max = mMaxHeight;

    return true;
}

void TerrainStorage::ensureHeightmapLoaded()
{
    if (mHeightmap.empty())
    {
        int hmapSize = getHeightmapSize();
        mHeightmap.resize(hmapSize*hmapSize);
        loadHeightmap(&mHeightmap[0]);

        for (int i=0; i<mHeightmap.size(); ++i)
        {
            if (mHeightmap[i] > mMaxHeight)
                mMaxHeight = mHeightmap[i];
            if (mHeightmap[i] < mMinHeight)
                mMinHeight = mHeightmap[i];
        }
    }
}

void TerrainStorage::fillVertexBuffers (int lodLevel, float size, const Ogre::Vector2& center, Terrain::Alignment align,
                                        std::vector<float>& positions,
                                        std::vector<float>& normals,
                                        std::vector<Ogre::uint8>& colours)
{
    ensureHeightmapLoaded();

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

    int heightmapSize = getHeightmapSize();
    int cellSize = getCellVertices()-1;

    for (int y = 0; y < numVerts; ++y)
    {
        for (int x = 0; x < numVerts; ++x)
        {
            const float scale = heightmapSize / static_cast<float>(heightmapSize+1);

            int inX = startX*cellSize + (x/float(numVerts-1)*size)*cellSize + (heightmapSize/2.f);
            int inY = startY*cellSize + (y/float(numVerts-1)*size)*cellSize + (heightmapSize/2.f);
            inX *= scale;
            inY *= scale;

            if (inX < 0 || inX >= heightmapSize || inY < 0 || inY >= heightmapSize)
            {
                std::cerr << "Warning: invalid heightmap cell, this should never happen. x=" << inX << ", y=" << inY << std::endl;
            }

            positions[x*numVerts*3 + y*3] = ((x/float(numVerts-1)-0.5) * size * getCellWorldSize());
            positions[x*numVerts*3 + y*3 + 1] = ((y/float(numVerts-1)-0.5) * size * getCellWorldSize());
            positions[x*numVerts*3 + y*3 + 2] = mHeightmap[inX*heightmapSize + inY];

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


