#include "terrainstorage.hpp"

#include <stdexcept>

#include <OgreVector2.h>
#include <OgreRoot.h>

namespace Terrain
{

TerrainStorage::TerrainStorage()
    : mMinHeight(std::numeric_limits<float>::max())
    , mMaxHeight(std::numeric_limits<float>::min())
    , mHeightmapSize(0)
    , mBlendmapSize(0)
{

}

Terrain::LayerInfo TerrainStorage::getDefaultLayer()
{
    Terrain::LayerInfo info;
    info.mDiffuseMap = "lava.png";
    info.mNormalMap = "lava_nh.png";
    info.mParallax = true;
    info.mSpecular = false;
    return info;
}

const int cellVerts = 33;

float TerrainStorage::getCellWorldSize()
{
    int numcells = getHeightmapSize() / (getCellVertices()-1);
    return getWorldSize() / numcells;
}

int TerrainStorage::getCellVertices()
{
    return cellVerts;
}

void TerrainStorage::getTriangleAt(const Ogre::Vector3& worldPos, Ogre::Plane& plane, float& height)
{
    float worldSize = getWorldSize();
    Ogre::Vector3 normalizedPos = worldPos / getWorldSize();
    normalizedPos += 0.5; // 0 .. 1

    float nX = normalizedPos.x;
    float nY = normalizedPos.z;

    int hmapSize = getHeightmapSize();

    //float factor = (float)hmapSize - 1.0f;
    float factor = hmapSize;
    float invFactor = 1.0f / factor;

    int startX = static_cast<int>(nX * factor);
    int startY = static_cast<int>(nY * factor);

    int endX = startX+1;
    int endY = startY+1;

    // now get points in terrain space (effectively rounding them to boundaries)
    // note that we do not clamp! We need a valid plane
    float startXTS = startX * invFactor;
    float startYTS = startY * invFactor;
    float endXTS = endX * invFactor;
    float endYTS = endY * invFactor;

    // now clamp
    startX = std::max(0, std::min(startX, hmapSize-1));
    startY = std::max(0, std::min(startY, hmapSize-1));
    endX = std::max(0, std::min(endX, hmapSize-1));
    endY = std::max(0, std::min(endY, hmapSize-1));

    // get parametric from start coord to next point
    float xParam = (nX - startXTS) / invFactor;
    float yParam = (nY - startYTS) / invFactor;

    Ogre::Vector3 v0 (startXTS * worldSize, startYTS * worldSize, getHeight(startX, startY));
    Ogre::Vector3 v1 (endXTS * worldSize, startYTS * worldSize, getHeight(endX, startY));
    Ogre::Vector3 v2 (endXTS * worldSize, endYTS * worldSize, getHeight(endX, endY));
    Ogre::Vector3 v3 (startXTS * worldSize, endYTS * worldSize, getHeight(startX, endY));

    bool secondTri = ((1.0 - yParam) > xParam);
    if (secondTri)
        plane.redefine(v0, v1, v3);
    else
        plane.redefine(v1, v2, v3);

    // Solve plane equation for z
    height = (-plane.normal.x * nX * worldSize
            -plane.normal.y * nY * worldSize
            - plane.d) / plane.normal.z;
}

float TerrainStorage::getHeightAt(const Ogre::Vector3 &worldPos)
{
    Ogre::Plane plane;
    float height;
    getTriangleAt(worldPos, plane, height);
}

Ogre::Vector3 TerrainStorage::getNormalAt(const Ogre::Vector3 &worldPos)
{
    Ogre::Plane plane;
    float height;
    getTriangleAt(worldPos, plane, height);
    return Ogre::Vector3(plane.normal.x, plane.normal.z, plane.normal.y);
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
        mHeightmapSize = hmapSize;
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

void TerrainStorage::ensureBlendmapLoaded()
{
    if (mBlendmap.empty())
    {
        mBlendmapSize = getBlendmapSize();

        mBlendmap.resize(mBlendmapSize*mBlendmapSize);
        loadBlendmap(mLayers, &mBlendmap[0]);
    }
}

float TerrainStorage::getHeight(int x, int y)
{
    int hmapSize = getHeightmapSize();
    x = std::max(0, std::min(hmapSize-1, x));
    y = std::max(0, std::min(hmapSize-1, y));
    return mHeightmap[y*hmapSize + x];
}

int TerrainStorage::getLayerIndex(int x, int y)
{
    x = std::max(0, std::min(mBlendmapSize-1, x));
    y = std::max(0, std::min(mBlendmapSize-1, y));
    return mBlendmap[y*mBlendmapSize + x];
}

Ogre::Vector3 TerrainStorage::getNormal(int x, int y)
{
    // FIXME: interpolate normal of the 4 closest faces

    if (x >= mHeightmapSize+1)
        --x;
    if (y >= mHeightmapSize+1)
        --y;

    float a = getHeight(x, y);
    float b = getHeight(x+1, y);
    float c = getHeight(x, y+1);

    // heightmap units to world units
    float scale = getWorldSize() / mHeightmapSize;

    Ogre::Plane plane;
    plane.redefine(Ogre::Vector3(x*scale, a, y*scale),
                   Ogre::Vector3(x*scale, c, (y+1)*scale), Ogre::Vector3((x+1)*scale, b, y*scale));
    return plane.normal;
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
            int inX = startX*cellSize + (x/float(numVerts-1)*size)*cellSize + (heightmapSize/2.f);
            int inY = startY*cellSize + (y/float(numVerts-1)*size)*cellSize + (heightmapSize/2.f);

            if (inX == heightmapSize)
                --inX;
            if (inY == heightmapSize)
                --inY;

            if (inX < 0 || inX >= heightmapSize || inY < 0 || inY >= heightmapSize)
            {
                std::cerr << "Warning: invalid heightmap cell, this should never happen. x=" << inX << ", y=" << inY << std::endl;
            }

            positions[x*numVerts*3 + y*3] = ((x/float(numVerts-1)-0.5) * size * getCellWorldSize());
            positions[x*numVerts*3 + y*3 + 1] = ((y/float(numVerts-1)-0.5) * size * getCellWorldSize());
            positions[x*numVerts*3 + y*3 + 2] = getHeight(inX, inY);

            // Adjust for wanted alignment
            convertPosition(align, positions[x*numVerts*3 + y*3], positions[x*numVerts*3 + y*3 + 1], positions[x*numVerts*3 + y*3 + 2]);

            Ogre::Vector3 normal = getNormal(inX, inY);
            normals[x*numVerts*3 + y*3] = normal.x;
            normals[x*numVerts*3 + y*3 + 1] = normal.y;
            normals[x*numVerts*3 + y*3 + 2] = normal.z;

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
    ensureBlendmapLoaded();

    Ogre::Vector2 origin = chunkCenter - Ogre::Vector2(chunkSize/2.f, chunkSize/2.f);
    int cellX = static_cast<int>(origin.x);
    int cellY = static_cast<int>(origin.y);

    if (chunkSize != 1)
    {
        std::cerr << "Unexpected chunk size " << chunkSize << " for blendmap " << std::endl;
    }

    // Blendmap size for this particular chunk
    int numcells = getHeightmapSize() / (getCellVertices()-1);
    int blendmapSize = mBlendmapSize / numcells * chunkSize;

    blendmapSize += 1; // blendmapSize+1 to smoothly blend into the next cell

    int blendmapStartX = cellX * mBlendmapSize / numcells + mBlendmapSize/2.f;
    int blendmapStartY = cellY * mBlendmapSize / numcells + mBlendmapSize/2.f;

    // Save the used texture indices so we know the total number of textures
    // and number of required blend maps
    std::set<int> layerIndices;

    for (int y=0; y<blendmapSize; ++y)
        for (int x=0; x<blendmapSize; ++x)
        {
            int layerIndex = getLayerIndex(blendmapStartX+x, blendmapStartY+y);
            layerIndices.insert(layerIndex);
        }

    // Due to the way the blending works, the base layer will always shine through in between
    // blend transitions (eg halfway between two texels, both blend values will be 0.5, so 25% of base layer visible).
    // To get a consistent look, we need to make sure to use the same base layer in all cells.
    // So we're always adding the base layer here, even if it's not referenced in this cell.

    // Not needed if no blending at all is performed.
    if (layerIndices.size() > 1)
        layerIndices.insert(0);

    // Makes sure the indices are sorted, or rather,
    // retrieved as sorted. This is important to keep the splatting order
    // consistent across cells.
    for (std::set<int>::iterator it = layerIndices.begin(); it != layerIndices.end(); ++it)
    {
        if (*it >= mLayers.size())
        {
            std::stringstream errormsg;
            errormsg << "Invalid layer '" << *it << "' referenced, only " << mLayers.size() << " layers are defined";
            throw std::runtime_error(errormsg.str());
        }
        layerList.push_back(mLayers[*it]);
    }

    int numLayers = layerIndices.size();
    // numLayers-1 since the base layer doesn't need blending
    int numBlendmaps = pack ? static_cast<int>(std::ceil((numLayers - 1) / 4.f)) : (numLayers - 1);

    int channels = pack ? 4 : 1;

    // Second iteration - create and fill in the blend maps

    for (int i=0; i<numBlendmaps; ++i)
    {
        Ogre::PixelFormat format = pack ? Ogre::PF_A8B8G8R8 : Ogre::PF_A8;

        Ogre::uchar* pData =
                        OGRE_ALLOC_T(Ogre::uchar, blendmapSize*blendmapSize*channels, Ogre::MEMCATEGORY_GENERAL);
        memset(pData, 0, blendmapSize*blendmapSize*channels);

        for (int y=0; y<blendmapSize; ++y)
        {
            for (int x=0; x<blendmapSize; ++x)
            {
                int layerIndex = getLayerIndex(blendmapStartX+x, blendmapStartY+y);
                int blendIndex = (pack ? static_cast<int>(std::floor((layerIndex - 1) / 4.f)) : layerIndex - 1);
                int channel = pack ? std::max(0, (layerIndex-1) % 4) : 0;

                if (blendIndex == i)
                    pData[y*blendmapSize*channels + x*channels + channel] = 255;
                else
                    pData[y*blendmapSize*channels + x*channels + channel] = 0;
            }
        }
        blendmaps.push_back(Ogre::PixelBox(blendmapSize, blendmapSize, 1, format, pData));
    }
}

}


