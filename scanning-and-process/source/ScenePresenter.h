/********************************************************************
*
*   Project     Artec 3D Scanning SDK Samples
*
*   Purpose:    Simple GLFW renderer for CompositeMesh object (description)
*
*   Copyright:  Artec Group
*
********************************************************************/
#pragma once
#include <memory>

#include <artec/sdk/base/TRef.h>
#include <artec/sdk/base/Point.h>
#include <artec/sdk/base/ICompositeMesh.h>
#include <artec/sdk/base/ITexture.h>
#include <artec/sdk/base/Errors.h>


namespace asdk {
    using namespace artec::sdk::base;
};
using asdk::TRef;

class ScenePresenter
{
public:
    ScenePresenter();

    asdk::ErrorCode init( const asdk::ICompositeMesh& mesh );

public:
    // returns whether the mesh is textured
    bool isTextured() const;

    // returns the center of the mesh (used for proper centering)
    asdk::Point3F getMassCenter() const;

    // returns the size of the mesh (used for proper scaling)
    asdk::Point3F getBoundingBoxSize() const;

    // returns the max size of the mesh (used for proper camera positioning)
    float getMaxSize() const;

    // action methods
    void prepareForRendering();
    void render() const;


private:
    asdk::Point3D massCenter_;
    asdk::Point3F boundingBoxSize_;

    int numberOfVertices_;
    std::vector<float> vertexCoords_;
    std::vector<float> normals_;
    std::vector<float> texCoords_;

    bool isTextured_;

    TRef<asdk::ITexture> meshTexture_;
};


asdk::ErrorCode DisplayScene( const asdk::ICompositeMesh& inputMesh );

void PrintDisplayHelp();
