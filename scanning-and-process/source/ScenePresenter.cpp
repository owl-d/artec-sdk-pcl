/********************************************************************
*
*   Project     Artec 3D Scanning SDK Samples
*
*   Purpose:    Simple GLFW renderer for CompositeMesh object (implementation)
*
*   Copyright:  Artec Group
*
********************************************************************/
#include "ScenePresenter.h"

#define GLFW_INCLUDE_GLU
#include <GLFW/glfw3.h>
#include <iostream>


//***********************************************
//    ScenePresenter class implementation
//
 
// This constant determines the size of a window to display 3D scene
const int PresenterWindowSize = 900;


ScenePresenter::ScenePresenter()
    : numberOfVertices_()
    , isTextured_()
{
}

asdk::Point3F ScenePresenter::getMassCenter() const
{
    return (asdk::Point3F)massCenter_;
}

asdk::Point3F ScenePresenter::getBoundingBoxSize() const
{
    return boundingBoxSize_;
}

float ScenePresenter::getMaxSize() const
{
    return std::max( std::max( boundingBoxSize_.x, boundingBoxSize_.y ), boundingBoxSize_.z );
}

bool ScenePresenter::isTextured() const
{
    return isTextured_;
}

// setting up data for the 3D scene renderer
asdk::ErrorCode ScenePresenter::init( const asdk::ICompositeMesh& mesh )
{

    asdk::IndexTriplet* meshTriangles = mesh.getTriangles()->getPointer();
    const int numTriangles = mesh.getTriangles()->getSize();

    const asdk::Point3F* meshPoints = mesh.getPoints()->getPointer();
    const int numPoints = mesh.getPoints()->getSize();

    if( mesh.getPointsNormals() == NULL )
    {
        asdk::ICompositeMesh& mutableMesh = const_cast<asdk::ICompositeMesh&>( mesh );
        mutableMesh.calculate( asdk::CM_PointsNormals_Default );
    }
    if( mesh.getPointsNormals() == NULL )
    {
        std::wcerr << L"No normals cannot be calculated when setting up the composite mesh." << std::endl;
        return asdk::ErrorCode_OperationFailed;
    }

    const asdk::Point3F* meshNormals = mesh.getPointsNormals()->getPointer();
    isTextured_ = mesh.isTextured();

    numberOfVertices_ = numTriangles *3;

    vertexCoords_.resize( numberOfVertices_ *3 );
    normals_.resize( numberOfVertices_ *3 );
    if( isTextured_ )
    {
        texCoords_.resize( numberOfVertices_ *2 );
    }


    // get mass center for the entire model
    massCenter_ = asdk::Point3D( 0, 0, 0 );
    for( int index = 0; index < numPoints; ++index )
    {
        massCenter_ += (asdk::Point3D)meshPoints[index];
    }
    massCenter_ /= numPoints;


    // prepare polymesh vertices, normals and texcoords into internal buffers
    int vertexIndex = 0;
    int normalIndex = 0;
    int texcoordIndex = 0;

    for( int index = 0; index < numTriangles; ++index )
    {
        asdk::IndexTriplet current = meshTriangles[index];

        vertexCoords_[vertexIndex++] = meshPoints[current.x].x;
        vertexCoords_[vertexIndex++] = meshPoints[current.x].y;
        vertexCoords_[vertexIndex++] = meshPoints[current.x].z;

        normals_[normalIndex++] = meshNormals[current.x].x;
        normals_[normalIndex++] = meshNormals[current.x].y;
        normals_[normalIndex++] = meshNormals[current.x].z;

        vertexCoords_[vertexIndex++] = meshPoints[current.y].x;
        vertexCoords_[vertexIndex++] = meshPoints[current.y].y;
        vertexCoords_[vertexIndex++] = meshPoints[current.y].z;

        normals_[normalIndex++] = meshNormals[current.y].x;
        normals_[normalIndex++] = meshNormals[current.y].y;
        normals_[normalIndex++] = meshNormals[current.y].z;

        vertexCoords_[vertexIndex++] = meshPoints[current.z].x;
        vertexCoords_[vertexIndex++] = meshPoints[current.z].y;
        vertexCoords_[vertexIndex++] = meshPoints[current.z].z;

        normals_[normalIndex++] = meshNormals[current.z].x;
        normals_[normalIndex++] = meshNormals[current.z].y;
        normals_[normalIndex++] = meshNormals[current.z].z;
    
        if( isTextured_ )
        {
            asdk::TriangleUV triangle = mesh.getTriangleUV(index);

            texCoords_[texcoordIndex++] = triangle.uv[0].u;
            texCoords_[texcoordIndex++] = triangle.uv[0].v;
            texCoords_[texcoordIndex++] = triangle.uv[1].u;
            texCoords_[texcoordIndex++] = triangle.uv[1].v;
            texCoords_[texcoordIndex++] = triangle.uv[2].u;
            texCoords_[texcoordIndex++] = triangle.uv[2].v;
        }
    }

    if( isTextured_ )
    {
        asdk::createTexture( &meshTexture_, mesh.getTexture(0)->getImage() );
    }


    // calculate mesh limits, that is, the size of its bounding box
    asdk::Point3F min, max;

    if( numberOfVertices_ > 0 )
    {
        asdk::Point3F first( vertexCoords_[0], vertexCoords_[1], vertexCoords_[2] );
        max = min = first;
    }

    for( int index = 3; index < numberOfVertices_ *3; )
    {
        float& x = vertexCoords_[index++]; 
        float& y = vertexCoords_[index++]; 
        float& z = vertexCoords_[index++]; 

        if( x > max.x ) max.x = x;
        else if( x < min.x ) min.x = x;
        
        if( y > max.y ) max.y = y;
        else if( y < min.y ) min.y = y;
        
        if( z > max.z ) max.z = z;
        else if( z < min.z ) min.z = z;
    }

    boundingBoxSize_ = max - min;

    return asdk::ErrorCode_OK;
}


// make OpenGL preparations for scene rendering
void ScenePresenter::prepareForRendering()
{
    glEnableClientState( GL_VERTEX_ARRAY );
    glEnableClientState( GL_NORMAL_ARRAY );

    glVertexPointer( 3, GL_FLOAT, 0, vertexCoords_.data() );
    glNormalPointer( GL_FLOAT, 0, normals_.data() );

    if( isTextured_ )
    {
        const void* imagePointer = meshTexture_->getImage()->getPointer();
        const asdk::ImageHeader header = meshTexture_->getImage()->getHeader();
        const float bcolor[]= { 0.0f, 0.0f, 0.0f, 0.0f };

        int format = GL_BGR_EXT;
        switch( header.pixelFormat )
        {
        case asdk::PixelFormat_Mono:
            format = GL_LUMINANCE;
            break;
        case asdk::PixelFormat_BGR:
            format = GL_BGR_EXT;
            break;
        case asdk::PixelFormat_BGRA:
            format = GL_BGRA_EXT;
            break;
        default:
            isTextured_ = false;
            return;
        }
        

        glTexCoordPointer( 2, GL_FLOAT, 0, texCoords_.data() );

        GLuint texName = 0;
        glGenTextures( 1, &texName );
        glBindTexture( GL_TEXTURE_2D, texName );

        glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR ); 
        glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR );
        glTexParameterfv( GL_TEXTURE_2D, GL_TEXTURE_BORDER_COLOR, bcolor );
        glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP );
        glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP );
        glTexImage2D( 
            GL_TEXTURE_2D, 0, GL_RGBA8,
            header.width,
            header.height,
            0, format, GL_UNSIGNED_BYTE, imagePointer 
        );

        glEnable( GL_TEXTURE_2D );
        glEnableClientState( GL_TEXTURE_COORD_ARRAY );

        // disable lighting in order to prevent color distortion

        glDisable( GL_LIGHTING );
        glDisable( GL_LIGHT0 );
    } 
    else 
    {
        glEnable( GL_LIGHTING );
        glEnable( GL_LIGHT0 );
        
        glDisable( GL_TEXTURE_2D );
        glDisableClientState( GL_TEXTURE_COORD_ARRAY );
    }
}

// visualize the scene based on prepared data
void ScenePresenter::render() const
{
    // use directional lights only
    asdk::Point4F lightPosition( 0.3f,  0.7f, -1, 0 );        // main light

    asdk::Point4F diffuseColor( 1.0f, 1.0f, 0.9f, 1. );
    asdk::Point4F specularColor( 0.5f, 0.5f, 0.45f, 1. );
    
    // main light
    glLightfv( GL_LIGHT0, GL_POSITION, lightPosition );
    glLightfv( GL_LIGHT0, GL_DIFFUSE,  diffuseColor );
    glLightfv( GL_LIGHT0, GL_SPECULAR, specularColor );


    glPushMatrix();

    // canonic rotation
    glRotatef( -90.0, 1.0, 0.0, 0.0 );

    // moving to the model center
    glTranslated( -massCenter_.x, -massCenter_.y, -massCenter_.z );

    glCullFace( GL_BACK );
    glColor3f( 1.0f, 1.0f, 1.0f );
    if( isTextured_ )
    {
        glEnable( GL_TEXTURE_2D );
    }
    else
    {
        glEnable( GL_LIGHTING );
    }
    glDrawArrays( GL_TRIANGLES, 0, numberOfVertices_ );

    glCullFace( GL_FRONT );
    glColor3f( 0.15f, 0.15f, 0.15f );
    if( isTextured_ )
    {
        glDisable( GL_TEXTURE_2D );
    }
    else
    {
        glDisable( GL_LIGHTING );
    }
    glDrawArrays( GL_TRIANGLES, 0, numberOfVertices_ );

    glPopMatrix();
}


//
// Implementation of a simple GLFW viewer for the scene
//

// longitude and latitude rotation angles 
static GLfloat sAlpha = 0.f, sBeta = 90.f;

// mouse position in the screen
static double sCursorX = 0.0, sCursorY = 0.0;

// camera positional shifts
static GLfloat sCameraWideShift = 0.f, sCameraHighShift = 0.f, sCameraDepth = 0.f;


// GLFW callback for handling of errors
static void errorCallback( int error, const char* description )
{
    fputs( description, stderr );
}

// GLFW callback for handling of keyboard strokes 
static void keyCallback( GLFWwindow* window, int key, int scancode, int action, int mods )
{
    if( key == GLFW_KEY_ESCAPE && action == GLFW_PRESS )
    {
        glfwSetWindowShouldClose( window, GL_TRUE );
    }
}

// GLFW callback for handling of mouse buttons 
static void mouseButtonCallback( GLFWwindow* window, int button, int action, int mods )
{
    if( button != GLFW_MOUSE_BUTTON_LEFT ) return;

    if( action == GLFW_PRESS )
    {
        glfwSetInputMode( window, GLFW_CURSOR, GLFW_CURSOR_DISABLED );
        glfwGetCursorPos( window, &sCursorX, &sCursorY );
    }
    else
    {
        glfwSetInputMode( window, GLFW_CURSOR, GLFW_CURSOR_NORMAL );
    }
}

// GLFW callback for handling of mouse moves
static void cursorPositionCallback( GLFWwindow* window, double x, double y )
{
    if( glfwGetInputMode( window, GLFW_CURSOR ) == GLFW_CURSOR_DISABLED )
    {
        if( 
            glfwGetKey( window, GLFW_KEY_LEFT_CONTROL ) == GLFW_PRESS 
            || glfwGetKey( window, GLFW_KEY_RIGHT_CONTROL ) == GLFW_PRESS
        ){
            sCameraWideShift += (GLfloat)(x - sCursorX);  
            sCameraHighShift -= (GLfloat)(y - sCursorY); 
        } 
        else 
        {
            sAlpha += (GLfloat) (x - sCursorX) / 10.f;
            sBeta += (GLfloat) (y - sCursorY) / 10.f;
        }

        sCursorX = x;
        sCursorY = y;
    }
}

// GLFW callback for handling of mouse scrolling
static void scrollCallback( GLFWwindow* window, double x, double y )
{
    float delta = (float) y * 50.f;
    if( 
        glfwGetKey( window, GLFW_KEY_LEFT_CONTROL ) == GLFW_PRESS
        || glfwGetKey( window, GLFW_KEY_RIGHT_CONTROL ) == GLFW_PRESS
    ){
        delta *= 10;
    }

    sCameraDepth += delta;
    if( sCameraDepth < 0 )
    {
        sCameraDepth = 0;
    }
}

// show info regarding scene window user interaction
void PrintDisplayHelp()
{
    std::wcout << std::endl;
    std::wcout << L"Presentation help:" << std::endl;
    std::wcout << L" - push & hold left mouse button to rotate" << std::endl;
    std::wcout << L" - use CTRL + left mouse button to move" << std::endl;
    std::wcout << L" - use mouse scroll to zoom" << std::endl;
    std::wcout << L" - use CTRL + mouse scroll to zoom fast" << std::endl;
    std::wcout << L" - press ESC to exit" << std::endl;
}


// main viewer presenting function
asdk::ErrorCode DisplayScene( const asdk::ICompositeMesh& inputMesh )
{
    ScenePresenter theScene;

    asdk::ErrorCode ec = theScene.init( inputMesh );
    if( ec != asdk::ErrorCode_OK ) return ec;

    
    float modelMaxSize = theScene.getMaxSize();
    if( modelMaxSize > 4000.f )
    {
        modelMaxSize = 4000.f;  // heuristic
    }

    sCameraDepth = modelMaxSize *4;  // heuristic


    glfwSetErrorCallback( errorCallback );

    if( !glfwInit() )
    {
        std::wcerr << L"GLFW error while initialization." << std::endl;
        return asdk::ErrorCode_OperationFailed;
    }

    GLFWwindow* window = glfwCreateWindow( 
        PresenterWindowSize, PresenterWindowSize, "Press ESC to exit", NULL, NULL 
    );
    if( !window )
    {
        glfwTerminate();
        std::wcerr << L"GLFW error while creating window." << std::endl;
        return asdk::ErrorCode_OperationFailed;
    }

    PrintDisplayHelp();


    // GLFW initialization

    glfwMakeContextCurrent( window );
    glfwSwapInterval( 1 );

    glfwSetKeyCallback( window, keyCallback );
    glfwSetMouseButtonCallback( window, mouseButtonCallback );
    glfwSetCursorPosCallback( window, cursorPositionCallback );
    glfwSetScrollCallback( window, scrollCallback );

    // GL initialization

    glEnable( GL_CULL_FACE );

    glEnable( GL_DEPTH_TEST );
    glDepthFunc( GL_LEQUAL );

    theScene.prepareForRendering();


    // main user interaction processing cycle

    while( !glfwWindowShouldClose( window ) )
    {
        // clear background
        glClearColor( 0.91f, 0.91f, 0.91f, 1.0f );
        glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );

        int width, height;
        glfwGetFramebufferSize( window, &width, &height );

        // resize field of view
        glMatrixMode( GL_PROJECTION );
        glLoadIdentity();

        glViewport( 0, 0, width, height );

        GLfloat ratio = 1.0f * width / height;
        gluPerspective( 20, ratio, 1, modelMaxSize *30 );

        // transform scene
        glMatrixMode( GL_MODELVIEW );
        glLoadIdentity();

        // move back
        glTranslatef( sCameraWideShift, sCameraHighShift, -sCameraDepth );

        // rotate the view
        glRotatef( sBeta, 1.0, 0.0, 0.0 );  
        glRotatef( sAlpha, 0.0, 1.0, 0.0 );

        // paint the picture
        theScene.render();

        glfwSwapBuffers( window );
        glfwPollEvents();
    }

    glfwDestroyWindow( window );

    glfwTerminate();

    return asdk::ErrorCode_OK;
}
