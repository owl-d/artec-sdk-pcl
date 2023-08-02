/********************************************************************
*
*	Project		Artec 3D Scanning SDK Samples
*
*	Purpose:	Simple capture sample
*
*	Copyright:	Artec Group
*
********************************************************************/

#undef NDEBUG

#include <string>
#include <iostream>

#include <artec/sdk/capturing/IScanner.h>
#include <artec/sdk/capturing/IArrayScannerId.h>
#include <artec/sdk/capturing/IFrameProcessor.h>
#include <artec/sdk/capturing/IFrame.h>
#include <artec/sdk/base/BaseSdkDefines.h>
#include <artec/sdk/base/Log.h>
#include <artec/sdk/base/io/ObjIO.h>
#include <artec/sdk/base/IFrameMesh.h>
#include <artec/sdk/base/TArrayRef.h>

namespace asdk {
    using namespace artec::sdk::base;
    using namespace artec::sdk::capturing;
};
using asdk::TRef;
using asdk::TArrayRef;

int main( int argc, char **argv )
{
    // The log verbosity level is set here. It is set to the most
    // verbose value - Trace. If you have any problems working with 
    // our examples, please do not hesitate to send us this extensive 
    // information along with your questions. However, if you feel 
    // comfortable with these Artec Scanning SDK code examples,
    // we suggest you to set this level to asdk::VerboseLevel_Info.
    asdk::setOutputLevel( asdk::VerboseLevel_Trace );

	asdk::ErrorCode ec = asdk::ErrorCode_OK;

    TRef<asdk::IArrayScannerId> scannersList;

	std::wcout << L"Enumerating scanners... ";
	ec = asdk::enumerateScanners( &scannersList );
	if( ec != asdk::ErrorCode_OK ) //if error occurs
	{
		std::wcout << L"failed" << std::endl;
		return 1;
	}
	std::wcout << L"done" << std::endl;

	int scanner_count = scannersList->getSize();
    if( scanner_count == 0 )
    {
        std::wcout << L"No scanners found" << std::endl;
        return 3;
    }

    const asdk::ScannerId* idArray = scannersList->getPointer();

    const asdk::ScannerId& defaultScanner = idArray[0]; // just take the first available scanner

    std::wcout 
        << L"Connecting to " << asdk::getScannerTypeName( defaultScanner.type ) 
        << L" scanner " << defaultScanner.serial << L"... "
    ;

    TRef<asdk::IScanner> scanner;
    ec = asdk::createScanner( &scanner, &defaultScanner );

    if( ec != asdk::ErrorCode_OK )
    {
        std::wcout << L"failed" << std::endl;
        return 2;
    }
    std::wcout << L"done" << std::endl;

	std::wcout << L"Capturing frame... ";

    TRef<asdk::IFrame> frame;		// IFrame : Captured frame
    TRef<asdk::IFrameMesh> mesh;	// IFrameMesh : 3D surface obtained from the scanner (IMesh + IImage)

    TRef<asdk::IFrameProcessor> processor;				// Class : Raw frames processor that provides several methods to handle scanner's parameters
	ec = scanner->createFrameProcessor( &processor );	// Create IFrameProcessor processor
	if( ec == asdk::ErrorCode_OK )
	{
        frame = NULL;
		ec = scanner->capture( &frame, true ); // Perform capture. If captureTexture=true, texture will be captured. if false, how different?
		if( ec == asdk::ErrorCode_OK ) 
		{
            mesh = NULL;
			ec = processor->reconstructAndTexturizeMesh( &mesh, frame );	// Method both reconstructs a surface from the frame data and then if texture is captured, applies texture to the output.
																			// This is a slow method. Use reconstructMesh() instead.
			if( ec == asdk::ErrorCode_OK )
			{
				std::wcout << L"done" << std::endl;
				// save the mesh
				ec = asdk::io::Obj::save( L"frame.obj", mesh ); // Save IFrameMesh/ICompositeMesh to OBJ file.

				// working with normals
				// 1. generate normals
				mesh->calculate( asdk::CM_Normals );	// For area of polygons, create calculated normals data.

				// 2. get normals array using helper class
				asdk::TArrayPoint3F pointsNormals  = mesh->getPointsNormals();	// Get vertex normals (smooth). Return pointer to the IArrayPoint3F array.

				// 3. get number of normals
				int normalCount = pointsNormals.size();	
                ASDK_UNUSED(normalCount);

				// 4. use normal
				asdk::Point3F point = pointsNormals[0];
                ASDK_UNUSED(point);

				std::wcout << L"Captured mesh saved to disk" << std::endl;
			}
			else
			{
				std::wcout << L"failed" << std::endl;
			}
		}
	}

    scanner = NULL;
	std::wcout << L"Scanner released" << std::endl;

    return 0;
}
