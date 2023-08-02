/********************************************************************
*
*   Project     Artec 3D Scanning SDK Samples
*
*   Purpose:    Scanning and processing sample
*
*   Copyright:  Artec Group
*
********************************************************************/
#include <iomanip>
#include <iostream>
#include <artec/sdk/base/Log.h>
#include <artec/sdk/base/Errors.h>

#include <artec/sdk/base/TRef.h>
#include <artec/sdk/base/RefBase.h>
#include <artec/sdk/base/IFrameMesh.h>
#include <artec/sdk/base/ICompositeMesh.h>
#include <artec/sdk/base/ICompositeContainer.h>
#include <artec/sdk/base/IModel.h>
#include <artec/sdk/base/ICancellationTokenSource.h>
#include <artec/sdk/base/io/ObjIO.h>
#include <artec/sdk/base/AlgorithmWorkset.h>
#include <artec/sdk/capturing/IScanner.h>
#include <artec/sdk/capturing/IScannerObserver.h>
#include <artec/sdk/capturing/IArrayScannerId.h>
#include <artec/sdk/scanning/IScanningProcedure.h>
#include <artec/sdk/scanning/IArrayScanner.h>
#include <artec/sdk/scanning/IScanningProcedureBundle.h>
#include <artec/sdk/algorithms/IAlgorithm.h>
#include <artec/sdk/algorithms/Algorithms.h>
#include "ScenePresenter.h"

namespace asdk {
    using namespace artec::sdk::base;
    using namespace artec::sdk::capturing;
    using namespace artec::sdk::scanning;
    using namespace artec::sdk::algorithms;
};
using asdk::TRef;

// this constant determines the number of frames to collect.
const int NumberOfFramesToCapture = 100;

// Texture processing needs a lot of processing power and a suitable
// graphic card. Define the following macro if you wish to include it
// into this sample.

#define ENABLE_TEXTURE_MAPPING

// Saving the results takes time and needs considerable amount
// of disc space. Uncomment one or both of the following macros
// in order to enable it. Make sure you have a subdirectory
// designated as OUTPUT_DIR in the current directory
// for output files to be placed.
#define OUTPUT_DIR L"scans"
//#define SAVE_FUSION_MESH_ON
//#define SAVE_TEXTURED_MESH_ON


// simple error log handling for SDK calls
#define SDK_STRINGIFY(x) #x
#define SDK_STRING(x) SDK_STRINGIFY(x)
#define SAFE_SDK_CALL(x)                                                 \
{                                                                        \
    asdk::ErrorCode ec = (x);                                            \
    if ( ec != asdk::ErrorCode_OK )                                      \
    {                                                                    \
        reportError( ec, __FILE__ " [ line " SDK_STRING(__LINE__) " ]"); \
        return ec;                                                       \
    }                                                                    \
}

void reportError( asdk::ErrorCode ec, const char *place )
{
    const wchar_t* msg = L"No error";

    switch( ec ){

    case asdk::ErrorCode_OutOfMemory:
        msg = L"Not enough storage is available to process the operation";
        break;

    case asdk::ErrorCode_ArgumentInvalid:
        msg = L"Provided argument is invalid";
        break;

    case asdk::ErrorCode_OperationInvalid:
        msg = L"Requested operation is invalid";
        break;

    case asdk::ErrorCode_FormatUnsupported:
        msg = L"Data format is unsupported or invalid";
        break;

    case asdk::ErrorCode_ScannerNotConnected:
        msg = L"Requested scanner is not connected";
        break;

    case asdk::ErrorCode_ScannerNotLicensed:
        msg = L"Requested scanner is not licensed";
        break;

    case asdk::ErrorCode_ScannerLocked:
        msg = L"Requested scanner is already used by someone else";
        break;

    case asdk::ErrorCode_ScannerInitializationFailed:
        msg = L"Scanner initialization failed";
        break;

    case asdk::ErrorCode_FrameCorrupted:
        msg = L"Frame is corrupted";
        break;

    case asdk::ErrorCode_FrameReconstructionFailed:
        msg = L"Frame reconstruction failed";
        break;

    case asdk::ErrorCode_FrameRegistrationFailed:
        msg = L"Frame registration failed";
        break;

    case asdk::ErrorCode_OperationUnsupported:
        msg = L"Requested operation is unsupported. Check versions";
        break;

    case asdk::ErrorCode_OperationDenied:
        msg = L"Requested operation is denied. Check your license(s)";
        break;

    case asdk::ErrorCode_OperationFailed:
        msg = L"Requested operation has failed";
        break;

    case asdk::ErrorCode_OperationAborted:
        msg = L"Requested operation was canceled from client's side";
        break;

    case asdk::ErrorCode_AllFramesAreFilteredOut:
        msg = L"Unable to start algorithm because input data turned out to be invalid. Please rescan the object.";
        break;

    default:
        msg = L"Unexplained error";
        break;
    }

    std::wcerr << msg << " [error " << std::hex << ec << "] " << "at " << place << std::endl;
}


// simple demonstration handler for scanner events
class SimpleScannerObserver : public asdk::ScannerObserverBase
{
public:
    // scanner event handling
    virtual void buttonPressed( asdk::ScannerButton button )
    {
        switch(button)
        {
        case asdk::ScannerButton_RecordStop:
            std::wcout << L"ScannerEvent: trigger button was pressed" << std::endl;
            break;

        case asdk::ScannerButton_Stop:
            std::wcout << L"ScannerEvent: stop button was pressed" << std::endl;
            break;

        case asdk::ScannerButton_Record:
            std::wcout << L"ScannerEvent: record button was pressed" << std::endl;
            break;

        default:
            std::wcout << L"ScannerEvent: unknown button was pressed" << std::endl;
            break;
        }
    }
    virtual void deviceOverheated()
    {
        std::wcout << L"ScannerEvent: device is overheated" << std::endl;
    }
    virtual void deviceTemperatureBackToNormal()
    {
        std::wcout << L"ScannerEvent: device temperature is back to normal" << std::endl;
    }
    virtual void deviceDisconnected()
    {
        std::wcout << L"ScannerEvent : device was disconnected" << std::endl;
    }
};

// creator function for simple demonstration handler defined above
//
// This function is used to initialize TRef<asdk::IScannerObserver>, and
// this is a preferred way to deal with current implementation of TRef.
// Please don't assign a newly created object directly to TRef as this
// may cause memory leaks. 
asdk::ErrorCode createSimpleScannerObserver( asdk::IScannerObserver** observer )
{
    *observer = new SimpleScannerObserver();

    return asdk::ErrorCode_OK;
}


// scanning procedure sample
asdk::ErrorCode ScanningProcedureSample( asdk::AlgorithmWorkset& workset  )
{
    std::wcout << L"Looking for scanner..." << std::endl;

    TRef<asdk::IScanner> scanner;
    asdk::ErrorCode errorCode = asdk::createScanner( &scanner, NULL ); //connect only one scanner
    if( errorCode != asdk::ErrorCode_OK )
    {
        std::wcout << L"No scanners found" << std::endl;
        return errorCode;
    }
    std::wcout << L"OK" << std::endl;
    std::wcout << L"Found scanner with serial number " << scanner->getId()->serial  << std::endl;


    std::wcout << L"Setting the scanner event handler..." << std::endl;
    TRef<asdk::IScannerObserver> observer;
    createSimpleScannerObserver( &observer );

    SAFE_SDK_CALL( scanner->setObserver( observer ) );
    std::wcout << L"OK" << std::endl;


    std::wcout << L"Scanner is ready, press ENTER to start" << std::endl;
    std::wcin.setf( ~std::ios::skipws, std::ios::skipws );
    std::wcin.get();


    std::wcout << L"Creating scanning procedure..." << std::endl;
    TRef<asdk::IScanningProcedure> scanning;

    asdk::ScanningProcedureSettings desc = { 0 };
    desc.maxFrameCount = NumberOfFramesToCapture;
    desc.initialState = asdk::ScanningState_Record;
    desc.pipelineConfiguration =
        asdk::ScanningPipeline_MapTexture
        | asdk::ScanningPipeline_FindGeometryKeyFrame
        | asdk::ScanningPipeline_RegisterFrame
        | asdk::ScanningPipeline_ConvertTextures
    ;
    desc.captureTexture = asdk::CaptureTextureMethod_EveryNFrame;
    desc.captureTextureFrequency = 10;
    desc.ignoreRegistrationErrors = false;

    SAFE_SDK_CALL( asdk::createScanningProcedure( &scanning, scanner, &desc ) ); //Create Scanning Procedure
    std::wcout << L"OK" << std::endl;


    std::wcout << L"Launching scanning procedure in a fully automatic mode..." << std::endl;
    SAFE_SDK_CALL( executeJob( scanning, &workset ) );
    std::wcout << L"OK" << std::endl;


    std::wcout << L"Preparing workset for further processing..." << std::endl;
    std::swap( workset.in, workset.out ); // 1-1. Run std::swap to put the previous algorithm's output to the input of the next one
    workset.out->clear();                 // 1-2. Clear the new workset.out (which is actually the former input)
    std::wcout << L"OK" << std::endl;

    return asdk::ErrorCode_OK;
}


// example of the post-scanning processing
asdk::ErrorCode AlgorithmProcessingSample( asdk::AlgorithmWorkset& workset  )
{
    // get scanner type from the very first scan in workset
    asdk::ScannerType scannerType = (asdk::ScannerType)workset.in->getElement(0)->getScannerType();

    // apply serial registration
    {
        std::wcout << L"Creating serial registration procedure..." << std::endl;

        TRef<asdk::IAlgorithm> serialRegistration;
        asdk::SerialRegistrationSettings serialDesc = { // 1-3. Initialize the default algorithm's settings, alter them as necessary
            scannerType, asdk::SerialRegistrationType_FineTextured
        };

        SAFE_SDK_CALL( asdk::createSerialRegistrationAlgorithm( &serialRegistration, &serialDesc ) );   // 1-4. createXxxxAlgorithm() using the obtained parameters' values
        std::wcout << L"OK" << std::endl;


        std::wcout << L"Launching the serial registration algorithm..." << std::endl;
        SAFE_SDK_CALL( asdk::executeJob( serialRegistration, &workset ) );          // 1-5. Run executeJob()
        std::wcout << L"OK" << std::endl;
    }

    // prepare serial registration output for the global registration
    std::swap( workset.in, workset.out );   // 2-1. Run std::swap to put the previous algorithm's output to the input of the next one
    workset.out->clear();                   // 2-2. Clear the new workset.out (which is actually the former input)


    // proceed with global registration
    {
        std::wcout << L"Creating global registration procedure..." << std::endl;

        TRef<asdk::IAlgorithm> globalRegistration;
        asdk::GlobalRegistrationSettings globalDesc = {     // 2-3. Initialize the default algorithm's settings, alter them as necessary
            scannerType, asdk::GlobalRegistrationType_Geometry
        };
        SAFE_SDK_CALL( asdk::createGlobalRegistrationAlgorithm( &globalRegistration, &globalDesc ) ); // 2-4. createXxxxAlgorithm() using the obtained parameters' values
        std::wcout << L"OK" << std::endl;


        std::wcout << L"Launching the global registration algorithm..." << std::endl;
        SAFE_SDK_CALL( asdk::executeJob( globalRegistration, &workset ) );  // 2-5. Run executeJob()
        std::wcout << L"OK" << std::endl;
    }

    // prepare global registration output for outliers removal
    std::swap( workset.in, workset.out );   // 3-1. Run std::swap to put the previous algorithm's output to the input of the next one
    workset.out->clear();                   // 3-2. Clear the new workset.out (which is actually the former input)


    // apply outliers removal
    {
        std::wcout << L"Creating outliers removal procedure..." << std::endl;

        TRef<asdk::IAlgorithm> noOutliers;
        asdk::OutliersRemovalSettings outliersDesc;
        // get default settings
        SAFE_SDK_CALL( asdk::initializeOutliersRemovalSettings( &outliersDesc, scannerType ) ); // 3-3. Initialize the default algorithm's settings, alter them as necessary
        SAFE_SDK_CALL( asdk::createOutliersRemovalAlgorithm( &noOutliers, &outliersDesc ) );    // 3-4. createXxxxAlgorithm() using the obtained parameters' values
        std::wcout << L"OK" << std::endl;


        std::wcout << L"Launching the outliers removal algorithm..." << std::endl;
        SAFE_SDK_CALL( asdk::executeJob( noOutliers, &workset ) );  // 3-5. Run executeJob()
        std::wcout << L"OK" << std::endl;
    }

    // prepare outliers removal results for fusion input
    std::swap( workset.in, workset.out );   // 4-1. Run std::swap to put the previous algorithm's output to the input of the next one
    workset.out->clear();                   // 4-2. Clear the new workset.out (which is actually the former input)


    // apply fast fusion
    {
        std::wcout << L"Creating fast fusion procedure..." << std::endl;
        TRef<asdk::IAlgorithm> fusion;
        asdk::FastFusionSettings fusionDesc;
        // get default settings
        SAFE_SDK_CALL( asdk::initializeFastFusionSettings( &fusionDesc, scannerType ) );    // 4-3. Initialize the default algorithm's settings, alter them as necessary
        fusionDesc.resolution = 2.f;
        SAFE_SDK_CALL( asdk::createFastFusionAlgorithm( &fusion, &fusionDesc ) );           // 4-4. createXxxxAlgorithm() using the obtained parameters' values
        std::wcout << L"OK" << std::endl;


        std::wcout << L"Launching the fast fusion algorithm..." << std::endl;
        SAFE_SDK_CALL( asdk::executeJob( fusion, &workset ) );                              // 4-5. Run executeJob()
        std::wcout << L"OK" << std::endl;
    }


    std::wcout << L"Preparing workset for further processing..." << std::endl;
    std::swap( workset.in, workset.out );       // 5-1. Run std::swap to put the previous algorithm's output to the input of the next one
    workset.out->clear();                       // 5-2. Clear the new workset.out (which is actually the former input)
    std::wcout << L"OK" << std::endl;

    return asdk::ErrorCode_OK;
}


#ifdef ENABLE_TEXTURE_MAPPING

// texture mapping snippet
asdk::ErrorCode TextureProcessingSample( asdk::AlgorithmWorkset& workset  )
{
    // get scanner type from the very first scan in workset
    asdk::ScannerType scannerType = (asdk::ScannerType)workset.in->getElement(0)->getScannerType();

    // apply texture mapping
    {
        std::wcout << L"Creating texture mapping procedure..." << std::endl;
        TRef<asdk::IAlgorithm> texturize;
        asdk::TexturizationSettings textureDesc;
        // get default settings
        SAFE_SDK_CALL( asdk::initializeTexturizationSettings( &textureDesc, scannerType ) );    // 5-3. Initialize the default algorithm's settings, alter them as necessary
        textureDesc.texturizeType = asdk::TexturizeType_Atlas;
        SAFE_SDK_CALL( asdk::createTexturizationAlgorithm( &texturize, &textureDesc ) );        // 5-4. createXxxxAlgorithm() using the obtained parameters' values
        std::wcout << L"OK" << std::endl;


        std::wcout << L"Launching the texture mapping algorithm..." << std::endl;
        SAFE_SDK_CALL( asdk::executeJob( texturize, &workset ) );                               // 5-5. Run executeJob()
        std::wcout << L"OK" << std::endl;
    }


    std::wcout << L"Preparing workset for further processing..." << std::endl;
    std::swap( workset.in, workset.out );           // 6-1. Run std::swap to put the previous algorithm's output to the input of the next one
    workset.out->clear();                           // 6-2. Clear the new workset.out (which is actually the former input)
    std::wcout << L"OK" << std::endl;

    return asdk::ErrorCode_OK;
}

#endif


int main( int argc, char **argv )
{
    // The log verbosity level is set here. It is set to the most
    // verbose value - Trace. If you have any problems working with 
    // our examples, please do not hesitate to send us this extensive 
    // information along with your questions. However, if you feel 
    // comfortable with these Artec Scanning SDK code examples,
    // we suggest you to set this level to asdk::VerboseLevel_Info.
    asdk::setOutputLevel( asdk::VerboseLevel_Trace );

    // create workset for scanned data
    TRef<asdk::IModel> inputContainer;
    TRef<asdk::IModel> outputContainer;
    TRef<asdk::ICancellationTokenSource> ctSource;

    SAFE_SDK_CALL( asdk::createModel( &inputContainer ) );
    SAFE_SDK_CALL( asdk::createModel( &outputContainer ) );
    SAFE_SDK_CALL( asdk::createCancellationTokenSource( &ctSource ) );

    asdk::AlgorithmWorkset workset = { inputContainer, outputContainer, 0, ctSource->getToken(), 0 };


    asdk::ErrorCode errorCode = ScanningProcedureSample( workset );
    if( errorCode != asdk::ErrorCode_OK )
    {
        std::wcout << L"Finishing work on errors when scanning..." << std::endl;
        return (int)errorCode;
    }


    errorCode = AlgorithmProcessingSample( workset );
    if( errorCode != asdk::ErrorCode_OK )
    {
        std::wcout << L"Finishing work on errors when processing..." << std::endl;
        return (int)errorCode;
    }


    #ifdef SAVE_FUSION_MESH_ON
    // saving the resulting mesh to OBJ format
    {
        asdk::ICompositeContainer* meshContainer = workset.in->getCompositeContainer();
        if( meshContainer && meshContainer->getSize() > 0 )
        {
            asdk::ICompositeMesh* resultMesh = meshContainer->getElement( 0 );

            std::wcout << L"Saving the resulting mesh to an OBJ file..." << std::endl;
            const wchar_t* filename = OUTPUT_DIR L"\\untextured-mesh.obj";
            errorCode = asdk::io::saveObjCompositeToFile( filename, resultMesh );
            if( errorCode != asdk::ErrorCode_OK )
            {
                std::wcout << L"Cannot open file '" << filename << L"'" << std::endl;
                std::wcout << L"skipped" << std::endl;
            }
            else
            {
                std::wcout << L"OK" << std::endl;
            }
        }
    }

    // saving all frame meshes to OBJ format
    {
        std::wcout << L"Saving all the reconstructed frames to separate OBJ files..." << std::endl;

        errorCode = asdk::ErrorCode_OK;

        for( int ix = 0;  ix < workset.in->getSize(); ix++ )
        {
            TRef<asdk::IScan> scan( workset.in->getElement( ix ) );

            for( int jx = 0; jx < scan->getSize(); jx++ )
            {
                TRef<asdk::IFrameMesh> frameMesh( scan->getElement( jx ) );

                std::wstring pathFormat( OUTPUT_DIR L"\\frame-S%02dF%02d.obj" );
                std::vector<wchar_t> pathBuffer( pathFormat.size() +1 );
                std::swprintf( pathBuffer.data(), pathBuffer.size(), pathFormat.c_str(), ix, jx );

                errorCode = asdk::io::saveObjFrameToFile( pathBuffer.data(), frameMesh );
                if( errorCode != asdk::ErrorCode_OK )
                {
                    std::wcout << L"Cannot open file '" << pathBuffer.data() << "'" << std::endl;
                    std::wcout << L"skipped" << std::endl;
                    break;
                }
            }
        }
        if( errorCode == asdk::ErrorCode_OK )
        {
            std::wcout << L"OK" << std::endl;
        }
    }
    #endif


    #ifdef ENABLE_TEXTURE_MAPPING

    errorCode = TextureProcessingSample( workset );
    if( errorCode != asdk::ErrorCode_OK )
    {
        std::wcout << L"failed" << std::endl;
        std::wcout << L"Continue to work withstanding errors when texturing..." << std::endl;
    }
    else
    {
        #ifdef SAVE_TEXTURED_MESH_ON

        // saving the resulting texture to OBJ format
        {
            asdk::ICompositeContainer* meshContainer = workset.in->getCompositeContainer();
            if( meshContainer && meshContainer->getSize() > 0 )
            {
                asdk::ICompositeMesh* resultMesh = meshContainer->getElement( 0 );

                std::wcout << L"Saving the resulting textured mesh to an OBJ file..." << std::endl;
                const wchar_t* filename = OUTPUT_DIR L"\\textured-mesh.obj";
                errorCode = asdk::io::saveObjCompositeToFile( filename, resultMesh );
                if( errorCode != asdk::ErrorCode_OK )
                {
                    std::wcout << L"Cannot open file '" << filename << "'" << std::endl;
                    std::wcout << L"skipped" << std::endl;
                }
                else
                {
                    std::wcout << L"OK" << std::endl;
                }
            }
        }

        #endif
    }

    #endif


    // demonstrating the results via the simple GLFW viewer
    {
        asdk::ICompositeContainer* meshContainer = workset.in->getCompositeContainer();
        if( meshContainer && meshContainer->getSize() > 0 )
        {
            asdk::ICompositeMesh* resultMesh = meshContainer->getElement( 0 );

            std::wcout << L"Showing the resulting mesh..." << std::endl;
            SAFE_SDK_CALL( DisplayScene( *resultMesh ) );
            std::wcout << L"OK" << std::endl;
        }
    }

    std::wcout << L"Finishing work with capturing library..." << std::endl;

    return (int)errorCode;
}
