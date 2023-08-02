#ifdef _MSC_VER
#define _CRT_SECURE_NO_WARNINGS 1
#endif // _MSC_VER 

#include <artec/sdk/project/IProject.h>
#include <artec/sdk/project/EntryInfo.h>
#include <artec/sdk/project/ProjectSettings.h>
#include <artec/sdk/project/ProjectLoaderSettings.h>
#include <artec/sdk/project/ProjectSaverSettings.h>
#include <artec/sdk/algorithms/Algorithms.h>
#include <artec/sdk/algorithms/IAlgorithm.h>
#include <artec/sdk/base/IModel.h>
#include <artec/sdk/base/IScan.h>
#include <artec/sdk/base/IFrameMesh.h>
#include <artec/sdk/base/ICompositeContainer.h>
#include <artec/sdk/base/ICompositeMesh.h>
#include <artec/sdk/base/AlgorithmWorkset.h>
#include <artec/sdk/base/IProgressInfo.h>
#include <artec/sdk/base/IJobObserver.h>
#include <artec/sdk/base/IJob.h>
#include <artec/sdk/base/TRef.h>
#include <artec/sdk/base/IArray.h>

#include <boost/atomic.hpp>
#include <boost/thread.hpp>

#include <iomanip>
#include <iostream>
#include <string>

#include <stdlib.h>

using namespace artec::sdk;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Helper classes, functions and macros

// Simple error log handling for SDK calls
#define SDK_STRINGIFY(x) #x
#define SDK_STRING(x) SDK_STRINGIFY(x)
#define SAFE_SDK_CALL(x)                                                    \
{                                                                           \
    base::ErrorCode ec = (x);                                               \
    if ( ec != base::ErrorCode_OK )                                         \
    {                                                                       \
        reportError( ec, __FILE__ " [ line " SDK_STRING(__LINE__) " ]");    \
        return ec;                                                          \
    }                                                                       \
}

void printUsage()
{
    std::cout << "Usage:" << '\n';
    std::cout << " project-sample.exe input_project_path output_project_path" << '\n';
    std::cout << '\n';
    std::cout << "  input_project_path \t path to a valid Arteс Studio project which must exist" << '\n';
    std::cout << "  output_project_path \t path to the destination project which must NOT exist" << '\n';
    std::cout << '\n';
    std::cout << "Example:" << '\n';
    std::cout << " project-sample.exe C:\\project1\\project1.sproj C:\\processed\\processed.sproj" << std::endl;
    std::cout << '\n';
}

std::wstring stringToWString(const std::string& src)
{
    std::wstring result(src.length(), L' ');
    mbstowcs(&result[0], src.c_str(), src.length());
    return result;
}

void reportError(base::ErrorCode ec, const char* place)
{
    const char* msg = "No error";

    switch (ec)
    {
    case base::ErrorCode_OutOfMemory:
        msg = "Not enough storage is available to process the operation";
        break;

    case base::ErrorCode_ArgumentInvalid:
        msg = "Provided argument is invalid";
        break;

    case base::ErrorCode_OperationInvalid:
        msg = "Requested operation is invalid";
        break;

    case base::ErrorCode_FormatUnsupported:
        msg = "Data format is unsupported or invalid";
        break;

    case base::ErrorCode_ScannerNotConnected:
        msg = "Requested scanner is not connected";
        break;

    case base::ErrorCode_ScannerNotLicensed:
        msg = "Requested scanner is not licensed";
        break;

    case base::ErrorCode_ScannerLocked:
        msg = "Requested scanner is already used by someone else";
        break;

    case base::ErrorCode_ScannerInitializationFailed:
        msg = "Scanner initialization failed";
        break;

    case base::ErrorCode_FrameCorrupted:
        msg = "Frame is corrupted";
        break;

    case base::ErrorCode_FrameReconstructionFailed:
        msg = "Frame reconstruction failed";
        break;

    case base::ErrorCode_FrameRegistrationFailed:
        msg = "Frame registration failed";
        break;

    case base::ErrorCode_OperationUnsupported:
        msg = "Requested operation is unsupported. Check versions";
        break;

    case base::ErrorCode_OperationDenied:
        msg = "Requested operation is denied. Check your license(s)";
        break;

    case base::ErrorCode_OperationFailed:
        msg = "Requested operation has failed";
        break;

    case base::ErrorCode_OperationAborted:
        msg = "Requested operation was canceled from client's side";
        break;

    case base::ErrorCode_AllFramesAreFilteredOut:
        msg = "Unable to start algorithm because input data turned out to be invalid. Please rescan the object.";
        break;

    default:
        msg = "Unexplained error";
        break;
    }

    std::cerr << msg << " [error " << std::hex << ec << "] " << "at " << place << std::endl;
}

// Objects of this class yield processor time to other threads while waiting for a completion signal
class SleepingObserver : public base::JobObserverBase
{
public:
    SleepingObserver()
    : done_(false)
    , result_(base::ErrorCode_OK)
    {
    }

    void completed(base::ErrorCode result) override
    {
        result_ = result;
        done_ = true;
    }

    void waitForCompletion() const
    {
        while (!done_)
        {
            boost::this_thread::yield();
        }
    }

    base::ErrorCode getResult() const
    {
        return result_;
    }

private:
    boost::atomic<bool> done_;
    base::ErrorCode result_;
}; // class SleepingObserver

base::ErrorCode createSleepingObserver(SleepingObserver** observer)
{
    if (!observer)
    {
        return base::errors::ErrorCode_ArgumentInvalid;
    }
    *observer = new SleepingObserver;
    return base::ErrorCode_OK;
}

// This progress implementation simply prints the current progress to the console
class SimpleConsoleProgress : public base::ProgressInfoBase
{
public:
    SimpleConsoleProgress(const char* name)
    : name_(name)
    {
    }

private:
    void report(int current, int total) override
    {
        std::cout << name_ << ": " << current << " of " << total << std::endl;
    }

    void pulse() override {}

    void notify(base::DetailsInfo details) override {}

private:
    const std::string name_;
}; // Class SimpleConsoleProgress

// Run a job asynchronously with progress tracking and release a reference to the job on success
base::ErrorCode runJobAsynchronously(
    base::IModel* srcModel, base::IModel* trgModel, base::IJob* job, const char* jobName)
{
    // And use it to align the scans
    base::AlgorithmWorkset workset;
    workset.in = srcModel;
    workset.out = trgModel;
    SimpleConsoleProgress* const progress = new SimpleConsoleProgress(jobName);
    base::TRef<SimpleConsoleProgress> progressRef;
    progressRef.attach(progress);
    workset.progress = progressRef;
    workset.cancellation = NULL;
    base::TRef<SleepingObserver> observer;
    createSleepingObserver(&observer);
    SAFE_SDK_CALL(base::launchJob(job, &workset, observer));
    // Here, a real-world application should perform some useful operations instead of waiting 
    observer->waitForCompletion();
    return observer->getResult();
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Functions that constitute the program major operations

// Load scans from an input project
base::ErrorCode loadInputScans(project::IProject* project, base::IModel** pModel)
{
    // Load two scans
    const int numScansToLoad = 2;
    base::TRef<base::IArrayUuid> uuids;
    SAFE_SDK_CALL(createArrayUuid(&uuids, numScansToLoad));
    int numScans = 0;
    const int numEntries = project->getEntryCount();
    for (int i = 0; i < numEntries && numScans < numScansToLoad; ++i)
    {
        project::EntryInfo entry;
        SAFE_SDK_CALL(project->getEntry(i, &entry));
        if (entry.type == project::EntryType_Scan)
        {
            uuids->setElement(numScans, entry.uuid);
            ++numScans;
        }
    }
    // Check the number of scans: the alignment algorithm requires this number to be no less than 2
    if (numScans < numScansToLoad)
    {
        std::cerr << "Not enough scans to align." << std::endl;
        return base::ErrorCode_OperationFailed;
    }

    // Load the scans
    project::ProjectLoaderSettings settings;
    settings.entryList = uuids;
    base::TRef<base::IJob> loader;
    SAFE_SDK_CALL(project->createLoader(&loader, &settings));
    SAFE_SDK_CALL(base::createModel(pModel));
    SAFE_SDK_CALL(runJobAsynchronously(nullptr, *pModel, loader, "Loading scans"));

    return base::ErrorCode_OK;
}

// Apply auto-alignment to the IModel's scans
base::ErrorCode autoAlignModelScans(base::IModel* model, base::IModel* targetModel)
{
    // Calculate normals to vertices for each frame in order to prepare the scans for alignment
    const int numScans = model->getSize();
    for (int i = 0; i < numScans; ++i)
    {
        base::IScan* const scan = model->getElement(i);
        const int numFrames = scan->getSize();
        for (int j = 0; j < numFrames; ++j)
        {
            base::IFrameMesh* const frame = scan->getElement(j);
            frame->calculate(base::CM_PointsNormals_Default);
        }
    }

    // Remember transformation matrix of the second scan 
    const base::Matrix4x4D& secondScanTransformationBefore = model->getElement(1)->getScanTransformation();

    // Now create an Auto-alignment algorithm instance
    {
        algorithms::AutoAlignSettings autoAlignSettings;
        autoAlignSettings.scannerType = base::ScannerType_Unknown;
        base::TRef<algorithms::IAlgorithm> autoAlignAlgorithm;
        SAFE_SDK_CALL(algorithms::createAutoalignAlgorithm(&autoAlignAlgorithm, &autoAlignSettings));
        SAFE_SDK_CALL(runJobAsynchronously(model, targetModel, autoAlignAlgorithm, "Auto-aligning"));
    }

    // Check whether the second scan has changed its position
    const base::Matrix4x4D& secondScanTransformationAfter = targetModel->getElement(1)->getScanTransformation();
    if (secondScanTransformationAfter == secondScanTransformationBefore)
    {
        std::cerr << "No alignment happened." << std::endl;
        return base::ErrorCode_OperationFailed;
    }
    return base::ErrorCode_OK;
}

// Make a model (3D mesh) out of the aligned scans
base::ErrorCode fuseModelScans(base::IModel* model, base::IModel* targetModel)
{
    // Run Global registration
    base::TRef<base::IModel> registrationTargetModel;
    SAFE_SDK_CALL(base::createModel(&registrationTargetModel));
    {
        algorithms::GlobalRegistrationSettings registrationSettings;
        registrationSettings.registrationType = algorithms::GlobalRegistrationType_Geometry;
        registrationSettings.scannerType = base::ScannerType_Unknown;
        base::TRef<algorithms::IAlgorithm> registrationAlgorithm;
        SAFE_SDK_CALL(algorithms::createGlobalRegistrationAlgorithm(&registrationAlgorithm, &registrationSettings));
        SAFE_SDK_CALL(runJobAsynchronously(model, registrationTargetModel, registrationAlgorithm, "Global registration"));
    }

    // Apply a Fast-fusion algorithm
    {
        base::TRef<algorithms::IAlgorithm> fastFusionAlgorithm;
        algorithms::FastFusionSettings fastFusionSettings;
        SAFE_SDK_CALL(algorithms::initializeFastFusionSettings(&fastFusionSettings, artec::sdk::base::ScannerType_Unknown));
        SAFE_SDK_CALL(algorithms::createFastFusionAlgorithm(&fastFusionAlgorithm, &fastFusionSettings));
        SAFE_SDK_CALL(runJobAsynchronously(registrationTargetModel, targetModel, fastFusionAlgorithm, "Fast fusion"));
    }

    // Check the presence of a model
    const base::ICompositeContainer* const compositeContainer = targetModel->getCompositeContainer();
    if (compositeContainer == NULL || compositeContainer->getSize() < 1)
    {
        std::cerr << "No model mesh created." << std::endl;
        return base::ErrorCode_OperationFailed;
    }
    return base::ErrorCode_OK;
}

// Save the IModel object to an Artec Studio project
base::ErrorCode saveModelAsProject(base::IModel* model, project::IProject* project, const wchar_t* projectDstPath)
{
    project::ProjectSaverSettings saveSettings;
    saveSettings.path = projectDstPath;
    SAFE_SDK_CALL(base::generateUuid(&saveSettings.projectId));
    base::TRef<base::IJob> saver;
    SAFE_SDK_CALL(project->createSaver(&saver, &saveSettings));
    SAFE_SDK_CALL(runJobAsynchronously(model, nullptr, saver, "Saving project"));
    return base::ErrorCode_OK;
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int main(int argc, char** argv)
{
    if (argc < 3)
    {
        printUsage();
        return -1;
    }

    const std::wstring srcProjectPath = stringToWString(argv[1]);
    base::TRef<project::IProject> srcProject;
    base::TRef<base::IModel> srcModel;
    SAFE_SDK_CALL(project::openProject(&srcProject, srcProjectPath.c_str()));
    if (loadInputScans(srcProject, &srcModel) != base::ErrorCode_OK)
    {
        std::cerr << "Failed to load input data. Exiting..." << std::endl;
        return 1;
    }

    base::TRef<base::IModel> alignTargetModel;
    SAFE_SDK_CALL(base::createModel(&alignTargetModel));
    if (autoAlignModelScans(srcModel, alignTargetModel) != base::ErrorCode_OK)
    {
        std::cerr << "Auto-alignment failed. Exiting..." << std::endl;
        return 1;
    }

    base::TRef<base::IModel> fusionTargetModel;
    SAFE_SDK_CALL(base::createModel(&fusionTargetModel));
    if (fuseModelScans(alignTargetModel, fusionTargetModel) != base::ErrorCode_OK)
    {
        std::cerr << "3D mesh model creation failed. Exiting..." << std::endl;
        return 1;
    }

    const std::wstring dstProjectPath = stringToWString(argv[2]);
    if (saveModelAsProject(fusionTargetModel, srcProject, dstProjectPath.c_str()) != base::ErrorCode_OK)
    {
        std::cerr << "Project saving failed. Exiting..." << std::endl;
        return 1;
    }

    return 0;
}
