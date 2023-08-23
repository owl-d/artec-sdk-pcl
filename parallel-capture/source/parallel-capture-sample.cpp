/********************************************************************
*
*	Project		Artec 3D Scanning SDK Samples
*
*	Purpose:	Parallel capture sample
*
*	Copyright:	Artec Group
*
********************************************************************/

#include <string>
#include <iostream>
#include <iomanip>
#include <boost/thread.hpp>
#include <boost/chrono.hpp>
#include <boost/format.hpp>

#include <artec/sdk/base/Log.h>
#include <artec/sdk/base/IFrameMesh.h>
#include <artec/sdk/base/TArrayRef.h>

#include <artec/sdk/capturing/IScanner.h>
#include <artec/sdk/capturing/IFrame.h>
#include <artec/sdk/capturing/IFrameProcessor.h>
#include <artec/sdk/capturing/IArrayScannerId.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <thread>

using namespace boost::chrono;
namespace asdk {
    using namespace artec::sdk::base;
    using namespace artec::sdk::capturing;
};
using asdk::TRef;
using asdk::TArrayRef;

pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;

// this constant determines the number of frames to collect.
const int NumberOfFramesToCapture = 1000;

void viewer_set()
{
    std::wcout << L"Init cloud" << std::endl;
    cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("Point Cloud Viewer"));
    viewer->addPointCloud<pcl::PointXYZ>(cloud, "point_cloud");
    viewer->setBackgroundColor(0.0, 0.0, 0.0);
    viewer->setSize(1200, 1000);
    viewer->setCameraPosition(0, 0, 0, 0, 0, -200, 0, 1, 0);

    while (!viewer->wasStopped())
    {
        viewer->updatePointCloud<pcl::PointXYZ>(cloud, "point_cloud");
        viewer->spinOnce();
        std::this_thread::sleep_for(std::chrono::milliseconds(200)); //0.2sec
    }
}

void mesh2pcd(TRef<asdk::IFrameMesh> mesh)
{
    if (mesh->isTextured())
    {
        cloud->clear();
        asdk::TArrayPoint3F points = mesh->getPoints();
        int size = points.size();

        std::wcout << L"Current Points Size... : " << size << std::endl;
        for (int i = 0; i < size; i++)
        {
            cloud->push_back(pcl::PointXYZ(points[i].x, points[i].y, points[i].z));
        }
    }
    else std::wcout << L"Mesh has no texture" << std::endl;
}


int main(int argc, char** argv)
{
    // The log verbosity level is set here. It is set to the most
    // verbose value - Trace. If you have any problems working with 
    // our examples, please do not hesitate to send us this extensive 
    // information along with your questions. However, if you feel 
    // comfortable with these Artec Scanning SDK code examples,
    // we suggest you to set this level to asdk::VerboseLevel_Info.
    asdk::setOutputLevel(asdk::VerboseLevel_Trace);

    // Initialize Pointcloud and viz
    std::thread cloud_viz(viewer_set);

    TRef<asdk::IScanner> scanner;
    {
        TRef<asdk::IArrayScannerId> scannersList;

        // Look for the scanners attached
        std::wcout << L"Enumerating scanners... ";
        if (asdk::enumerateScanners(&scannersList) != asdk::ErrorCode_OK)
        {
            std::wcout << L"failed" << std::endl;
            return 1;
        }
        std::wcout << L"done" << std::endl;

        // Check for any scanners found
        if (scannersList->getSize() == 0)
        {
            std::wcout << L"No scanners are found" << std::endl;
            return 2;
        }

        // Connect to the first available scanner
        TArrayRef<asdk::IArrayScannerId> scannersArray(scannersList);
        for (int i = 0; i < scannersArray.size(); i++)
        {
            std::wcout << L"Scanner " << scannersArray[i].serial << L" is found" << std::endl;

            std::wcout << L"Connecting to the scanner... ";
            if (asdk::createScanner(&scanner, &scannersArray[i]) != asdk::ErrorCode_OK)
            {
                std::wcout << L"failed" << std::endl;
                continue;
            }
            std::wcout << L"done" << std::endl;

            break;
        }

        if (!scanner)
        {
            std::wcout << L"No scanner can be connected to" << std::endl;
            return 3;
        }
    }

    // Start to work

    int framesAlreadyCaptured = scanner->getFrameNumber();

    // Storage for the frame meshes constructed
    std::vector< TRef<asdk::IFrameMesh> > meshes;
    meshes.resize(NumberOfFramesToCapture);

    boost::mutex meshesProtection;

    std::wcout << L"Capturing " << NumberOfFramesToCapture << " frames with " << boost::thread::hardware_concurrency() << L" threads" << std::endl;
    high_resolution_clock::time_point startTime = high_resolution_clock::now();

    // Start frame processing for every hard-supported thread available
    boost::thread_group captureThreads;
    for (unsigned i = 0; i < boost::thread::hardware_concurrency(); i++)
    {
        captureThreads.create_thread([&scanner, &meshes, &meshesProtection, framesAlreadyCaptured]
            {
                // Initialize a frame processor 
                TRef<asdk::IFrameProcessor> processor;
                if (scanner->createFrameProcessor(&processor) != asdk::ErrorCode_OK)
                {
                    return;
                }

                while (true)
                {
                    // Capture the next single frame image
                    TRef<asdk::IFrame> frame;
                    asdk::ErrorCode ec = scanner->capture(&frame, true);
                    if (ec != asdk::ErrorCode_OK)
                    {
                        if (ec == asdk::ErrorCode_FrameCaptureTimeout)
                        {
                            std::wcout << L"Capture error: frame capture timeout" << std::endl;
                        }
                        else if (ec == asdk::ErrorCode_FrameCorrupted)
                        {
                            std::wcout << L"Capture error: frame corrupted" << std::endl;
                        }
                        else if (ec == asdk::ErrorCode_FrameReconstructionFailed)
                        {
                            std::wcout << L"Capture error: frame reconstruction failed" << std::endl;
                        }
                        else if (ec == asdk::ErrorCode_FrameRegistrationFailed)
                        {
                            std::wcout << L"Capture error: frame registration failed" << std::endl;
                        }
                        else
                        {
                            std::wcout << L"Capture error: unknown error" << std::endl;
                        }

                        continue;
                    }

                    int frameNumber = frame->getFrameNumber();
                    if (frameNumber >= NumberOfFramesToCapture + framesAlreadyCaptured)
                    {
                        return;
                    }
                    std::wcout << "frame " << std::setw(4) << (frameNumber + 1) << "\r";

                    // Reconstruct 3D mesh for the captured frame
                    TRef<asdk::IFrameMesh> mesh;
                    if (processor->reconstructAndTexturizeMesh(&mesh, frame) != asdk::ErrorCode_OK)
                    {
                        std::wcout << L"Capture error: reconstruction failed for frame " << std::setw(4) << (frameNumber + 1) << std::endl;
                        continue;
                    }

                    mesh2pcd(mesh);

                    // Calculate additional data
                    mesh->calculate(asdk::CM_Normals);

                    std::wcout << "mesh  " << std::setw(4) << (frameNumber + 1) << "\r";
                    std::wcout.flush();

                    // Save the mesh for later use
                    boost::lock_guard<boost::mutex> guard(meshesProtection);

                    meshes[frameNumber - framesAlreadyCaptured] = mesh;
                }
            });
    }

    // Wait for the capture process to finish
    captureThreads.join_all();

    high_resolution_clock::time_point stopTime = high_resolution_clock::now();

    float timeDelta = duration_cast<milliseconds>(stopTime - startTime).count() / 1000.f;
    float fps = NumberOfFramesToCapture / timeDelta;
    std::wcout << "\nfps = " << std::fixed << std::setw(4) << std::setprecision(2) << fps << std::endl;

    int successfullyCapturedMeshes = 0;
    for (int i = 0; i < NumberOfFramesToCapture; i++)
    {
        if (meshes[i])
        {
            successfullyCapturedMeshes++;
        }
    }

    std::wcout << NumberOfFramesToCapture << " shots captured with " << successfullyCapturedMeshes << " meshes constructed." << std::endl;

    scanner = NULL;
    std::wcout << L"Scanner released" << std::endl;

    cloud_viz.join();
    std::wcout << L"Waits for the thread to finish its execution" << std::endl;

    return 0;
}
