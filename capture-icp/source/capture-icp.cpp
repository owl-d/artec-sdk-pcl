#define _CRT_SECURE_NO_WARNINGS
#pragma warning(disable : 4996)

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
#include <pcl/common/concatenate.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/gicp.h>
#include <pcl/filters/voxel_grid.h>

#include <thread>
#include <math.h>
#include "common.h"

using namespace boost::chrono;
namespace asdk {
    using namespace artec::sdk::base;
    using namespace artec::sdk::capturing;
};
using asdk::TRef;
using asdk::TArrayRef;

pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
pcl::PointCloud<pcl::PointXYZ>::Ptr cur_cloud;
pcl::PointCloud<pcl::PointXYZ>::Ptr prev_cloud;
Eigen::Matrix4f prev_tf;
Eigen::Matrix4f cur_tf;
bool cloud_flag = true;

// Hyper Parameter
const int NumberOfFramesToCapture = 100; // # of frames to collect.


void viewer_set()
{
    std::wcout << L"Init cloud" << std::endl;
    cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
    cur_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
    prev_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("Point Cloud Viewer"));
    //viewer->addPointCloud<pcl::PointXYZ>(cur_cloud, "cur_cloud");
    viewer->addPointCloud<pcl::PointXYZ>(cloud, "point_cloud");
    viewer->setBackgroundColor(0.0, 0.0, 0.0);
    viewer->setSize(1200, 1000);

    prev_tf = Eigen::Matrix4f::Identity();

    while (!viewer->wasStopped())
    {
        viewer->updatePointCloud<pcl::PointXYZ>(cloud, "point_cloud");
        //viewer->updatePointCloud<pcl::PointXYZ>(cur_cloud, "cur_cloud");
        viewer->spinOnce();
        std::this_thread::sleep_for(std::chrono::milliseconds(200)); //0.2sec
    }
}

void mesh2pcd(TRef<asdk::IFrameMesh> mesh)
{
    if (mesh->isTextured())
    {
        cur_cloud->clear();
        asdk::TArrayPoint3F points = mesh->getPoints();
        int size = points.size();

        std::wcout << L"Current Points Size... : " << size << std::endl;
        for (int i = 0; i < size; i++)
        {
            cur_cloud->push_back(pcl::PointXYZ(points[i].x, points[i].y, points[i].z));
        }
        if (cloud_flag) {
            *cloud = *cur_cloud;
            *prev_cloud = *cur_cloud;
            cloud_flag = false;
        }
    }
    else std::wcout << L"Mesh has no texture" << std::endl;
}

void ICP() {

    std::chrono::system_clock::time_point t_start = std::chrono::system_clock::now(); // Time Consuming Check

    // ICP
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setMaximumIterations(500); //def=200
    icp.setMaxCorrespondenceDistance(50); //def=5
    //icp.setUseReciprocalCorrespondences(true); //def=false
    icp.setRANSACIterations(300); //def=0
    icp.setRANSACOutlierRejectionThreshold(60); //def=0.05

    //maximum allowable squared difference between two consecutive transformations
    //in order for an optimization to be considered as having converged to the final solution. : 너무 크면 일찍 수렴했다고 판단하고, 너무 작으면 오래 걸림
    icp.setTransformationEpsilon(5e-8); //def=0.0005, 1e-10 부터 오래걸림
    icp.setEuclideanFitnessEpsilon(5e-8); //def=-1.79769e+308

    // tf guess
    Eigen::Vector3f angle = tf2euler(prev_tf);
    angle(1) += deg2rad(2); // pitch += 3 deg
    Eigen::Matrix4f guess_tf = euler2tf(angle);
    std::wcout << L"Guess tf (R/P/Y) : " << rad2deg(angle(0)) << L" / " << rad2deg(angle(1)) << L" / " << rad2deg(angle(2)) << std::endl;

    pcl::PointCloud<pcl::PointXYZ>::Ptr align_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr co_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    icp.setInputSource(cur_cloud);
    icp.setInputTarget(cloud);
    icp.align(*align_cloud, guess_tf);

    if (icp.hasConverged())
    {
        std::wcout << L"\nICP converged. Score is " << icp.getFitnessScore() << std::endl;

        cur_tf = icp.getFinalTransformation();
        Eigen::Vector3f theta = tf2euler(cur_tf);
        std::wcout << L"Translation(X/Y/Z) :" << cur_tf(0, 3) << " / " << cur_tf(1, 3) << " / " << cur_tf(2, 3) << std::endl;
        std::wcout << L"Rotation(R/P/Y) :" << rad2deg(theta(0)) << " / " << rad2deg(theta(1)) << " / " << rad2deg(theta(2)) << std::endl;
    }
    else std::wcout << L"ICP did not converge." << std::endl;

    std::chrono::system_clock::time_point t_end = std::chrono::system_clock::now();
    std::chrono::duration<double> t_reg = t_end - t_start;
    std::wcout << L"ICP Takes " << t_reg.count() << L" sec..." << endl;

    if (icp.getFitnessScore(100) < 5) {
        *co_cloud = *cloud + *align_cloud;
        *prev_cloud = *align_cloud;
        //pcl::transformPointCloud(*cur_cloud, *prev_cloud, guess_tf);
        prev_tf = cur_tf;

        //Remove duplicated points
        pcl::VoxelGrid<pcl::PointXYZ> sor;
        sor.setInputCloud(co_cloud);
        sor.setLeafSize(0.2f, 0.2f, 0.2f);
        sor.filter(*cloud);

        std::wcout << L"Accumulated Points Size... : " << cloud->points.size() << std::endl;
    }
    else std::wcout << L"[High ICP score] Skip registration of this frame. Score is " << icp.getFitnessScore() << std::endl;
}

int main(int argc, char** argv)
{
    asdk::setOutputLevel(asdk::VerboseLevel_Trace);

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

    //std::wcout << L"Capturing " << NumberOfFramesToCapture << " frames with " << boost::thread::hardware_concurrency() << L" threads" << std::endl;
    std::wcout << L"Capturing " << NumberOfFramesToCapture << " frames with 1" << L" threads" << std::endl;
    high_resolution_clock::time_point startTime = high_resolution_clock::now();

    // Initialize Pointcloud and viz
    std::thread cloud_viz(viewer_set);

    // Start frame processing for every hard-supported thread available
    boost::thread_group captureThreads;
    //for( unsigned i = 0; i < boost::thread::hardware_concurrency(); i++ )
    for (unsigned i = 0; i < 1; i++)
    {
        captureThreads.create_thread([&scanner, &meshes, &meshesProtection, framesAlreadyCaptured]
            {
                // Initialize a frame processor 
                TRef<asdk::IFrameProcessor> processor;
                if (scanner->createFrameProcessor(&processor) != asdk::ErrorCode_OK)
                {
                    return;
                }

                for (int i = 0; i < NumberOfFramesToCapture; i++)
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
                    //std::wcout << "frame " << std::setw(4) << (frameNumber + 1) << "\r";
                    std::wcout << "\n\n---------------------------------------\nframe " << std::setw(4) << (frameNumber + 1) << std::endl;

                    // Reconstruct 3D mesh for the captured frame
                    TRef<asdk::IFrameMesh> mesh;
                    if (processor->reconstructAndTexturizeMesh(&mesh, frame) != asdk::ErrorCode_OK)
                    {
                        std::wcout << L"Capture error: reconstruction failed for frame " << std::setw(4) << (frameNumber + 1) << std::endl;
                        continue;
                    }
                    mesh2pcd(mesh);
                    ICP();

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

    std::wcout << L"Accumulated Point Cloud is saved." << std::endl;
    pcl::io::savePCDFileASCII("C:/Users/ssoss/Desktop/samples_origin/samples/output_cloud.pcd", *cloud);

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
