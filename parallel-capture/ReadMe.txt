========================================================================
    CONSOLE APPLICATION : parallel-capture-sample Project Overview
========================================================================

/////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////

parallel-capture-sample is a test application for Artec Capture SDK.

This sample initializes the scanner and captures 3D frames as fast as possible
using multiple threads. Frames are saved for an optional post-processing.

/////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////

** Modify : Doyu Lim (2023.08)
Convert Mesh to Point cloud construct and real-time visualization using PCL lib.

To use PCL in Visual Studio, please add 'pcl-property-sheet.props' to your peoject.