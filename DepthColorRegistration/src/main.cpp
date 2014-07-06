//
// main.cpp
//
// Software License Agreement (BSD License)
//
// Copyright (c) 2014, Pat O'Keefe
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without modification, are
// permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice, this list of
// conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice, this list
// of conditions and the following disclaimer in the documentation and/or other materials
// provided with the distribution.
// 3. Neither the name of the copyright holder nor the names of its contributors may be
// used to endorse or promote products derived from this software without specific prior
// written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
// EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
// MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
// THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
// PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
// STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
// THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//


#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include <iostream>

#include "Registration.h"

using namespace cv;
using namespace std;


int main(int argc, const char * argv[]) {

    //----------------------
    // Open an OpenNI device
    //----------------------

    //TODO: You'll want to open an RGB camera stream here too (the one with wich you wish to register the depth)

    cout << "Device opening ..." << endl;

    VideoCapture capture;
    capture.open( CAP_OPENNI );

    if( !capture.isOpened() )
    {
        cout << "Can not open a capture object." << endl;
        return -1;
    }

    // We don't want registration on, since we're going to do it ourselves.
    // Some devices with RGB cameras can perform registration on device
    bool modeRes=false;
    modeRes = capture.set( CAP_PROP_OPENNI_REGISTRATION, 0 );

    if (!modeRes) {
        cout << "Can't disable registration. That's crazy!\n" << endl;
        return -1;
    }

    // Display the current configuration
    cout << "\nDepth generator output mode:" << endl <<
        "FRAME_WIDTH      " << capture.get( CAP_PROP_FRAME_WIDTH ) << endl <<
        "FRAME_HEIGHT     " << capture.get( CAP_PROP_FRAME_HEIGHT ) << endl <<
        "FRAME_MAX_DEPTH  " << capture.get( CAP_PROP_OPENNI_FRAME_MAX_DEPTH ) << " mm" << endl <<
        "FPS              " << capture.get( CAP_PROP_FPS ) << endl <<
        "REGISTRATION     " << capture.get( CAP_PROP_OPENNI_REGISTRATION ) << endl;


    //---------------------------------------
    // Specify camera properties and geometry
    //--------------------------------------

    //TODO: Fill in the values for your setup.

    // Depth camera intrinsics
    Matx33f unregisteredCameraMatrix = Matx33f::eye();
    unregisteredCameraMatrix(0,0) = 570.0f;
    unregisteredCameraMatrix(1,1) = 570.0f;
    unregisteredCameraMatrix(0,2) = 320.0f-0.5f;
    unregisteredCameraMatrix(1,2) = 240.0f-0.5f;

    // NOTE: The depth distortion coefficients are currently not used by the Registration class.
    Vec<float, 5> unregisteredDistCoeffs(0,0,0,0,0);


    // RGB camera intrinsics
    Matx33f registeredCameraMatrix = Matx33f::eye();
    registeredCameraMatrix(0,0) = 570.0f;
    registeredCameraMatrix(1,1) = 570.0f;
    registeredCameraMatrix(0,2) = 320.0f-0.5f;
    registeredCameraMatrix(1,2) = 240.0f-0.5f;

    Vec<float, 5> registeredDistCoeffs(0,0,0,0,0);

    Size2i registeredImagePlaneSize = Size2i(640, 480);

    // The rigid body transformation between cameras.
    // Used as: uv_rgb = K_rgb * [R | t] * z * inv(K_ir) * uv_ir
    Matx44f registrationRbt = Matx44f::eye();
    registrationRbt(0,3) = .04;


    //------------------------------
    // Create our registration class
    //------------------------------
    oc::Registration registration(unregisteredCameraMatrix,
                                  unregisteredDistCoeffs,
                                  registeredCameraMatrix,
                                  registeredDistCoeffs,
                                  registrationRbt);

    for (;;) {

        Mat_<uint16_t> depthMap;

        if( !capture.grab() )
        {
            cout << "Can't grab depth." << endl;
            return -1;
        }
        else
        {
            if( capture.retrieve( depthMap, CAP_OPENNI_DEPTH_MAP ) )
            {

                // Actually perform the registration
                Mat_<uint16_t> registeredDepth;
                bool performDilation = false;
                registration.registerDepthToColor(depthMap,
                                                  registeredImagePlaneSize,
                                                  registeredDepth,
                                                  performDilation);


                //Display the unregistered and registered depth
                const float scaleFactor = 0.05f;
                {
                    Mat_<uint8_t> show;
                    depthMap.convertTo( show, CV_8UC1, scaleFactor );
                    imshow( "depth map", show );
                }
                {
                    Mat_<uint8_t> show;
                    registeredDepth.convertTo( show, CV_8UC1, scaleFactor );
                    imshow( "registered map", show );
                }

            }

        }

        if( waitKey( 1 ) >= 0 )
            break;
    }



    return 0;
}
