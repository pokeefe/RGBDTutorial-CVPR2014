//
// Registration.cpp
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


#include <stdio.h>
#include <iostream>
#include "Registration.h"

#include "opencv2/calib3d.hpp"
#include "opencv2/rgbd.hpp"

using namespace cv;

namespace oc {

    Registration::Registration(const cv::Matx33f& unregisteredCameraMatrix,
                               const cv::Vec<float, 5>& unregisteredDistCoeffs,
                               const cv::Matx33f& registeredCameraMatrix,
                               const cv::Vec<float, 5>& registeredDistCoeffs,
                               const cv::Matx44f& Rt) {

        _unregisteredCameraMatrix = unregisteredCameraMatrix;
        _unregisteredDistCoeffs = unregisteredDistCoeffs;

        _registeredCameraMatrix = registeredCameraMatrix;
        _registeredDistCoeffs = registeredDistCoeffs;

        _rbtRgb2Depth = Rt;

    }

    void Registration::registerDepthToColor(const cv::Mat_<uint16_t>& unregisteredDepthMillimeters,
                                            const cv::Size& outputImagePlaneSize,
                                            cv::Mat_<uint16_t>& registeredDepth,
                                            bool depthDilation)
    {

        // Create out output Mat filled with an initial value of 0
        registeredDepth.create(outputImagePlaneSize);
        registeredDepth.setTo(0);

        cv::Rect registeredDepthBounds(cv::Point(), registeredDepth.size());

        Mat3f cloud, transformedCloud;

        // Unproject the points to obtain an XYZ cloud
        // Output will be scaled to meters as floats
        depthTo3d(unregisteredDepthMillimeters, _unregisteredCameraMatrix, cloud);

        // Transform the cloud by the rbt between our cameras
        perspectiveTransform(cloud, transformedCloud, _rbtRgb2Depth);


        std::vector<Point2f> outputProjectedPoints(transformedCloud.cols);

        for( int y = 0; y < transformedCloud.rows; y++ )
        {

            // Project an entire row of points. This has high overhead, so doing this for each point would be slow.
            // Doing this for the entire image at once would require more memory.
            projectPoints(transformedCloud.cv::Mat::row(y),
                          Mat::zeros(3, 1, CV_32FC1),
                          Mat::zeros(3, 1, CV_32FC1),
                          _registeredCameraMatrix,
                          _registeredDistCoeffs,
                          outputProjectedPoints);

            Point3f* transformedCloudRowPtr = (Point3f*)transformedCloud.ptr(y);

            for( int x = 0; x < transformedCloud.cols; x++ )
            {

                Point3f& p = (*transformedCloudRowPtr++);

                // Go back to millimeters, since that's what our output will be
                float cloudDepthMillimeters = 1e3*p.z;

                //Cast to integer pixel location
                Point2i projectedPixelLocation = outputProjectedPoints[x];

                // Ensure that the projected point is actually contained in our output image
                if (!registeredDepthBounds.contains(projectedPixelLocation))
                    continue;

                uint16_t& outputDepthLocation = registeredDepth.at<uint16_t>(projectedPixelLocation.y, projectedPixelLocation.x);


                // Occlusion check
                if ( outputDepthLocation == 0 || (outputDepthLocation > cloudDepthMillimeters) ) {
                    outputDepthLocation = cloudDepthMillimeters;
                }


                // If desired, dilate this point to avoid holes in the final image
                if (depthDilation) {

                    // Choosing to dilate in a 2x2 region, where the original projected location is in the bottom right of this
                    // region. This is what's done on PrimeSense devices, but a more accurate scheme could be used.
                    Point2i dilatedProjectedLocations[3] = {Point2i(projectedPixelLocation.x - 1, projectedPixelLocation.y    ),
                                                            Point2i(projectedPixelLocation.x    , projectedPixelLocation.y - 1),
                                                            Point2i(projectedPixelLocation.x - 1, projectedPixelLocation.y - 1)};


                    for (int i = 0; i < 3; i++) {

                        Point2i& dilatedCoordinates = dilatedProjectedLocations[i];

                        if (!registeredDepthBounds.contains(dilatedCoordinates))
                            continue;

                        uint16_t& outputDepthLocation = registeredDepth.at<uint16_t>(dilatedCoordinates.y, dilatedCoordinates.x);

                        // Occlusion check
                        if ( outputDepthLocation == 0 || (outputDepthLocation > cloudDepthMillimeters) ) {
                            outputDepthLocation = cloudDepthMillimeters;
                        }

                    }

                } // depthDilation

            } // iterate over a row
        } // iterate over the columns

    }


}
