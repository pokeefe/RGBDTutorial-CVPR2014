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

#if CV_VERSION_MAJOR == 3
#include "opencv2/calib3d.hpp"
#include "opencv2/rgbd.hpp"
#else
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/rgbd/rgbd.hpp"
#endif

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

        // Figure out whether we'll have to apply a distortion
        _has_distortion = false;
        for(unsigned char i = 0; i < 5; ++i)
            _has_distortion |= (unregisteredDistCoeffs(i) != 0);

        // A point (i,j,1) will have to be converted to 3d first, by multiplying it by K.inv()
        // It will then be transformed by _rbtRgb2Depth
        cv::Matx44f K = cv::Matx44f::zeros();
        for(unsigned char j = 0; j < 3; ++j)
            for(unsigned char i = 0; i < 3; ++i)
                K(j, i) = _unregisteredCameraMatrix(j, i);
        K(3, 3) = 1;

        if (_has_distortion)
            _projection = _rbtRgb2Depth * K.inv();
        else {
            // In case there is no distortion, projecting it is just applying _registeredCameraMatrix
            _projection = cv::Matx44f::zeros();
            for(unsigned char j = 0; j < 3; ++j)
                for(unsigned char i = 0; i < 3; ++i)
                    _projection(j, i) = _registeredCameraMatrix(j, i);
            _projection(3, 3) = 1;
            _projection = _projection * _rbtRgb2Depth * K.inv();
        }
    }

    void Registration::registerDepthToColor(const cv::Mat_<uint16_t>& unregisteredDepthMillimeters,
                                            const cv::Size& outputImagePlaneSize,
                                            cv::Mat_<uint16_t>& registeredDepth,
                                            bool depthDilation)
    {

        // Create out output Mat filled with an initial value of 0
        registeredDepth = cv::Mat_<uint16_t>::zeros(outputImagePlaneSize);

        cv::Rect registeredDepthBounds(cv::Point(), registeredDepth.size());

        Mat_<cv::Point3f> transformedCloud;
        {
            Mat_<cv::Point3f> point_tmp(outputImagePlaneSize);
            for(size_t j = 0; j < point_tmp.rows; ++j) {
                const uint16_t *depth = unregisteredDepthMillimeters[j];

                cv::Point3f *point = point_tmp[j];
                for(size_t i = 0; i < point_tmp.cols; ++i, ++depth, ++point) {
                    float rescaled_depth = float(*depth) / 1000.0;
                    point->x = i * rescaled_depth;
                    point->y = j * rescaled_depth;
                    point->z = rescaled_depth;
                }
            }

            perspectiveTransform(point_tmp, transformedCloud, _projection);
        }

        std::vector<Point2f> outputProjectedPoints(transformedCloud.cols);

        for( int y = 0; y < transformedCloud.rows; y++ )
        {
            if (_has_distortion) {
                // Project an entire row of points. This has high overhead, so doing this for each point would be slow.
                // Doing this for the entire image at once would require more memory.
                projectPoints(transformedCloud.row(y),
                          cv::Vec3f(0,0,0),
                          cv::Vec3f(0,0,0),
                          _registeredCameraMatrix,
                          _registeredDistCoeffs,
                          outputProjectedPoints);
            } else {
                // With no distortion, we can project the points right up
                cv::Point2f *point2d = &outputProjectedPoints[0],
                            *point2d_end = point2d + outputProjectedPoints.size();
                cv::Point3f *point3d = transformedCloud[y];
                for( ; point2d < point2d_end; ++point2d, ++point3d ) {
                    point2d->x = point3d->x / point3d->z;
                    point2d->y = point3d->y / point3d->z;
                }
            }
            cv::Point2f *outputProjectedPoint = &outputProjectedPoints[0];
            cv::Point3f *p = transformedCloud[y], *p_end = p + transformedCloud.cols;

            for( ; p < p_end; ++outputProjectedPoint, ++p )
            {
                // Go back to millimeters, since that's what our output will be
                float cloudDepthMillimeters = 1e3*p->z;

                //Cast to integer pixel location
                Point2i projectedPixelLocation = *outputProjectedPoint;

                // Ensure that the projected point is actually contained in our output image
                if (!registeredDepthBounds.contains(projectedPixelLocation))
                    continue;

                uint16_t& outputDepthLocation = registeredDepth(projectedPixelLocation.y, projectedPixelLocation.x);


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

                        uint16_t& outputDepthLocation = registeredDepth(dilatedCoordinates.y, dilatedCoordinates.x);

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
