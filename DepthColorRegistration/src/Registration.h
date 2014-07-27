//
// Registration.h
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

#if CV_VERSION_MAJOR == 3
#include "opencv2/core.hpp"
#else
#include "opencv2/core/core.hpp"
#endif

namespace oc {


    /** Class for registering depth data to an external color camera
     * Registration is performed by creating a depth cloud, transforming the cloud by
     * the rigid body transformation between the cameras, and then projecting the
     * transformed points into the RGB camera.
     *
     * uv_rgb = K_rgb * [R | t] * z * inv(K_ir) * uv_ir
     */
    class Registration {
    public:

        /** Constructor
         * @param unregisteredCameraMatrix the camera matrix of the depth camera
         * @param unregisteredDistCoeffs the distortion coefficients of the depth camera. NOTE: CURRENTLY UNUSED
         * @param registeredCameraMatrix the camera matrix of the external RGB camera
         * @param registeredDistCoeffs the distortion coefficients of the RGB camera
         * @param Rt the rigid body transform between the cameras. Used as seen above.
         */
        Registration(const cv::Matx33f& unregisteredCameraMatrix,
                     const cv::Vec<float, 5>& unregisteredDistCoeffs,
                     const cv::Matx33f& registeredCameraMatrix,
                     const cv::Vec<float, 5>& registeredDistCoeffs,
                     const cv::Matx44f& Rt);

        /** Performs the registration
         * @param unregisteredDepthMillimeters the raw depth from the depth camera in millimieters
         * @param outputImagePlaneSize the dimensions of the registered image in pixels
         * @param registeredDepth the final registered depth in millimeters
         * @param depthDilation whether or not the depth is dilated to avoid holes and occlusion errors (optional)
         */
        void registerDepthToColor(const cv::Mat_<uint16_t>& unregisteredDepthMillimeters,
                                  const cv::Size& outputImagePlaneSize,
                                  cv::Mat_<uint16_t>& registeredDepth,
                                  bool depthDilation=false);

    private:

        cv::Matx33f _unregisteredCameraMatrix;
        cv::Vec<float, 5> _unregisteredDistCoeffs;

        cv::Matx33f _registeredCameraMatrix;
        cv::Vec<float, 5> _registeredDistCoeffs;

        cv::Matx44f _rbtRgb2Depth, _projection;

        bool _has_distortion;
    };

}
