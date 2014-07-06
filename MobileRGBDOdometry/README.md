# Mobile OpenCV RGBD Odometry #

This sample demonstrates how easy it is to integrate OpenCV's RGBD odometry into a minimal sample project from the Structure SDK. The odometry is a part of the RGBD module, which can be found in the [OpenCV contrib repository](https://github.com/Itseez/opencv_contrib). Currently, there is no visualization of the odometry result (it's just printed to the console). You can roll your own VR or AR game, or even do some large scale mapping.

For more information about the Structure Sensor and the Structure SDK, visit [this website](http://structure.io/).


## Building ##

Not included is a binary of `opencv2.framework` that includes the RGBD module. You can download a binary of pre-alpha 3.0 [here.](https://www.dropbox.com/s/l9yu7hzv9j3d5qh/opencv2.framework.zip)

If you'd like to build `opencv2.framework` yourself with the contrib modules, check out `build_opencv_ios_with_contrib.diff` and apply it to your opencv repo. It assumes that opencv_contrib has been cloned on the same level as the main OpenCV repositiory.
