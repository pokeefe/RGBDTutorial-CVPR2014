# Depth Color Registration #

Registering depth to an external color camera.

To use this, edit the camera intrinsics and extrinsics in main.cpp that correspond to your actual setup. (Not needed to just see something happening)

Also, RGB camera access and frame synchronization aren't included here, since those will vary wildly depending on your hardware.


## Building ##

    mkdir build
    cd build
    cmake ..
    make

