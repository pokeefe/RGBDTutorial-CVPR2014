/*
    This file is part of the Structure SDK.
    Copyright © 2014 Occipital, Inc. All rights reserved.
    http://structure.io
*/

#pragma once

#import <Structure/Structure.h>
#import <CoreMotion/CoreMotion.h>
#import <GLKit/GLKit.h>

//------------------------------------------------------------------------------
#pragma mark - STMesh

/** Reference to face-vertex triangle mesh data.

Stores mesh data as a collection of vertices and faces. STMesh objects are references, and access to the underlying data should be protected by locks in case multiple threads may be accessing it.

Since OpenGL ES only supports 16 bits unsigned short for face indices, meshes larges than 65535 faces have to be split into smaller submeshes. STMesh is therefore a reference to a collection of partial meshes, each of them having less than 65k faces.
*/
ST_API
@interface STMesh : NSObject

/// Number of partial meshes.
- (int)numberOfMeshes;

/// Number of faces of a given submesh.
- (int)numberOfMeshFaces:(int)meshIndex;

/// Number of lines of a given submesh.
- (int)numberOfMeshLines:(int)meshIndex;

/// Number of vertices of a given submesh.
- (int)numberOfMeshVertices:(int)meshIndex;

/** Pointer to a contigous chunk of `numberOfMeshVertices:meshIndex` `GLKVector3` values representing per-vertex normals (nx, ny, nz).

@return This method returns NULL is there are no normals.
*/
- (GLKVector3 *)meshNormals:(int)meshIndex;

/// Pointer to a contigous chunk of `numberOfMeshVertices:meshIndex` `GLKVector3` values representing vertices coordinates (x, y, z).
- (GLKVector3 *)meshVertices:(int)meshIndex;

/// Pointer to a contigous chunk of `(3 * numberOfMeshFaces:meshIndex)` 16 bits `unsigned short` values representing vertex indices. Each face is represented by three vertex indices.
- (unsigned short *)meshFaces:(int)meshIndex;

/** STMesh can also return a polygon mesh as a sequence of lines.

@return This method returns a pointer to a contigous chunk of `(2 * numberOfMeshLines:meshIndex)` 16 bits `unsigned short` values representing vertex indices. Each line is represented by two vertex indices.
*/
- (unsigned short *)meshLines:(int)meshIndex;

/// Save the mesh as a Wavefront OBJ file.
- (void)writeToObjFile:(NSString *)fileName;

/// Create a copy of the current mesh
- (id)initWithMesh:(STMesh*)mesh;

/** Create a decimated (simplified) mesh from the current mesh with a target number of faces. `numFaces` can range from 1 to 65535. If target number of faces is larger than the current mesh number of faces, no processing is done.
 
@return This method returns a new decimated (simplified) mesh. `nil` is returned on error.
*/
- (STMesh *)meshFromDecimation:(unsigned int)numFaces error:(NSError **)error;

@end

//------------------------------------------------------------------------------
# pragma mark - STScene

/** Common data shared and updated by the SLAM pipeline.

An STScene object regroup information about the camera and the reconstructed mesh.
SLAM objects will be updating the scene, potentially using background threads.
As a result, special care should be taken when accessing STScene members if an STTracker or STMapper is still active.
In particular, STMesh objects should be propertly locked.
*/
ST_API
@interface STScene : NSObject

/** Mandatory initializer for STScene.

@param glContext a valid EAGLContext.
@param frameBufferSize size of the active view framebuffer.
@param sensorInfo Structure Sensor information.
@param freeGLTextureUnit a GL_TEXTUREX unit which will be used when SLAM objects need to render meshes to an OpenGL texture.
*/
- (id) initWithContext:(EAGLContext *)glContext
       frameBufferSize:(CGSize)frameBufferSize
            sensorInfo:(struct STSensorInfo *)sensorInfo
     freeGLTextureUnit:(GLenum)textureUnit;

/** Reference to the current scene mesh.

This mesh may be modified by a background thread if an instance of STMapper is running, so proper locking is necessary.
*/
- (STMesh *)lockAndGetSceneMesh;

/// Unlock the mesh
- (void) unlockSceneMesh;

/** OpenGL projection matrix representing a Structure Sensor virtual camera.

This matrix can be used to render a scene by simulating the same camera properties as the Structure Sensor depth camera.
*/
- (GLKMatrix4)depthCameraGLProjectionMatrix;

/** Render the scene mesh from the given viewpoint.

A virtual camera with the given projection and pose matrices will be used to render the mesh using OpenGL. This method is generally faster than using sceneMeshRef and manually rendering it, since in most cases STScene can reuse mesh data previously uploaded to the GPU.
*/
- (void)renderMeshFromViewpoint:(GLKMatrix4)cameraPose
             cameraGLProjection:(GLKMatrix4)gLProjection
                          alpha:(float)alpha;

/// Clear the scene mesh and state.
- (void)clear;

@end

//------------------------------------------------------------------------------
# pragma mark - STTracker

/** Track the 3D position of the Structure Sensor.

STTracker uses sensor information and optionally IMU data to estimate how the camera is being moved over time, in real-time.
*/
ST_API
@interface STTracker : NSObject

/// STScene object storing common SLAM information.
@property (nonatomic, retain) STScene *scene;

/// Recommended initializer since STTracker cannot be used until an STScene has been provided.
- (id)initWithScene:(STScene *)scene;

/// Reset the tracker to its initial state.
- (void)reset;

/// Set the current camera pose. Tracking will take this as the initial pose.
- (void)setCameraPose:(GLKMatrix4)cameraPose;

/** Update the camera pose estimate using the given depth frame.

Returns true if success, false otherwise.
*/
- (BOOL)updateCameraPoseFromDepth:(STFloatDepthFrame *)depthFrame;

/// Update the current pose estimates using the provided motion data.
- (void)updateCameraPoseFromMotion:(CMDeviceMotion *)motionData;

/// Return the most recent camera pose estimate.
- (GLKMatrix4)lastCameraPose;

// The better mode with integration of timestamp
- (GLKMatrix4) lastCameraPoseWtihTimestamp:(double)timestamp;

/** Tracking Mode adjustement.
 
STTracker can run in two different modes. STTrackingModeAccurate is best during scanning, but it
will also take more CPU resources. STTrackingModeFast is designed for very fast tracking, and works
best when tracking against static mesh, for example after a scan has already been done.
*/
enum STTrackingMode {
    STTrackingModeAccurate = 0, // best for scanning, but uses more CPU. This is the default.
    STTrackingModeFast, // will use less CPU, this mode is best for 30 FPS tracking against a fixed mesh
};

- (void)setTrackingMode:(STTrackingMode)mode;

@end

//------------------------------------------------------------------------------
# pragma mark - STMapper

/** Integrate sensor data to reconstruct a 3D model of a scene.

STMapper will update the scene mesh progressively as new depth frames are fed.
It works in a background thread, which means that it may update the STScene object at any time.
You need to call the blocking stop method to make sure mapping has fully stopped.

The mapper works over a fixed cuboid defining the volume of interest in the scene.
This volume can be initialized interactively using STCubePlacementInitializer.

The volume is defined by its size in the real world, in meters, and is discretized into cells.
The volume resolution specifies the number of cells. As a consequence, the maximal level of detail which can be obtained by STMapper is roughly determined by volumeSizeInMeters / volumeResolution.
In short, the bigger the volume size, the higher the resolution has to be to keep the same level of details.
*/
ST_API
@interface STMapper : NSObject

/// The STScene model which will be updated.
@property (nonatomic, retain) STScene *scene;

/// The rectangular cuboid size in meters.
@property (nonatomic) GLKVector3 volumeSizeInMeters;

/** Number of cells for each dimension.

To keep the level of details isotropic, it is recommended to use a similar same aspect ratio as volumeSizeInMeters.
To keep mapping real-time, the recommended value is 128x128x128.

@note The volume resolution cannot be changed after initialization.
*/
@property (nonatomic, readonly) GLKVector3 volumeResolution;

/// Initialize with a given scene and volume resolution.
- (id)initWithScene:(STScene *)scene
   volumeResolution:(GLKVector3)volumeResolution;

/** Specify whether the volume cuboid has been initialized on top of a support plane.

If the mapper is aware that the volume in on top of a support plane, it will adapt the pipeline to be more robust.
*/
- (void)setHasSupportPlane:(BOOL)hasIt;

/// Stop any processing which may still be happening in background threads.
- (void)stop;

/// Reset the mapper state.
- (void)reset;

/// Integrate a new depth frame to the model.
- (void)integrateDepthFrame:(STFloatDepthFrame *)depthFrame
                 cameraPose:(GLKMatrix4)cameraPose;

@end

//------------------------------------------------------------------------------
# pragma mark - STCubePlacementInitializer

/** Automatically and interactively place a cubic volume of interest in the scene.

This class uses an heuristic to help a user select the volume of interest to be scanned in a scene.
If it can determines a supporting table, e.g. if an object is on a table or lying on the floor, then it will align the base of the cuboid with the plane.
Otherwise it will initialize the cube around the depth of the central area of the depth image.
*/
ST_API
@interface STCubePlacementInitializer : NSObject

/// Structure Sensor information. Required.
@property (nonatomic) STSensorInfo *cameraInfo;

/// Width, height and depth of the volume cuboid.
@property (nonatomic) GLKVector3 volumeSizeInMeters;

/// Most recent estimated cube 3D pose, taking Structure Sensor as a reference.
@property (nonatomic, readonly) GLKMatrix4 cubePose;

/// Whether the last cube placement was made with a supporting plane. Useful for STMapper.
@property (nonatomic, readonly) BOOL hasSupportPlane;

/// Initialize with all the required fields.
- (id)initWithCameraInfo:(STSensorInfo *)cameraInfo
      volumeSizeInMeters:(GLKVector3)volumeSize;

/// Update the current pose estimate from a depth frame and a CoreMotion gravity vector.
- (void)updateCubePose:(STFloatDepthFrame *)frame
               gravity:(GLKVector3)gravity;

@end

//------------------------------------------------------------------------------
# pragma mark - STCubeRenderer

/** Helper class to render a cuboid.

STCubeRenderer can render a wireframe outline of a cube, and also highlight the part of scene which fits in the given cube.
This can be used to better visualize where the current cube is located.
*/

ST_API
@interface STCubeRenderer : NSObject

/// The global SLAM scene.
@property (nonatomic, retain) STScene *scene;

/// Initialize with required properties.
- (id)initWithScene:(STScene *)scene;

/// A depth frame is required to use renderHighlightedDepth.
- (void)setDepthFrame:(STFloatDepthFrame *)depthFrame;

/// Whether the cube has a support plane. Rendering will be adjusted in that case.
- (void)setCubeHasSupportPlane:(BOOL)hasSupportPlane;

/// Specify the cube size and the volume resolution in cells.
- (void)adjustCubeSize:(const GLKVector3)sizeInMeters
      volumeResolution:(const GLKVector3)resolution;

/// Highlight the depth frame area which fits inside the cube.
- (void)renderHighlightedDepth:(GLKMatrix4)cubePose;

/// Render the cube wireframe outline at the given pose.
- (void)renderCubeOutline:(GLKMatrix4)cubePose
                depthTest:(BOOL)useDepthTest;

@end

//------------------------------------------------------------------------------
# pragma mark - STNormalFrame

/** Processed normal frame with normal vector in each pixel.
 
Output class from STNormalEstimatior.
*/
ST_API
@interface STNormalFrame : NSObject

/// Image width.
@property (readonly, nonatomic) int width;

/// Image height.
@property (readonly, nonatomic) int height;

/// Pointer to the beginning of a contiguous chunk of (`width` * `height`) normal pixel values.
@property (readonly, nonatomic) const GLKVector3 *normals;

@end

//------------------------------------------------------------------------------
# pragma mark - STNormalEstimator

/** Helper class to estimate surface normal.
 
STNormalEstimator calculates a unit vector representing the surface normal for each depth pixel.
*/

ST_API
@interface STNormalEstimator : NSObject

/// Init with required STSensorInfo data.
- (id)initWithSensorInfo:(STSensorInfo *)sensorInfo;

/// Calculates normals with a depth frame.
- (STNormalFrame *)calculateNormalsWithProcessedFrame:(STFloatDepthFrame *)floatDepthFrame;

@end

//------------------------------------------------------------------------------
# pragma mark - STGLTexture

/** Helper class to manipulation OpenGL textures.

This class makes it easier to initialize a GL texture.
It will use texture cache if the program is running on device, and regular GL textures on simulator.
*/
ST_API
@interface STGLTexture : NSObject

/// OpenGL id of the texture.
@property (nonatomic, readonly) GLint glId;

/// Whether initWithContext or createWithContext was already called.
@property (nonatomic, readonly) BOOL isInitialized;

/// Initialize the texture and immediatly call createWithContext.
- (id)initWithContext:(EAGLContext *)context
                width:(int)width
               height:(int)height
            sizeBytes:(int)sizeBytes
             glFormat:(GLenum)format
               glType:(GLenum)type;

/// Initialize the underlying GL texture.
- (void)createWithContext:(EAGLContext *)context
                    width:(int)width
                   height:(int)height
                sizeBytes:(int)sizeBytes
                 glFormat:(GLenum)format
                   glType:(GLenum)type;

/// Upload the texture data to GPU.
- (void)uploadData:(uint8_t *)data;

/// Bind the texture. Equivalent to `glBindTexture(glId)` in simulator, but also support texture cache.
- (void)bind;

@end

//------------------------------------------------------------------------------
# pragma mark - STGLTextureShaderRGBA

/// Helper class to render a flat 2D texture with OpenGL ES.
ST_API
@interface STGLTextureShaderRGBA : NSObject

/// Enable the underlying shader program.
- (void)useShaderProgram;

/// Render the texture on a fullscreen quad using the given GL_TEXTUREX unit.
- (void)renderTextureWithAlpha:(float)alpha
                   textureUnit:(GLint)textureUnit;

@end

//------------------------------------------------------------------------------
# pragma mark - STDepthToRgba

/// Helper class to convert float depth data to RGB values for better visualization.
ST_API
@interface STDepthToRgba : NSObject

/// Pointer to the RGBA values.
@property (nonatomic, readonly) uint8_t *rgbaBuffer;

/// Init with required STSensorInfo data.
- (id)initWithSensorInfo:(STSensorInfo *)sensorInfo;

/// Convert the given depth frame to RGBA.
- (uint8_t *)convertDepthToRgba:(STFloatDepthFrame *)frame;

@end
