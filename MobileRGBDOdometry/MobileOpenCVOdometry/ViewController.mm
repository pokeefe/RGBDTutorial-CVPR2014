/*
  This file is part of the Structure SDK.
  Copyright © 2014 Occipital, Inc. All rights reserved.
  http://structure.io
*/


#import "ViewController.h"

#import <AVFoundation/AVFoundation.h>
#import <Structure/StructureSLAM.h>

#include <iostream>
#include <opencv2/rgbd.hpp>
#include <opencv2/imgproc.hpp>

#define CONNECT_TEXT @"Please Connect Structure Sensor"
#define CHARGE_TEXT @"Please Charge Structure Sensor"


@interface ViewController () <AVCaptureVideoDataOutputSampleBufferDelegate> {
    
    STSensorController *_sensorController;
    
    AVCaptureSession *_session;

    UIImageView *_depthImageView;
    UIImageView *_normalsImageView;
    UIImageView *_colorImageView;
    
    uint16_t *_linearizeBuffer;
    uint8_t *_coloredDepthBuffer;
    uint8_t *_normalsBuffer;

    STFloatDepthFrame *_floatDepthFrame;
    STNormalEstimator *_normalsEstimator;
    
    UILabel* _statusLabel;
    
    cv::Odometry* odometry;
    
    cv::Ptr<cv::OdometryFrame> prevOdometryFrame;
    cv::Ptr<cv::OdometryFrame> currOdometryFrame;

    std::vector<cv::Mat> allOdometryPoses;

}

- (BOOL)connectAndStartStreaming;
- (void)renderDepthFrame:(STDepthFrame*)depthFrame;
- (void)renderNormalsFrame:(STDepthFrame*)normalsFrame;
- (void)renderColorFrame:(CMSampleBufferRef)sampleBuffer;
- (void)startAVCaptureSession;

- (void)setupOpenCVOdometry;

@end

@implementation ViewController


- (void)viewDidLoad
{
    [super viewDidLoad];
    
    _sensorController = [STSensorController sharedController];
    _sensorController.delegate = self;
    
    // Request that we receive depth frames with synchronized color pairs
    [_sensorController setFrameSyncConfig:FRAME_SYNC_DEPTH_AND_RGB];
    

    // Create three image views where we will render our frames
    
    CGRect depthFrame = self.view.frame;
    depthFrame.size.height /= 2;
    depthFrame.origin.y = self.view.frame.size.height/2;
    depthFrame.origin.x = 1;
    depthFrame.origin.x = -self.view.frame.size.width * 0.25;
    
    CGRect normalsFrame = self.view.frame;
    normalsFrame.size.height /= 2;
    normalsFrame.origin.y = self.view.frame.size.height/2;
    normalsFrame.origin.x = 1;
    normalsFrame.origin.x = self.view.frame.size.width * 0.25;
    
    CGRect colorFrame = self.view.frame;
    colorFrame.size.height /= 2;
    
    _linearizeBuffer = NULL;
    _coloredDepthBuffer = NULL;
    _normalsBuffer = NULL;

    _depthImageView = [[UIImageView alloc] initWithFrame:depthFrame];
    _depthImageView.contentMode = UIViewContentModeScaleAspectFit;
    [self.view addSubview:_depthImageView];
    
    _normalsImageView = [[UIImageView alloc] initWithFrame:normalsFrame];
    _normalsImageView.contentMode = UIViewContentModeScaleAspectFit;
    [self.view addSubview:_normalsImageView];
    
    _colorImageView = [[UIImageView alloc] initWithFrame:colorFrame];
    _colorImageView.contentMode = UIViewContentModeScaleAspectFit;
    [self.view addSubview:_colorImageView];
    

    // When the app enters the foreground, we can choose to restart the stream
    [[NSNotificationCenter defaultCenter] addObserver:self selector:@selector(appWillEnterForeground) name:UIApplicationWillEnterForegroundNotification object:nil];

    #if !TARGET_IPHONE_SIMULATOR
    [self startAVCaptureSession];
    #endif
    
    [self setupOpenCVOdometry];
    
    // Sample usage of wireless debugging API
//    NSError* error = nil;
//    [STWirelessLog broadcastLogsToWirelessConsoleAtAddress:@"10.1.10.44" usingPort:4999 error:&error];
//
//    if (error)
//        NSLog(@"Oh no! Can't start wireless log: %@", [error localizedDescription]);

}

- (void)dealloc
{
    if (_linearizeBuffer)
        free(_linearizeBuffer);
    
    if (_coloredDepthBuffer)
        free(_coloredDepthBuffer);
    
    if (_normalsBuffer)
        free(_normalsBuffer);
}


- (void)viewDidAppear:(BOOL)animated
{
    static BOOL fromLaunch = true;
    if(fromLaunch)
    {

        //
        // Create a UILabel in the center of our view to display status messages
        //
    
        // We do this here instead of in viewDidLoad so that we get the correctly size/rotation view bounds
        if (!_statusLabel) {
            
            _statusLabel = [[UILabel alloc] initWithFrame:self.view.bounds];
            _statusLabel.backgroundColor = [[UIColor blackColor] colorWithAlphaComponent:0.7];
            _statusLabel.textAlignment = NSTextAlignmentCenter;
            _statusLabel.font = [UIFont systemFontOfSize:35.0];
            
            [_statusLabel setText:CONNECT_TEXT];
            [_statusLabel setTextColor:[UIColor whiteColor]];
            [self.view addSubview: _statusLabel];
        }

        [self connectAndStartStreaming];
        fromLaunch = false;
    }
}


- (void)appWillEnterForeground
{

    BOOL success = [self connectAndStartStreaming];
    
    if(!success)
    {
        // Workaround for direct multitasking between two Structure Apps.
        
        // HACK ALERT! Try once more after a delay if we failed to reconnect on foregrounding.
        // 0.75s was not enough, 0.95s was, but this might depend on the other app using the sensor.
        // We need a better solution to this.
        [NSTimer scheduledTimerWithTimeInterval:2.0 target:self
                                       selector:@selector(connectAndStartStreaming) userInfo:nil repeats:NO];
    }

}


- (void)didReceiveMemoryWarning
{
    [super didReceiveMemoryWarning];
    // Dispose of any resources that can be recreated.
}


- (BOOL)connectAndStartStreaming
{
    
    STSensorControllerInitStatus result = [_sensorController initializeSensorConnection];
    
    BOOL didSucceed = (result == STSensorControllerInitStatusSuccess || result == STSensorControllerInitStatusAlreadyInitialized);
    
    
    if (didSucceed)
    {
        // Now that we're about to stream, hide the status label
        [self hideStatusMessage];
        
        // Set sensor stream quality
        StructureStreamConfig streamConfig = CONFIG_QVGA_DEPTH;
        
        // After this call, we will start to receive frames through the delegate methods
        [_sensorController startStreamingWithConfig:streamConfig];
        
        // Allocate the depth (shift) -> to depth (millimeters) converter class
        _floatDepthFrame = [[STFloatDepthFrame alloc] init];
        
        // Allocate the depth -> surface normals converter class
        _normalsEstimator = [[STNormalEstimator alloc] initWithSensorInfo:[_sensorController getSensorInfo:streamConfig]];
    }
    else
    {
        if (result == STSensorControllerInitStatusSensorNotFound)
            NSLog(@"[Debug] No Structure Sensor found!");
        else if (result == STSensorControllerInitStatusOpenFailed)
            NSLog(@"[Error] Structure Sensor open failed.");
        else if (result == STSensorControllerInitStatusSensorIsWakingUp)
            NSLog(@"[Debug] Structure Sensor is waking from low power.");
        else if (result != STSensorControllerInitStatusSuccess)
            NSLog(@"[Debug] Structure Sensor failed to init with status %d.", (int)result);
        
        [self showStatusMessage:CONNECT_TEXT];
    }
    
    return didSucceed;
    
}


- (void)showStatusMessage:(NSString *)msg
{

    _statusLabel.hidden = false;
    _statusLabel.text = msg;

}

- (void)hideStatusMessage
{
    _statusLabel.hidden = true;
}

- (void)setupOpenCVOdometry
{
    
    prevOdometryFrame = cv::Ptr<cv::OdometryFrame>(new cv::OdometryFrame());
    currOdometryFrame = cv::Ptr<cv::OdometryFrame>(new cv::OdometryFrame());
    
    cv::Mat1f cameraMatrix (3,3); cv::setIdentity(cameraMatrix);

    //TODO: Replace with device specific intrinsics

    //QVGA iOS iPad Air
    cameraMatrix(0,0) = 288.0f;
    cameraMatrix(1,1) = 288.0f;
    cameraMatrix(0,2) = 161.5f;
    cameraMatrix(1,2) = 121.5f;
    
    // OpenCV odometry ignores lens distortion
    
    float minDepth = 0.3f;
    float maxDepth = 4.f;
    float maxDepthDiff = 0.07f;
    
    std::vector<int> iterCounts = cv::Mat(cv::Vec3i(7,7,10));
    std::vector<float> minGradientMagnitudes = cv::Mat(cv::Vec3f(10,10,10));
    
    float maxPointsPart = cv::RgbdOdometry::DEFAULT_MAX_POINTS_PART();
    
    
    odometry = new cv::RgbdOdometry (cameraMatrix, minDepth, maxDepth, maxDepthDiff, iterCounts, minGradientMagnitudes, maxPointsPart,
                                     cv::Odometry::RIGID_BODY_MOTION);
    
    
}

void convertBGRASampleBufferToRGB (CMSampleBufferRef sampleBuffer, cv::Mat& dest)
{
    
    CVImageBufferRef pixelBuffer = CMSampleBufferGetImageBuffer(sampleBuffer);
    
    CVPixelBufferLockBaseAddress(pixelBuffer, 0);
    
    size_t width = CVPixelBufferGetWidth(pixelBuffer);
    size_t height = CVPixelBufferGetHeight(pixelBuffer);
    
    dest.create((int)height, (int)width, CV_8UC3);
    
    unsigned char* ptr = (unsigned char*) CVPixelBufferGetBaseAddressOfPlane(pixelBuffer, 0);
    
    // Use NEON to convert from BGRA to RGB, or use Accelerate in the case that we're on simulator
#ifdef __ARM_NEON__
    
    uint8_t* sourcePtr = (uint8_t*)ptr;
    uint8_t* destPtr = (uint8_t*)dest.data;
    
    const int numPixels = (int)(width*height);
    
    int pixel = 0;
    for (; pixel < numPixels; pixel += 16)
    {
        
        uint8x16x4_t sourcePixelsBGRA = vld4q_u8((const unsigned char*)sourcePtr);
        uint8x16x3_t sourceRGB;
        sourceRGB.val[0] = sourcePixelsBGRA.val[2];
        sourceRGB.val[1] = sourcePixelsBGRA.val[1];
        sourceRGB.val[2] = sourcePixelsBGRA.val[0];
        vst3q_u8((unsigned char *)destPtr, sourceRGB);
        
        sourcePtr += 16*4;
        destPtr += 16*3;
        
    }
    
    // Convert any leftover pixels (15 or less would remain, if any)
    for (; pixel < numPixels; pixel++) {
        uint8_t* sourceBGRAPixel = sourcePtr;
        uint8_t* destRGBPixel = destPtr;
        
        destRGBPixel[0] = sourceBGRAPixel[2];
        destRGBPixel[1] = sourceBGRAPixel[1];
        destRGBPixel[2] = sourceBGRAPixel[0];
        
        sourcePtr += 4;
        destPtr += 3;
    }
    
#else
    
    vImage_Buffer src;
    src.width = width;
    src.height = height;
    src.data = ptr;
    src.rowBytes = CVPixelBufferGetBytesPerRow(pixelBuffer);
    
    vImage_Buffer destImage;
    destImage.width = width;
    destImage.height = height;
    destImage.rowBytes = width*3;
    destImage.data = dest.data;
    
    vImage_Error err;
    err = vImageConvert_BGRA8888toRGB888(&src, &destImage, kvImageNoFlags);
    if(err != kvImageNoError){
        NSLog(@"Error in Pixel Copy vImage_error %ld", err);
    }
    
    
#endif
    
    
    CVPixelBufferUnlockBaseAddress(pixelBuffer, 0);
}


#pragma mark -
#pragma mark Structure SDK Delegate Methods

- (void)sensorDidDisconnect
{
    NSLog(@"Structure Sensor disconnected!");
    [self showStatusMessage:CONNECT_TEXT];
}

- (void)sensorDidConnect
{
    NSLog(@"Structure Sensor connected!");
    [self connectAndStartStreaming];
}

- (void)sensorDidEnterLowPowerMode
{
    // Notify the user that the sensor needs to be charged.
    [self showStatusMessage:CHARGE_TEXT];
}

- (void)sensorDidLeaveLowPowerMode
{
    
}

- (void)sensorBatteryNeedsCharging
{
    // Notify the user that the sensor needs to be charged.
    [self showStatusMessage:CHARGE_TEXT];
}

- (void)sensorDidStopStreaming:(STSensorControllerDidStopStreamingReason)reason
{
    //If needed, change any UI elements to account for the stopped stream
}

- (void)sensorDidOutputDepthFrame:(STDepthFrame *)depthFrame
{
    [self renderDepthFrame:depthFrame];
}

// This synchronized API will only be called when two frames match. Typically, timestamps are within 1ms of each other.
// Two important things have to happen for this method to be called:
// Tell the SDK we want framesync: [_ocSensorController setFrameSyncConfig:FRAME_SYNC_DEPTH_AND_RGB];
// Give the SDK color frames as they come in:     [_ocSensorController frameSyncNewColorImage:sampleBuffer];
- (void)sensorDidOutputSynchronizedDepthFrame:(STDepthFrame*)depthFrame
                                andColorFrame:(CMSampleBufferRef)sampleBuffer
{
    
    
    // Fill from STDepthFrame
    cv::Mat depth(depthFrame->height, depthFrame->width, CV_16U, depthFrame->data);
    
    // scale depth to meters
    cv::Mat depthMeters;
    depth.convertTo(depthMeters, CV_32FC1, 1.f/1000.f);
    
    
    depthMeters.setTo(std::numeric_limits<float>::quiet_NaN(), depth == 0);
    depth = depthMeters;
    
    
    // Fill from CMSampleBuffer
    cv::Mat vgaImage;
    convertBGRASampleBufferToRGB(sampleBuffer, vgaImage);
    
    //TODO: The conversion to grayscale and the decimation should all be done at once with NEON instead of separated.
    // A convertBGRASampleBufferToDecimatedGrayscale function should be used instead of convertBGRASampleBufferToRGB
    // and then these calls.
    
    cv::Mat vgaGray;
    cv::cvtColor(vgaImage, vgaGray, cv::COLOR_BGR2GRAY);
    
    cv::Mat1b qvgaGray;
    cv::resize(vgaGray, qvgaGray, cv::Size(320, 240));
    
    currOdometryFrame->image = qvgaGray;
    currOdometryFrame->depth = depth;
    
    // Compute the delta pose between this frame and the last one
    cv::Mat deltaRt;
    if(!allOdometryPoses.empty())
    {
        bool res = odometry->compute(currOdometryFrame, prevOdometryFrame, deltaRt);
        
        if(!res)
            deltaRt = cv::Mat::eye(4,4,CV_64FC1);
    }
    
    if( allOdometryPoses.empty() )
    {
        allOdometryPoses.push_back(cv::Mat::eye(4,4,CV_64FC1));
    }
    else
    {
        cv::Mat& prevRt = *allOdometryPoses.rbegin();
        allOdometryPoses.push_back( prevRt * deltaRt );
        std::cout << "Current pose: " << *allOdometryPoses.rbegin() << std::endl;
        
    }
    
    if(!prevOdometryFrame.empty())
        prevOdometryFrame->release();
    std::swap(prevOdometryFrame, currOdometryFrame);
    
    
    [self renderDepthFrame:depthFrame];
    [self renderNormalsFrame:depthFrame];
    [self renderColorFrame:sampleBuffer];
}


#pragma mark -
#pragma mark Rendering

- (void)populateLinearizeBuffer:(size_t)depthValuesCount
{
    _linearizeBuffer = (uint16_t*)malloc(depthValuesCount);
    
    int maxShiftValue = 2048;
    for (int i=0; i < maxShiftValue * 2 ; i++)
    {
        float v = i/ (float)maxShiftValue;
        v = powf(v, 3)* 6;
        _linearizeBuffer[i] = v*6*256;
    }
    
}

- (void)convertShiftToRGBA:(const uint16_t*)shiftedDepth depthValuesCount:(size_t)depthValuesCount
{
    for (size_t i = 0; i < depthValuesCount; i++)
    {
        // Use a lookup table to make the non-linear shifted depth values vary more linearly with metric depth
        int linearizedDepth = _linearizeBuffer[shiftedDepth[i]];
        
        // Use the upper byte of the linearized shift value to choose a base color
        // Base colors range from: (closest) White, Red, Orange, Yellow, Green, Cyan, Blue, Black (farthest)
        int lowerByte = (linearizedDepth & 0xff);
        
        // Use the lower byte to scale between the base colors
        int upperByte = (linearizedDepth >> 8);
        
        switch (upperByte)
        {
            case 0:
                _coloredDepthBuffer[4*i+0] = 255;
                _coloredDepthBuffer[4*i+1] = 255-lowerByte;
                _coloredDepthBuffer[4*i+2] = 255-lowerByte;
                _coloredDepthBuffer[4*i+3] = 255;
                break;
            case 1:
                _coloredDepthBuffer[4*i+0] = 255;
                _coloredDepthBuffer[4*i+1] = lowerByte;
                _coloredDepthBuffer[4*i+2] = 0;
                break;
            case 2:
                _coloredDepthBuffer[4*i+0] = 255-lowerByte;
                _coloredDepthBuffer[4*i+1] = 255;
                _coloredDepthBuffer[4*i+2] = 0;
                break;
            case 3:
                _coloredDepthBuffer[4*i+0] = 0;
                _coloredDepthBuffer[4*i+1] = 255;
                _coloredDepthBuffer[4*i+2] = lowerByte;
                break;
            case 4:
                _coloredDepthBuffer[4*i+0] = 0;
                _coloredDepthBuffer[4*i+1] = 255-lowerByte;
                _coloredDepthBuffer[4*i+2] = 255;
                break;
            case 5:
                _coloredDepthBuffer[4*i+0] = 0;
                _coloredDepthBuffer[4*i+1] = 0;
                _coloredDepthBuffer[4*i+2] = 255-lowerByte;
                break;
            default:
                _coloredDepthBuffer[4*i+0] = 0;
                _coloredDepthBuffer[4*i+1] = 0;
                _coloredDepthBuffer[4*i+2] = 0;
                break;
        }
    }
}

- (void)renderDepthFrame:(STDepthFrame *)depthFrame
{
    size_t cols = depthFrame->width;
    size_t rows = depthFrame->height;
    
    if (_linearizeBuffer == NULL || _normalsBuffer == NULL)
    {
        [self populateLinearizeBuffer:cols * rows];
        _coloredDepthBuffer = (uint8_t*)malloc(cols * rows * 4);
    }
    
    // Conversion of 16-bit non-linear shift depth values to 32-bit RGBA
    //
    // Adopted from: https://github.com/OpenKinect/libfreenect/blob/master/examples/glview.c
    //
    [self convertShiftToRGBA:depthFrame->data depthValuesCount:cols * rows];
    
    CGColorSpaceRef colorSpace = CGColorSpaceCreateDeviceRGB();
    
    CGBitmapInfo bitmapInfo;
    bitmapInfo = (CGBitmapInfo)kCGImageAlphaNoneSkipLast;
    bitmapInfo |= kCGBitmapByteOrder32Big;
    
    NSData *data = [NSData dataWithBytes:_coloredDepthBuffer length:cols * rows * 4];
    CGDataProviderRef provider = CGDataProviderCreateWithCFData((CFDataRef)data); //toll-free ARC bridging
    
    CGImageRef imageRef = CGImageCreate(cols,                        //width
                                       rows,                        //height
                                       8,                           //bits per component
                                       8 * 4,                       //bits per pixel
                                       cols * 4,                    //bytes per row
                                       colorSpace,                  //Quartz color space
                                       bitmapInfo,                  //Bitmap info (alpha channel?, order, etc)
                                       provider,                    //Source of data for bitmap
                                       NULL,                        //decode
                                       false,                       //pixel interpolation
                                       kCGRenderingIntentDefault);  //rendering intent
    
    // Assign CGImage to UIImage
    _depthImageView.image = [UIImage imageWithCGImage:imageRef];
    
    CGImageRelease(imageRef);
    CGDataProviderRelease(provider);
    CGColorSpaceRelease(colorSpace);
    
}

- (void) renderNormalsFrame: (STDepthFrame*) depthFrame
{
    // Convert depth units from shift to millimeters (stored as floats)
    [_floatDepthFrame updateFromDepthFrame:depthFrame];
    
    // Estimate surface normal direction from depth float values
    STNormalFrame *normalsFrame = [_normalsEstimator calculateNormalsWithProcessedFrame:_floatDepthFrame];
    
    size_t cols = normalsFrame.width;
    size_t rows = normalsFrame.height;
    
    // Convert normal unit vectors (ranging from -1 to 1) to RGB (ranging from 0 to 255)
    // Z can be slightly positive in some cases too!
    if (_normalsBuffer == NULL)
    {
        _normalsBuffer = (uint8_t*)malloc(cols * rows * 4);
    }
    for (size_t i = 0; i < cols * rows; i++)
    {
        _normalsBuffer[4*i+0] = (uint8_t)( ( ( normalsFrame.normals[i].x / 2 ) + 0.5 ) * 255);
        _normalsBuffer[4*i+1] = (uint8_t)( ( ( normalsFrame.normals[i].y / 2 ) + 0.5 ) * 255);
        _normalsBuffer[4*i+2] = (uint8_t)( ( ( normalsFrame.normals[i].z / 2 ) + 0.5 ) * 255);
    }
    
    CGColorSpaceRef colorSpace = CGColorSpaceCreateDeviceRGB();
    
    CGBitmapInfo bitmapInfo;
    bitmapInfo = (CGBitmapInfo)kCGImageAlphaNoneSkipFirst;
    bitmapInfo |= kCGBitmapByteOrder32Little;
    
    NSData *data = [NSData dataWithBytes:_normalsBuffer length:cols * rows * 4];
    CGDataProviderRef provider = CGDataProviderCreateWithCFData((CFDataRef)data);
    
    CGImageRef imageRef = CGImageCreate(cols,
                                        rows,
                                        8,
                                        8 * 4,
                                        cols * 4,
                                        colorSpace,
                                        bitmapInfo,
                                        provider,
                                        NULL,
                                        false,
                                        kCGRenderingIntentDefault);
    
    _normalsImageView.image = [[UIImage alloc] initWithCGImage:imageRef];
    
    CGImageRelease(imageRef);
    CGDataProviderRelease(provider);
    CGColorSpaceRelease(colorSpace);

}

- (void)renderColorFrame:(CMSampleBufferRef)sampleBuffer
{
    CVImageBufferRef pixelBuffer = CMSampleBufferGetImageBuffer(sampleBuffer);
    CVPixelBufferLockBaseAddress(pixelBuffer, 0);
    
    size_t cols = CVPixelBufferGetWidth(pixelBuffer);
    size_t rows = CVPixelBufferGetHeight(pixelBuffer);
    
    CGColorSpaceRef colorSpace = CGColorSpaceCreateDeviceRGB();
    
    unsigned char *ptr = (unsigned char *) CVPixelBufferGetBaseAddressOfPlane(pixelBuffer, 0);
    
    NSData *data = [[NSData alloc] initWithBytes:ptr length:rows*cols*4];
    CVPixelBufferUnlockBaseAddress(pixelBuffer, 0);
    
    CGBitmapInfo bitmapInfo;
    bitmapInfo = (CGBitmapInfo)kCGImageAlphaNoneSkipFirst;
    bitmapInfo |= kCGBitmapByteOrder32Little;
    
    CGDataProviderRef provider = CGDataProviderCreateWithCFData((CFDataRef)data);
    
    CGImageRef imageRef = CGImageCreate(cols,
                                        rows,
                                        8,
                                        8 * 4,
                                        cols*4,
                                        colorSpace,
                                        bitmapInfo,
                                        provider,
                                        NULL,
                                        false,
                                        kCGRenderingIntentDefault);
    
    _colorImageView.image = [[UIImage alloc] initWithCGImage:imageRef];
    
    CGImageRelease(imageRef);
    CGDataProviderRelease(provider);
    CGColorSpaceRelease(colorSpace);
    
}



#pragma mark -  AVFoundation

- (void)startAVCaptureSession
{
    NSString *sessionPreset = AVCaptureSessionPreset640x480;
    
    //-- Set up Capture Session.
    _session = [[AVCaptureSession alloc] init];
    [_session beginConfiguration];
    
    //-- Set preset session size.
    [_session setSessionPreset:sessionPreset];
    
    //-- Creata a video device and input from that Device.  Add the input to the capture session.
    AVCaptureDevice *videoDevice = [AVCaptureDevice defaultDeviceWithMediaType:AVMediaTypeVideo];
    if(videoDevice == nil)
        assert(0);
    
    NSError *error;
    [videoDevice lockForConfiguration:&error];
    
    // Auto-focus Auto-exposure, auto-white balance
    if ([[[UIDevice currentDevice] systemVersion] compare:@"7.0" options:NSNumericSearch] != NSOrderedAscending)
        [videoDevice setAutoFocusRangeRestriction:AVCaptureAutoFocusRangeRestrictionFar];

    [videoDevice setFocusMode:AVCaptureFocusModeContinuousAutoFocus];
    
    [videoDevice setExposureMode:AVCaptureExposureModeContinuousAutoExposure];
    [videoDevice setWhiteBalanceMode:AVCaptureWhiteBalanceModeContinuousAutoWhiteBalance];
    
    [videoDevice unlockForConfiguration];
    
    //-- Add the device to the session.
    AVCaptureDeviceInput *input = [AVCaptureDeviceInput deviceInputWithDevice:videoDevice error:&error];
    if(error)
        assert(0);
    
    [_session addInput:input]; // After this point, captureSession captureOptions are filled.
    
    //-- Create the output for the capture session.
    AVCaptureVideoDataOutput *dataOutput = [[AVCaptureVideoDataOutput alloc] init];
    
    [dataOutput setAlwaysDiscardsLateVideoFrames:YES];
    
    //-- Set to YUV420.
    [dataOutput setVideoSettings:[NSDictionary dictionaryWithObject:[NSNumber numberWithInt:kCVPixelFormatType_32BGRA]
                                                             forKey:(id)kCVPixelBufferPixelFormatTypeKey]];
    
    // Set dispatch to be on the main thread so OpenGL can do things with the data
    [dataOutput setSampleBufferDelegate:self queue:dispatch_get_main_queue()];
    
    [_session addOutput:dataOutput];

    if ([[[UIDevice currentDevice] systemVersion] compare:@"7.0" options:NSNumericSearch] != NSOrderedAscending)
    {
        [videoDevice lockForConfiguration:&error];
        [videoDevice setActiveVideoMaxFrameDuration:CMTimeMake(1, 30)];
        [videoDevice setActiveVideoMinFrameDuration:CMTimeMake(1, 30)];
        [videoDevice unlockForConfiguration];
    }
    else
    {
        AVCaptureConnection *conn = [dataOutput connectionWithMediaType:AVMediaTypeVideo];
        
        // Deprecated use is OK here because we're using the correct APIs on iOS 7 above when available
        // If we're running before iOS 7, we still really want 30 fps!
        #pragma clang diagnostic push
        #pragma clang diagnostic ignored "-Wdeprecated-declarations"
        conn.videoMinFrameDuration = CMTimeMake(1, 30);
        conn.videoMaxFrameDuration = CMTimeMake(1, 30);
        #pragma clang diagnostic pop
        
    }
    [_session commitConfiguration];
    
    [_session startRunning];
    
}


- (void)captureOutput:(AVCaptureOutput *)captureOutput didOutputSampleBuffer:(CMSampleBufferRef)sampleBuffer fromConnection:(AVCaptureConnection *)connection
{

    // Pass into the driver. The sampleBuffer will return later with a synchronized depth or IR pair.
    [_sensorController frameSyncNewColorImage:sampleBuffer];
    
    // If we weren't using framesync, we could just do the following instead:
    // [self renderColorFrame:sampleBuffer];
    
}


@end
