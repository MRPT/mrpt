#import <Foundation/Foundation.h>
#import <AVFoundation/AVFoundation.h>
#import <CoreMedia/CoreMedia.h>

@protocol CameraDelegate;

@interface Camera : NSObject <AVCaptureVideoDataOutputSampleBufferDelegate>
{
	AVCaptureSession *captureSession;
	AVCaptureDeviceInput *videoInput;
	AVCaptureVideoDataOutput *videoOutput;
}

@property(nonatomic, assign) id<CameraDelegate> delegate;

@end

@protocol CameraDelegate
- (void)processNewCameraFrame:(CVImageBufferRef)cameraFrame;
@end
