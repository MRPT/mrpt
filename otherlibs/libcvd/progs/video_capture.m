#import <QTKit/QTKit.h>
#include "video_capture.h"

#define DebugLog if (0) NSLog

// -*- mode: objc -*-

@interface VideoCapture: NSObject 
{
  QTCaptureSession*                 captureSession;
  NSMutableArray*                   videoDevices;
  QTCaptureDeviceInput*             deviceInput;
  QTCaptureDecompressedVideoOutput* videoOutput;

  vcapture_frame_callback callback;
  void* userdata;
  
  struct vcapture_frame_format requested;
  struct vcapture_frame_format actual;
  BOOL needActual;

}

- (NSArray*) devices;
- (int) device;
- (int) setDevice: (int)idx;

- (BOOL) active;
- (int) start;
- (int) stop;

- (vcapture_frame_callback) callback;
- (void) setCallback: (vcapture_frame_callback)callback withUserData:(void*)userdata;

- (void) getRequestedFormat:(struct vcapture_frame_format*)format;
- (int)  setRequestedFormat:(const struct vcapture_frame_format*)format;

- (void) captureOutput:(QTCaptureOutput *)captureOutput 
         didOutputVideoFrame:(CVImageBufferRef)videoFrame 
         withSampleBuffer:(QTSampleBuffer *)sampleBuffer 
         fromConnection:(QTCaptureConnection *)connection;

@end

@implementation VideoCapture 

- (id) init {
  if ( (self = [super init]) ) {

    NSAutoreleasePool* pool = [[NSAutoreleasePool alloc] init];

    captureSession = [[QTCaptureSession alloc] init];
    videoDevices = [[NSMutableArray alloc] init];
    deviceInput = nil;
    videoOutput = nil;
    callback = 0;
    userdata = 0;

    memset(&requested, 0, sizeof(struct vcapture_frame_format));
    memset(&actual, 0, sizeof(struct vcapture_frame_format));

    requested.type = VC_TYPE_RGB;
    needActual = YES;

    NSArray* allInputDevices = [QTCaptureDevice inputDevices];

    for (QTCaptureDevice* device in allInputDevices) {
      DebugLog(@"device: %@", device);
      if ([device hasMediaType:QTMediaTypeVideo] ||
          [device hasMediaType:QTMediaTypeMuxed]) {
        DebugLog(@"  it is video!");
        [videoDevices addObject:device];
      }
    }

    [pool release];

  }
  return self;
}

- (void) resetDevice {

  if ([self active]) {
    [self stop];
  }

  if (videoOutput) {
    [captureSession removeOutput:videoOutput];
    [videoOutput release];
    videoOutput = nil;
  }

  if (deviceInput) {
    [captureSession removeInput:deviceInput];
    QTCaptureDevice* device = [deviceInput device];
    if ([device isOpen]) {
      [device close];
    }
    [deviceInput release];
    deviceInput = nil;
  }

}

- (void) dealloc {
  DebugLog(@"deallocating a %@", [self className]);
  if (deviceInput) {
    if ([self active]) {
      DebugLog(@"stopping device in dealloc");
    }
    DebugLog(@"resetting device in dealloc");
  }
  [self resetDevice];
  [videoDevices release];
  [captureSession release];
  [super dealloc];
}

- (int) device {
  if (deviceInput == nil) {
    return -1;
  } else {
    return [videoDevices indexOfObject:[deviceInput device]];
  }
}

- (int) setDevice: (int)idx {

  if ([self device] == idx) {
    return VC_SUCCESS;
  }

  [self resetDevice];

  if (idx < 0) { return VC_SUCCESS; }
  if (idx >= [videoDevices count]) { return VC_INVALID_DEVICE; }


  QTCaptureDevice* device = [videoDevices objectAtIndex:idx];
  NSError* error = nil;

  BOOL success = [device open:&error];

  if (!success) {
    NSLog(@"Error opening device: %@", error);
    return VC_DEVICE_OPEN_ERROR;
  }

  deviceInput = [[QTCaptureDeviceInput alloc] initWithDevice: device];

  success = [captureSession addInput:deviceInput error:&error];
  if (!success) {
    NSLog(@"Error adding input: %@", error);
    [self resetDevice];
    return VC_DEVICE_OPEN_ERROR;
  }

  videoOutput = [[QTCaptureDecompressedVideoOutput alloc] init];
  success = [captureSession addOutput:videoOutput error:&error];
  if (!success) {
    NSLog(@"Error adding output: %@", error);
    [self resetDevice];
    return VC_DEVICE_OPEN_ERROR;
  }

  [videoOutput setDelegate:self];
  [videoOutput setAutomaticallyDropsLateVideoFrames:YES];

  needActual = YES;


  NSMutableDictionary* pixelBufferAttributes = [[NSMutableDictionary alloc] init];
  NSString* key;
  NSNumber* val;

  
  if (requested.fourcc || requested.type) {

    unsigned int type;
    if (requested.fourcc) {
      type = requested.fourcc;
    } else {
      type = vcapture_type_to_fourcc(requested.type);
    }

    key = (NSString*)kCVPixelBufferPixelFormatTypeKey;
    val = [NSNumber numberWithUnsignedInt:type];

    DebugLog(@"set pixelBufferAttributes %@->%@", key, val);

    [pixelBufferAttributes setObject:val forKey:key];

  }

  if (requested.width && requested.height) {

    key = (NSString*)kCVPixelBufferWidthKey;
    val = [NSNumber numberWithInt:requested.width];
    [pixelBufferAttributes setObject:val forKey:key];
    DebugLog(@"set pixelBufferAttributes %@->%@", key, val);

    key = (NSString*)kCVPixelBufferHeightKey;
    val = [NSNumber numberWithInt:requested.height];
    [pixelBufferAttributes setObject:val forKey:key];
    DebugLog(@"set pixelBufferAttributes %@->%@", key, val);

  }

  if ([pixelBufferAttributes count]) {
    [videoOutput setPixelBufferAttributes:pixelBufferAttributes];
  } else {
    [pixelBufferAttributes release];
  }
  
  return VC_SUCCESS;

}

- (NSArray*) devices {
  return videoDevices;
}

- (vcapture_frame_callback) callback {
  return callback;
}

- (void)captureOutput:(QTCaptureOutput *)captureOutput 
  didOutputVideoFrame:(CVImageBufferRef)videoFrame 
  withSampleBuffer:(QTSampleBuffer *)sampleBuffer 
  fromConnection:(QTCaptureConnection *)connection 

{

  [sampleBuffer incrementSampleUseCount];

  CVReturn status = CVPixelBufferLockBaseAddress(videoFrame, 0);

  if (status == 0) { 

    if (needActual) {

      needActual = NO;

      actual.fourcc = CVPixelBufferGetPixelFormatType(videoFrame);
      actual.type = vcapture_fourcc_to_type(actual.fourcc);
      actual.width = CVPixelBufferGetWidth(videoFrame);
      actual.height = CVPixelBufferGetHeight(videoFrame);

      size_t l, r, t, b;

      CVPixelBufferGetExtendedPixels(videoFrame, &l, &r, &t, &b);

      actual.pad_left = l;
      actual.pad_right = r;
      actual.pad_top = t;
      actual.pad_bottom = b;

      actual.bytes_per_row = CVPixelBufferGetBytesPerRow(videoFrame);
    
    }

    struct vcapture_frame frame;

    memset(frame.planes, 0, sizeof(frame.planes));

    unsigned long long timestamp = 0;


    if (CVPixelBufferIsPlanar(videoFrame)) {
      frame.count = CVPixelBufferGetPlaneCount(videoFrame);
      if (frame.count > 4) { frame.count = 4; }
      int i;
      for (i=0; i<frame.count; ++i) {
        frame.planes[i] = CVPixelBufferGetBaseAddressOfPlane(videoFrame, i);
      }
    } else {
      frame.count = 0;
      frame.planes[0] = CVPixelBufferGetBaseAddress(videoFrame);    
    }
  
    callback(&actual, &frame, timestamp, userdata);
  
    CVPixelBufferUnlockBaseAddress(videoFrame, 0);

  }

  [sampleBuffer decrementSampleUseCount];

}

- (BOOL) active {
  return [captureSession isRunning];
}

- (int) start {
  if ([self device] < 0) { return VC_NO_ACTIVE_DEVICE; }
  [captureSession startRunning];
  return VC_SUCCESS;
}

- (int) stop {
  if ([captureSession isRunning]) {
    [captureSession stopRunning];
  }
  return VC_SUCCESS;
}

- (void) setCallback: (vcapture_frame_callback)c withUserData:(void*)u {
  callback = c;
  userdata = u;
}

- (void) getRequestedFormat:(struct vcapture_frame_format*)format {
  memcpy(format, &requested, sizeof(struct vcapture_frame_format));
}

- (int) setRequestedFormat:(const struct vcapture_frame_format*)format {
  if ([self active]) {
    return VC_DEVICE_RUNNING;
  } 
  int oldDevice = [self device];
  [self resetDevice];
  memcpy(&requested, format, sizeof(struct vcapture_frame_format));
  return [self setDevice:oldDevice];
}

@end

//////////////////////////////////////////////////////////////////////

unsigned int vcapture_type_to_fourcc(enum vcapture_format_type t) {
  switch (t) {
  case VC_TYPE_RGB: return k24RGBPixelFormat;
  case VC_TYPE_RGBA: return kCVPixelFormatType_32RGBA;
  case VC_TYPE_ARGB: return kCVPixelFormatType_32ARGB;
  case VC_TYPE_FOURCC_2uvy: return '2uvy';
  case VC_TYPE_FOURCC_yuvs: return 'yuvs';
  case VC_TYPE_FOURCC_YVYU: return 'YVYU';
  case VC_TYPE_FOURCC_yuvu: return 'yuvu';
  default:
    break;
  }
  return 0;
}

enum vcapture_format_type vcapture_fourcc_to_type(unsigned int fourcc) {
  switch (fourcc) {
  case k24RGBPixelFormat: return VC_TYPE_RGB;
  case kCVPixelFormatType_32ARGB: return VC_TYPE_ARGB;
  case kCVPixelFormatType_32RGBA: return VC_TYPE_RGBA;
  case '2uvy': return VC_TYPE_FOURCC_2uvy;
  case 'yuvs': return VC_TYPE_FOURCC_yuvs;
  case 'YVYU': return VC_TYPE_FOURCC_YVYU;
  case 'yuvu': return VC_TYPE_FOURCC_yuvu;
  default:
    break;
  }
  return VC_TYPE_UNKNOWN;
}

//////////////////////////////////////////////////////////////////////

void vcapture_poll(int msec) {
  NSTimeInterval interval = msec / 1000.0;
  NSDate* date = [[NSDate alloc] initWithTimeIntervalSinceNow:interval];
  [[NSRunLoop currentRunLoop]  runMode:NSDefaultRunLoopMode beforeDate:date];
  [date release];
}

//////////////////////////////////////////////////////////////////////

struct video_capture {
  NSAutoreleasePool* pool;
  VideoCapture* instance;
};

//////////////////////////////////////////////////////////////////////

vcapture vcapture_alloc() {
  vcapture rval = (vcapture)malloc(sizeof(struct video_capture));
  if (!rval) { return 0; }
  rval->pool = [[NSAutoreleasePool alloc] init];
  rval->instance = [[VideoCapture alloc] init];
  if (!rval->instance) {
    vcapture_free(rval);
    rval = 0;
  }
  return rval;
}

void vcapture_free(vcapture v) {
  [v->instance release];
  [v->pool release];
  free(v);
}

//////////////////////////////////////////////////////////////////////

int vcapture_device_count(vcapture v) {
  return [[v->instance devices] count];
}

const char* vcapture_device_name(vcapture v, int idx) {

  NSArray* devices = [v->instance devices];
  if (idx >= [devices count]) {
    return 0;
  }

  QTCaptureDevice* device = [devices objectAtIndex:idx];
  NSString* str = [device description];

  if (!str) { return 0; }

  return [str cStringUsingEncoding:NSUTF8StringEncoding];

}

//////////////////////////////////////////////////////////////////////

int vcapture_get_device(vcapture v) {
  return [v->instance device];
}

int vcapture_set_device(vcapture v, int device) {
  return [v->instance setDevice:device];
}

//////////////////////////////////////////////////////////////////////

int vcapture_get_requested_format(vcapture v, struct vcapture_frame_format* f) {
  [v->instance getRequestedFormat:f];
  return VC_SUCCESS;
}

int vcapture_set_requested_format(vcapture v, const struct vcapture_frame_format* f) {
  return [v->instance setRequestedFormat:f];
}

//////////////////////////////////////////////////////////////////////

int vcapture_set_callback(vcapture v,
                          vcapture_frame_callback callback,
                          void* userdata) {
  [v->instance setCallback:callback withUserData:userdata];
  return VC_SUCCESS;
}
 
vcapture_frame_callback vcapture_get_callback(vcapture v) {
  return [v->instance callback];
}

//////////////////////////////////////////////////////////////////////

int vcapture_active(vcapture v) {
  return [v->instance active];
}

int vcapture_start(vcapture v) {
  return [v->instance start];
}

int vcapture_stop(vcapture v) {
  return [v->instance stop];
}
