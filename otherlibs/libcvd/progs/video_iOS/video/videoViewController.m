//
//  videoViewController.m
//  video
//
//  Created by Gerhard Reitmayr on 07/04/2011.
//  Copyright 2011 TU Graz. All rights reserved.
//

#import <QuartzCore/QuartzCore.h>

#import "videoViewController.h"
#import "EAGLView.h"

@interface videoViewController ()
@property (nonatomic, retain) EAGLContext *context;
@end

@implementation videoViewController

@synthesize context;

- (void)awakeFromNib
{
    EAGLContext *aContext = [[EAGLContext alloc] initWithAPI:kEAGLRenderingAPIOpenGLES1];
        
    if (!aContext)
        NSLog(@"Failed to create ES context");
    else if (![EAGLContext setCurrentContext:aContext])
        NSLog(@"Failed to set ES context current");
    
	self.context = aContext;
	[aContext release];
	
    [(EAGLView *)self.view setContext:context];
    [(EAGLView *)self.view setFramebuffer];

    glGenTextures(1, &videoFrameTexture);
    glBindTexture(GL_TEXTURE_2D, videoFrameTexture);
   	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	// This is necessary for non-power-of-two textures
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

    camera = [[Camera alloc] init];
	camera.delegate = self;    
}

- (void)dealloc
{    
    // Tear down context.
    if ([EAGLContext currentContext] == context)
        [EAGLContext setCurrentContext:nil];
    
    [context release];
    [camera release];
    [super dealloc];
}

- (void)didReceiveMemoryWarning
{
    // Releases the view if it doesn't have a superview.
    [super didReceiveMemoryWarning];
    
    // Release any cached data, images, etc. that aren't in use.
}

- (void)viewDidUnload
{
	[super viewDidUnload];

    // Tear down context.
    if ([EAGLContext currentContext] == context)
        [EAGLContext setCurrentContext:nil];
	self.context = nil;	
}

- (void)drawFrame
{
    [(EAGLView *)self.view setFramebuffer];
    
    // Replace the implementation of this method to do your own custom drawing.
    // Replace the implementation of this method to do your own custom drawing.
    static const GLfloat squareVertices[] = {
        0, 0,
        320,0,
        0, 240,
        320, 240
    };
    
	static const GLfloat textureVertices[] = {
        0, 0,
        1, 0,
        0, 1,
        1, 1
    };
    
    glClear(GL_COLOR_BUFFER_BIT);
    
    glMatrixMode(GL_PROJECTION);
    static const GLfloat proj[] = {
        0, -1.f/180, 0, 0,
        -1.f/120, 0, 0, 0,
        0, 0, 1, 0,
        1, 1, 0, 1
    };
    glLoadMatrixf(proj);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    
    glEnable(GL_TEXTURE_2D);
    glBindTexture(GL_TEXTURE_2D, videoFrameTexture);
    // Update attribute values.
    glVertexPointer(2, GL_FLOAT, 0, squareVertices);
    glEnableClientState(GL_VERTEX_ARRAY);
    glTexCoordPointer(2, GL_FLOAT, 0, textureVertices);
    glEnableClientState(GL_TEXTURE_COORD_ARRAY);
    
    glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
    
    glDisableClientState(GL_VERTEX_ARRAY);
    glDisableClientState(GL_TEXTURE_COORD_ARRAY);
    glDisable(GL_TEXTURE_2D);
    
    [(EAGLView *)self.view presentFramebuffer];
}

- (void)processNewCameraFrame:(CVImageBufferRef)cameraFrame;
{
	CVPixelBufferLockBaseAddress(cameraFrame, 0);
	int bufferHeight = CVPixelBufferGetHeight(cameraFrame);
	int bufferWidth = CVPixelBufferGetWidth(cameraFrame);
    
	// Using BGRA extension to pull in video frame data directly
	glBindTexture(GL_TEXTURE_2D, videoFrameTexture);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, bufferWidth, bufferHeight, 0, GL_BGRA, GL_UNSIGNED_BYTE, CVPixelBufferGetBaseAddress(cameraFrame));
    
    // do some vision stuff !
    // release image data
    CVPixelBufferUnlockBaseAddress(cameraFrame, 0);
    // render video texture
	[self drawFrame];
}


@end
