//////////////////////////////////////////////////////////////////////
//                                                                  //
//   VideoDisplay - Cheap and cheeful way of creating a GL X        //
//                  display                                         //
//                                                                  //
//   Tom Drummong & Paul Smith   2002                               //
//                                                                  //
//////////////////////////////////////////////////////////////////////

#include "cvd/videodisplay.h"

//#include <cmath>
//#include <cstdlib>
//#include <iostream>
//#include <cassert>

#include <iostream>
#include <math.h>
#include <stdlib.h>
#include <assert.h>

using namespace std;
using namespace CVD;

CVD::Exceptions::VideoDisplay::InitialisationError::InitialisationError(string w)
{
	what="VideoDisplay inintialisation error: " + w;
}

CVD::Exceptions::VideoDisplay::RuntimeError::RuntimeError(string w)
{
	what="VideoDisplay error: " + w;
}

//namespace CVD {
int CVD::defAttr[] = {GLX_RGBA,
		      GLX_RED_SIZE, 3, 
		      GLX_GREEN_SIZE, 3,
		      GLX_BLUE_SIZE, 2,
			  GLX_DOUBLEBUFFER, 0,
		      // GLX_DEPTH_SIZE, 8,
		      // GLX_STENCIL_SIZE, 8,
		      0};





CVD::VideoDisplay::VideoDisplay(double left, double top, double right, double bottom, double scale, int* visualAttr, bool map) :
   my_left(left),
   my_top(top),
   my_right(right),
   my_bottom(bottom),
   my_scale(scale),
   my_orig_left(left),
   my_orig_top(top),
   my_orig_right(right),
   my_orig_bottom(bottom),
   my_orig_scale(scale)
{
	init(my_left, my_top, my_right, my_bottom, scale, visualAttr, map);
}


CVD::VideoDisplay::VideoDisplay(ImageRef s, double scale, int* visualAttr) :
   my_left(0),
   my_top(0),
   my_right(s.x),
   my_bottom(s.y),
   my_scale(scale),
   my_orig_left(0),
   my_orig_top(0),
   my_orig_right(s.x),
   my_orig_bottom(s.y),
   my_orig_scale(scale)
{
	init(my_left, my_top, my_right, my_bottom, scale, visualAttr, true);
}


CVD::VideoDisplay::VideoDisplay(ImageRef s, const DoNotMapStruct&, int* visualAttr) :
   my_left(0),
   my_top(0),
   my_right(s.x),
   my_bottom(s.y),
   my_scale(1.0),
   my_orig_left(0),
   my_orig_top(0),
   my_orig_right(s.x),
   my_orig_bottom(s.y),
   my_orig_scale(1.0)
{
	init(my_left, my_top, my_right, my_bottom, 1.0, visualAttr, false);
}



void CVD::VideoDisplay::init(double left, double top, double right, double bottom, double scale, int* visualAttr, bool map)
{
   // Need these for converting mouse clicks efficiently
   // (This stays the same even when zooming)
   my_positive_right = right > left;
   my_positive_down = bottom > top;


  my_display = XOpenDisplay(NULL);
  if (my_display == NULL)
  	throw Exceptions::VideoDisplay::InitialisationError(string("Can't connect to display ") + getenv("DISPLAY"));


  // get an appropriate visual
  my_visual = glXChooseVisual(my_display, DefaultScreen(my_display), visualAttr);
  if (my_visual == NULL)
  	throw Exceptions::VideoDisplay::InitialisationError(string("No matching visual on display ") + getenv("DISPLAY"));
  
  // check I got an rgba visual (32 bit)
  int isRgba;
  (void) glXGetConfig(my_display, my_visual, GLX_RGBA, &isRgba);
  assert(isRgba);

  // create a GLX context
  if ((my_glx_context = glXCreateContext(my_display, my_visual, 0, GL_TRUE)) == NULL)
  	throw Exceptions::VideoDisplay::InitialisationError(string("Cannot create GL context on ") + getenv("DISPLAY"));
  
  // create a colormap (it's empty for rgba visuals)
  my_cmap = XCreateColormap(my_display,
			    RootWindow(my_display, my_visual->screen),
			    my_visual->visual,
			    AllocNone);

  // set the attributes for the window
  XSetWindowAttributes swa;
  swa.colormap = my_cmap;
  swa.border_pixel = 0;
  swa.event_mask = StructureNotifyMask;

   // Work out how big the display is
   int xsize = static_cast<int>(fabs(right - left) * scale);
   int ysize = static_cast<int>(fabs(bottom - top) * scale);
   // create a window
   my_window = XCreateWindow(my_display,
			    RootWindow(my_display, my_visual->screen),
			    0, // init_x
			    0, // init_y
			    xsize,
			    ysize,
			    0, 
			    my_visual->depth,
			    InputOutput,
			    my_visual->visual,
			    CWBorderPixel | CWColormap | CWEventMask, &swa);

  if (my_window == 0)
  	throw Exceptions::VideoDisplay::InitialisationError(string("Cannot create a window on ") + getenv("DISPLAY"));

  set_title("Video Display");

  
  if(map)
  {
	  XMapWindow(my_display, my_window);

	  // discard all event up to the MapNotify
	  XEvent ev;
	  do
	  {
		 XNextEvent(my_display,&ev);
	  }
	  while (ev.type != MapNotify);
  }

  // Connect the GLX context to the window
  if (!glXMakeCurrent(my_display, my_window, my_glx_context))
  	throw Exceptions::VideoDisplay::InitialisationError("Cannot make window current to GL context");
  
  XSelectInput(my_display,
	       my_window,
	       ButtonPressMask |
	       ButtonReleaseMask     |
	       Button1MotionMask     |
	       PointerMotionHintMask | 
	       KeyPressMask);

  
  // Sort out the GL scaling
  glLoadIdentity(); 
  glViewport(0, 0, xsize, ysize);

  glColor3f(1.0, 1.0, 1.0);

  glDrawBuffer(GL_FRONT);      // On Vino, we only use the front buffer 

  set_zoom(left, top, right, bottom, scale);
}

void CVD::VideoDisplay::set_zoom(double left, double top, double right, double bottom, double scale)
{
   // Make sure that this is the current window
   make_current();
   
   my_left = left;
   my_top = top;
   my_right = right;
   my_bottom = bottom;
   my_scale = scale;


   glMatrixMode(GL_MODELVIEW);
   glLoadIdentity();


   glMatrixMode(GL_PROJECTION);

   glLoadIdentity();
  glOrtho(left, right, bottom, top, -1 , 1);    // If the origin is the top left 

  glRasterPos2f(left, top);
  
  // video is now the same way as upside down to graphics!
  glPixelZoom(scale, -scale);
  //glPixelZoom(1,-1);

}

void CVD::VideoDisplay::zoom_in(double cx, double cy, double factor)
{
   double width = my_right - my_left;
   double height = my_bottom - my_top;

   //Work out the new half-widths
   width = width / 2 / factor;
   height = height / 2 / factor;

   // Work out the new image corners. scale is how much things are zoomed up, so
   // this is just multipled by the factor
   set_zoom(cx - width,  cy - height, cx + width, cy + height, my_scale * factor);
}

CVD::VideoDisplay::~VideoDisplay()
{
  glXMakeCurrent(my_display, None, NULL);
  glXDestroyContext(my_display, my_glx_context);
  
  XUnmapWindow(my_display, my_window);
  XDestroyWindow(my_display, my_window);
  XFlush(my_display);
  XCloseDisplay(my_display);
}

void CVD::VideoDisplay::set_title(const string& s)
{ 
   // give the window a name
   XStoreName(my_display, my_window, s.c_str());
}

void CVD::VideoDisplay::select_events(long event_mask){
  XSelectInput(my_display, my_window, event_mask);
  XFlush(my_display);
}

void CVD::VideoDisplay::flush(){
  XFlush(my_display);
}

void CVD::VideoDisplay::get_event(XEvent* event){
  XNextEvent(my_display, event);
}

int CVD::VideoDisplay::pending()
{
  return XPending(my_display);
}

void CVD::VideoDisplay::make_current()
{
   // Connect the GLX context to the window
	if (!glXMakeCurrent(my_display, my_window, my_glx_context))
		throw Exceptions::VideoDisplay::InitialisationError("Cannot make window current to GL context");
}

//
// DISPLAY TO VIDEO
// Converts a point on the display to the video's coordinates
//
void CVD::VideoDisplay::display_to_video(int dx, int dy, double& vx, double& vy)
{
   vx = dx / my_scale;
   if(my_positive_right)
      vx = my_left + vx;
   else
      vx = my_left - vx;

   vy = dy / my_scale;
   if(my_positive_down)
      vy = my_top + vy;
   else
      vy = my_top - vy;
}

//
// VIDEO TO DISPLAY
// Converts a point on the display to the video's coordinates
//
void CVD::VideoDisplay::video_to_display(double vx, double vy, int& dx, int& dy)
{
   double ix, iy;
   if(my_positive_right)
      ix = vx - my_left;
   else
      ix = my_left - vx;
   ix = ix * my_scale;

   if(my_positive_down)
      iy = vy - my_top;
   else
      iy = my_top - vy;
   iy = iy * my_scale;

   dx = static_cast<int>(ix);
   dy = static_cast<int>(iy);
}

//
// LOCATE DISPLAY POINTER
// Where is the pointer in the window?
//
void CVD::VideoDisplay::locate_display_pointer(int& x, int& y)
{
    Window root_return;
    Window child_return;
    int root_x_return;
    int root_y_return;
    unsigned int mask_return;

    XQueryPointer(my_display,
		  my_window,
		  &root_return,
		  &child_return,
		  &root_x_return,
		  &root_y_return,
		  &x,
		  &y,
		  &mask_return);
}

//
// LOCATE DISPLAY POINTER
// Where is the pointer in the video frame?
//
void CVD::VideoDisplay::locate_video_pointer(double& x, double& y)
{
   int dx, dy;
   locate_display_pointer(dx, dy);
   display_to_video(dx, dy, x, y);
}

void CVD::VideoDisplay::swap_buffers()
{
  glXSwapBuffers(my_display, glXGetCurrentDrawable());
}

//} // namespace CVD



const VideoDisplay::DoNotMapStruct VideoDisplay::DoNotMap;




