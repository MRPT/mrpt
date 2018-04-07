//-*- c++ -*-
//////////////////////////////////////////////////////////////////////
//                                                                  //
//   VideoDisplay - Cheap and cheeful way of creating a GL X        //
//                  display                                         //
//                                                                  //
//   Tom Drummong & Paul Smith   2002                               //
//                                                                  //
//////////////////////////////////////////////////////////////////////

#ifndef __VIDEODISPLAY_H
#define __VIDEODISPLAY_H

#include <string>
#include <assert.h>
#include <math.h>

#include <X11/Xlib.h>   //X11
#include <GL/glx.h>  // OpenGL (X extensions)
#include <GL/gl.h>   // OpenGL proper

#include <cvd/exceptions.h>
#include <cvd/image_ref.h>

namespace CVD {

	namespace Exceptions
	{
		/// %Exceptions specific to CVD::VideoDisplay
		/// @ingroup gException
		namespace VideoDisplay
		{
			/// Base class for all CVD::VideoDisplay exceptions
			/// @ingroup gException
			struct All: public CVD::Exceptions::All{};

			/// An exception occurred during initialisation
			/// @ingroup gException
			struct InitialisationError: public All
			{
				InitialisationError(std::string w); ///< Construct from error string
			};

			/// An exception occurred during run-time
			/// @ingroup gException
			struct RuntimeError: public All
			{
				RuntimeError(std::string w); ///< Construct from error string
			};
		}
	}


	/// The default GLX display request, corresponding to the minimum specification Gl display visual (8-bit colour)
	/// @relates VideoDisplay
	extern int defAttr[];

	/// A cheap and cheerful GL display window using X and the GLX library.
	/// VideoDisplay maintains for you the mapping between your choice of local 
	/// GL co-ordinates, and image co-ordinates and provides functions for
	/// converting between the two. It does not, however, manage any
	/// X events, for example resize or redraw events. To develop a
	/// more sophisticated display window, you must select which XEvents to monitor 
	/// using select_events() and then poll using pending() and use get_event()
	/// to handle events.
	/// @ingroup gGL
	class VideoDisplay
	{
		struct DoNotMapStruct
		{
			DoNotMapStruct(){}
		};

		public:
			
			static const DoNotMapStruct DoNotMap;

			/// Construct (and display) a display window
			/// @param left The local GL co-ordinate of the left of the window
			/// @param top The local GL co-ordinate of the top of the window
			/// @param right The local GL co-ordinate of the right of the window
			/// @param bottom The local GL co-ordinate of bottom of the window
			/// @param scale The number of image pixels per GL unit (default 1:1)
			/// @param visualAttr The attributes passed to glXChooseVisual
			/// @param map Whether or not to map the window
			VideoDisplay(double left, double top, double right, double bottom, double scale=1, int* visualAttr = defAttr, bool map=true);


			/// Construct (and display) a display window
			/// @param size The GL co-ordinate window goes from (0,0) to (size.x, size.y)
			/// @param map Whether or not to map the window
			/// @param visualAttr The attributes passed to glXChooseVisual
			VideoDisplay(ImageRef size, const DoNotMapStruct&,  int* visualAttr = defAttr);

			/// Construct (and display) a display window
			/// @param size The GL co-ordinate window goes from (0,0) to (size.x, size.y)
			/// @param scale The number of image pixels per GL unit (default 1:1)
			/// @param visualAttr The attributes passed to glXChooseVisual
			VideoDisplay(ImageRef size, double scale=1, int* visualAttr = defAttr);
			
			/// Destructor. This also removes the window from the display
			~VideoDisplay();

			/// Set the window title.
			/// @param s The new title
			void set_title(const std::string& s);
			
			/// Returns the connection number for this display
			int get_fd() {return ConnectionNumber(my_display);}
			/// Select which X events to handle (by default, nothing is handled)
			/// @param event_mask ORed list of XEvents to handle
			void select_events(long event_mask);
			/// Get the next X event from the queue by calling XNextEvent. 
			/// If the queue is empty, this function blocks until the next event
			/// @param event The next event
			void get_event(XEvent* event);
			/// Where is the mouse pointer in the window?
			/// @param x The x co-ordinate
			/// @param y The y co-ordinate
			void locate_display_pointer(int& x, int& y);
			/// Where is the mouse pointer in local GL co-ordinates?
			/// @param x The x co-ordinate
			/// @param y The y co-ordinate
			void locate_video_pointer(double& x, double& y);
			/// Flushes the output buffer by calling XFlush			
			void flush();
			/// How many events are waiting the queue?
			int pending();

			/// Set the desired mapping between local GL and image co-ordinates
			/// @param left The local GL co-ordinate of the left of the window
			/// @param top The local GL co-ordinate of the top of the window
			/// @param right The local GL co-ordinate of the right of the window
			/// @param bottom The local GL co-ordinate of bottom of the window
			/// @param scale The number if image pixels per GL unit
			void set_zoom(double left, double top, double right, double bottom, double scale);
			/// Change the image mapping to zoom in around the specified point
			/// @param cx The x co-ordinate of the zoom centre
			/// @param cy The y co-ordinate of the zoom centre
			/// @param factor The zoom factor (default 2)
			void zoom_in(double cx, double cy, double factor = 2);
			/// Change the image mapping to zoom out around the specified point
			/// @param cx The x co-ordinate of the zoom centre
			/// @param cy The y co-ordinate of the zoom centre
			/// @param factor The zoom factor (default 2)
			void zoom_out(double cx, double cy, double factor = 2)
			{assert(factor != 0.0); zoom_in(cx, cy, 1/factor);}
			/// Reset the image mapping to that when the VideoDisplay was created
			void zoom_reset()
			{set_zoom(my_orig_left, my_orig_top, my_orig_right, my_orig_bottom, my_orig_scale);}

			/// What is the width of the local GL co-ordinates displayed?
			double video_width() const {return fabs(my_right - my_left);}
			/// What is the height of the local GL co-ordinates displayed?
			double video_height() const {return fabs(my_bottom - my_top);}

			/// Convert image co-ordinates into local GL co-ordinates
			/// @param dx The image x co-ordinate to convert
			/// @param dy The image y co-ordinate to convert
			/// @param vx The returned local GL x co-ordinate
			/// @param vy The returned local GL y co-ordinate
			void display_to_video(int dx, int dy, double& vx, double& vy);
			/// Convert local GL co-ordinates into image co-ordinates
			/// @param dx The local GL x co-ordinate to convert
			/// @param dy The local GL y co-ordinate to convert
			/// @param vx The returned image x co-ordinate
			/// @param vy The returned image y co-ordinate
			void video_to_display(double dx, double dy, int& vx, int & vy);

			/// Make this window the current GL context. If there is more than one
			/// GL window being used in your program, this function should be called 
			/// before drawing to ensure you draw to the correct window.
			void make_current();

      /// Swap the front and back GL buffers. This calls glXSwapBuffers for the
      /// current context.
      void swap_buffers();
      /// What is my display?
      Display* display() {return my_display;}
			/// Which is my window?
			Window window() {return my_window;}

		private:
			double my_left;
			double my_top;
			double my_right;
			double my_bottom;
			bool my_positive_right;
			bool my_positive_down;
			double my_scale;
			double my_orig_left;
			double my_orig_top;
			double my_orig_right;
			double my_orig_bottom;
			double my_orig_scale;
			Display *my_display;
			XVisualInfo* my_visual;
			GLXContext my_glx_context;
			Colormap my_cmap;
			Window my_window;

			VideoDisplay( VideoDisplay& copyof );
			int operator = ( VideoDisplay& copyof );


			void init(double, double, double, double, double, int* visualAttr, bool);
	};
   
} // CVD

#endif
