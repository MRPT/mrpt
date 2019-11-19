/*                       
	This file is part of the CVD Library.

	Copyright (C) 2005 The Authors

	This library is free software; you can redistribute it and/or
	modify it under the terms of the GNU Lesser General Public
	License as published by the Free Software Foundation; either
	version 2.1 of the License, or (at your option) any later version.

	This library is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
	Lesser General Public License for more details.

	You should have received a copy of the GNU Lesser General Public
	License along with this library; if not, write to the Free Software
	Foundation, Inc., 
    51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*/

/**
\page tutorial1 Tutorial

\section images Images

Images are represented using the templated CVD#Image type. The template 
parameter determines the number of channels (i.e. whether the image is greyscale, 
RGB, etc) and precision of the image (8 bit, floating point, etc).

\subsection loadsave1 Loading and saving images from file.

@code
#include <cvd/image_io.h>
using namespace CVD;

int main()
{
  Image<byte> in; 					//Declares an 8 bit greyscale image

  in = img_load("test_image.jpg");			
  img_save(in, "new_image.png");
  
  return 0;
}

@endcode

CVD#img_load can be given either a filename or an istream. In this case, the image
will be converted in to an 8 bit geryscale image automatically to match the type of
<code>in</code>.

CVD#img_save can be given a filename or an ostream to save to. It can also be given an 
image type. In the absence of a type, it will use the extension of the filename to 
deduce the image type. It will choose the optimum number of channels and bits per channel
based on the type of <code>in</code> and the availability of features in the image
file format being used. Consequently, the resulting image in <code>new_image.png</code> 
will be 8 bit greyscale.


@code
#include <cvd/image_io.h>
using namespace CVD;

int main()
{
  Image<Rgb<float> > in; 					//Declares a floating rgb image

  in = img_load("test_image.jpg");			
  img_save(in, "new_image.png");
  
  return 0;
}
@endcode

This example loads and saves a file via a floating point RGB image. In this
case, the file saved will be an RGB PNG image with 16 bits per channel, since
that is the best available match to  <code>Image<Rgb<byte> > </code> Images of
any builtin type can be made, loaded in to and saved from as gery scale images.
RGB images can be used via CVD#Rgb and RGBA images via CVD#Rgba. 


\subsection imagedisplay Basic image visualisation
@code
#include <cvd/image_io.h>
#include <cvd/videodisplay.h>               //Very cheap and cheerful X window with OpenGL capabilities
#include <cvd/gl_helpers.h>                 //OpenGL wrappers for various CVD types
using namespace CVD;

int main()
{
  Image<Rgb<byte> > in; 					
  in = img_load("test_image.jpg");			

  VideoDisplay window(in.size());			//Create an OpenGL window with the dimensions of `in'
  glDrawPixels(in);

  glColor3f(1,0,0);
  glBegin(GL_LINES);
  glVertex(in.size()/2);
  glVertex(in.size());
  glEnd();
  glFlush();
  
  std::cin.get();
  return 0;
}
@endcode

This program loads and displays an image, then draws a red diagonal line from the center to the bottom
right corner of the image. The OpenGL coordinate system is set up so that (0,0) corresponds to pixel
(0,0) in the image. Note that OpenGL wrappers exist for CVD's size objects (known as CVD#ImageRef). 


\subsection imageref Pixel Access

Images can be accessed in two ways, either by row, then column or with an CVD#ImageRef object. 

@code
#include <cvd/image_io.h>
using namespace CVD;
using namespace std;

int main()
{
  Image<Rgb<byte> > in; 					

  in = img_load("test_image.jpg");		

  ImageRef pos;
  
  pos.x = 0;
  pos.y = in.size().y / 2;
 	
  cout << "Pixel at x=10, y=3 = " << in[3][10] << endl;
  cout << "Pixel at x=0, and half way down = " << in[pos] << endl;
  
  return 0;
}
@endcode


\subsection image2 Images in a bit more detail

Images behave like smart pointers. Copying images using = or copy construction
copies a pointer, but still references the same underlying data. Images are
reference counted, so when the last reference to an image is destructed, then the data is
deleted. This allows images to be efficiently returned from functions and stored in STL
containers, since copying is fast.

Making duplicate copies of the data must be done explicitly:

@code
	Image<byte> a, b;
	...
	...
	//fill in a with somethng
	...
	...

	//Duplicate a's data
	b.copy_from(a);

@endcode

\subsubsection helpers Useful Methods

There are a number of useful helper methods for images:
- Image(ImageRef)
	- Construct an image of a given size
- Image(ImageRef, T)
	- Construct an image of a given size and filled with a certain value
- resize(ImageRef)
	- This resizes an image.
	- The image now points to a new block of uninitialized data
	- All other copies of the data are unaffected
- zero()
    - Fills the memory with all zeros
	- Only use this on POD types
- fill(T)
	- Fills the image with a specific value
- make_unique()
    - Causes this image to point to a fresh copy of the data.
	- All other references to the data are unaffected
- copy_from_me()
	- This causes a copy to occur on assignment.
	- This is complementary to copy_from.
	- a = b.copy_from_me();


\subsubsection imstl Images and the STL
Images can be efficientlu stored in STL containers.
If you want to make a bag of 10 images, then 
the code
@code
vector<Image<byte> > some_images(an_image, 13);
@endcode
Will not make 13 copies of an_image images, it will make 13 references to the same 
block of data, since copying images does not copy the data. The following code can be
used instead:
@code
vector<Image<byte> > some_images(CreateImagesBegin(an_image), CreateImagesEnd(an_image, 13));
@endcode




\section video1 Video

Basic video access and display is demonstrated by the following code:
@code
#include <cvd/videosource.h>
#include <cvd/videodisplay.h>
#include <cvd/gl_helpers.h>

using namespace CVD;

int main()
{
	VideoBuffer<Rgb<byte> > * video_buffer = open_video_source<Rgb<byte> >("v4l2:///dev/video0");
	VideoDisplay disp(video_buffer->size());

	while(1)
	{
		VideoFrame<Rgb<byte> > *frame = video_buffer->get_frame();
		glDrawPixels(*frame);
		video_buffer->put_frame(frame);
	}
}
@endcode

Video is obtained from CVD#VideoBuffer's, which are templated on the type of the frame (like CVD#Image).
CVD#open_video_source is the most convinient way to open a video, and uses a URL like syntax. For example:
- v4l2://v4l2 device file
	- This opens a Video4Linux2 device
	- Example: v4l2:///dev/video0
- dc1394://camera number 
	- Opens a camera obeying the IEEE1394, DC spec (eg firewire webcams, and high end firewire video cameras).
	- The DC1394 camera number selectd the device
	- Example: dc1394://0
- files://glob
	- Opens a list of images given by a unix style glob
	- glob is relative to the current directory
	- Example files://img*.jpg 
- file://file
	- Opens AVI or MPEG files (or anything else compatible with ffmpeg)
	- Example file://video.mpg

CVD#open_video_source is a convinience function. More flexibility can be gained by using the underlying CVD#VideoBuffer classes
such as CVD#DVBuffer2.


VideoBuffers return CVD#VideoFrame objects, instead of images. These share much of the same functionality
with images, but they do not perform reference counting. However, they do have features specific to video, such
as a timestamp.  Images and VideoFrames actually share the same base class.

After use, video frames must be returned to the video buffer. The number of frames which can be held
out simultaneously depends on the buffer type. This number tends to be very limited if videos come from
a hardware source.


\section errors Errors (or Help! My program aborts!)

CVD uses exception throughout for error handling. If your program aborts, then it may be due to an
uncaught exception. All CVD exceptions belong to the same heirachy, so the following code will print 
a useful diagnostic error message:

@code
#include <cvd/image_io.h>
using namespace CVD;

int main()
{
  try
  {
    Image<byte> in = img_load("this_file_does_not_exist");
  }
  catch(Exceptions::All error)
  {
	std::cerr << "Error: " << error.what << std::endl;
  }

  return 0;
}
@endcode

**/
