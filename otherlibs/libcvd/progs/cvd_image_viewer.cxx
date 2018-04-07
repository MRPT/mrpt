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
#include <cvd/videodisplay.h>
#include <cvd/image_io.h>
#include <cvd/gl_helpers.h>
#include <X11/keysym.h>
#include <iostream>

using namespace CVD;
using namespace std;

int main(int argc, char** argv)
{
	if(argc != 1)
	{
		cerr << "Error: incorrect arguments\n.";
		cerr << "usage: " << argv[0] << " < image\n";
		return 1;
	}

	Image<Rgb<byte> > im;

	try
	{
		img_load(im, cin);
		
		VideoDisplay d(0, 0, im.size().x, im.size().y);

		d.select_events(KeyPressMask | ExposureMask);

		for(;;)
		{
			XEvent e;

			d.get_event(&e);

			if(e.type == KeyPress)
			{
				KeySym key;
				XLookupString(&e.xkey,	0, 0, &key, 0);

				if(key == XK_Escape || key == XK_q)
					break;
			}
			else if(e.type == Expose)
				glDrawPixels(im);
		}
	}
	catch(Exceptions::All b0rk)
	{
		cerr << "Error: " << b0rk.what << endl;
		return 1;
	}

	return 0;
}
