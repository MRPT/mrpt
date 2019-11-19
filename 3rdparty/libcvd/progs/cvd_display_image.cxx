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

#include <cvd/image_io.h>
#include <cvd/videodisplay.h>
#include <cvd/gl_helpers.h>

#include <iostream>

using namespace CVD;
using namespace std;

int main()
{
	try
	{	
		Image<Rgb<byte> > i = img_load(cin);


		VideoDisplay d(i.size());

		d.select_events(ExposureMask|KeyPressMask);

		for(;;)
		{
			glDrawPixels(i);
			glFlush();

			XEvent e;
			d.get_event(&e);

			if(e.type == KeyPress)
				return 0;
		}
	}
	catch(Exceptions::All a)
	{
		cerr << "Error: " << a.what << endl;
	}
}
