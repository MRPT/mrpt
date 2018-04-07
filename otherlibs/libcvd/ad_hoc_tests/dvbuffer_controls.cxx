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
#include <iostream>
#include <sys/time.h>

#include <X11/keysym.h>

#include <cvd/videodisplay.h>
#include <cvd/Linux/dvbuffer.h>


#define  DATA_TYPE GL_LUMINANCE
#define  pix byte



using namespace CVD;
using namespace std;

double dbl_time()
{
	struct timeval tv;
	gettimeofday(&tv, 0);
	return tv.tv_sec + 1e-6 * tv.tv_usec;
}


int main()
{
	DVBuffer2<pix> vbuf(0, 3, 0, 0);
	VideoDisplay vd(0, 0, vbuf.size().x, vbuf.size().y);
	
	XEvent e;
	vd.select_events(ButtonPressMask|KeyPressMask);

	double new_time, old_time=dbl_time();

	unsigned int shutter, iris, gain, exposure, brightness;
	bool  		 shut_on, iris_on, gain_on, exp_on, brt_on;
	int n;
	
	vbuf.set_exposure(400);
	vbuf.set_brightness(304);


	//Some kind of default values

	shutter=6;
	iris=4;
	gain=4;
	exposure=400;
	brightness=304;

	shut_on = gain_on= iris_on = exp_on = brt_on = 0;

	vbuf.set_shutter(shutter);
	vbuf.set_iris(iris);
	vbuf.set_gain(gain);
	vbuf.set_exposure(exposure);
	vbuf.set_brightness(brightness);

	
	for(n=0;;n++)
	{
		new_time = dbl_time();

		
		/*shutter    = vbuf.get_shutter();
		iris       = vbuf.get_iris();
		gain       = vbuf.get_iris();
		exposure   = vbuf.get_exposure();
		brightness = vbuf.get_brightness();*/

		
		if(n%50 == 0)
			printf("------------------------------------------------\n"
				   "%8s%8s%8s%8s%8s%8s\n", "fps", "shut", "iris", "gain", "expos", "bright");

		
		printf("%8.3f", 1/(new_time-old_time));
		
		if(!shut_on) printf("%8i", shutter  ); else printf("    *   ");
		if(!iris_on) printf("%8i", iris     ); else printf("    *   ");
		if(!gain_on) printf("%8i", gain     ); else printf("    *   ");

		if(!exp_on) printf("%8i", exposure); else printf("   *%4i", exposure);
		if(!brt_on) printf("%8i", brightness); else printf("    *   ");

		

		//printf("%8i%8i%8i%8i%8i\n", shutter, iris, gain, exposure, brightness);
		printf("\n");

		old_time = new_time;
	
		while(vd.pending())
		{
			vd.get_event(&e);
			if(e.type == KeyPress)
			{
				KeySym k;
				XLookupString((XKeyEvent*)&e, 0, 0, &k, 0);
				
				     if(k == XK_1) shutter++;
				else if(k == XK_q) shutter--;
				else if(k == XK_a) shut_on=!shut_on;
				else if(k == XK_2) iris++;
				else if(k == XK_w) iris--;
				else if(k == XK_s) iris_on =!iris_on;
				else if(k == XK_3) gain++;
				else if(k == XK_e) gain--;
				else if(k == XK_d) gain_on=!gain_on;
				else if(k == XK_4) exposure++;
				else if(k == XK_r) exposure--;
				else if(k == XK_f) exp_on=!exp_on;
				else if(k == XK_5) brightness++;
				else if(k == XK_t) brightness--;
				else if(k == XK_g) brt_on=!brt_on;
				else if(k == XK_Escape) return 0;
			}
			else if(e.type == ButtonPress)
				return 0;

			vbuf.set_shutter(shutter);
			vbuf.set_iris(iris);
			vbuf.set_gain(gain);
			vbuf.set_exposure(exposure);
			vbuf.set_brightness(brightness);
			
			dc1394_auto_on_off(vbuf.handle(), vbuf.node(), FEATURE_SHUTTER, shut_on);
			dc1394_auto_on_off(vbuf.handle(), vbuf.node(), FEATURE_IRIS, iris_on);
			dc1394_auto_on_off(vbuf.handle(), vbuf.node(), FEATURE_GAIN, gain_on);
			dc1394_auto_on_off(vbuf.handle(), vbuf.node(), FEATURE_EXPOSURE, exp_on);
			dc1394_auto_on_off(vbuf.handle(), vbuf.node(), FEATURE_BRIGHTNESS, brt_on);
		}
	
		VideoFrame<pix>*  vf = vbuf.get_frame();

		glDrawPixels(vf->size().x, vf->size().y, DATA_TYPE, GL_UNSIGNED_BYTE, vf->data());
		vbuf.put_frame(vf);
	}
}
