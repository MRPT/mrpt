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
/////////////////////////////////////////////////////////////////////////////
//                                                                         //
//  videoplay.C                                                            //
//                                                                         //
//  Test program for videofilebuffer                                       //
//                                                                         //
//  Paul Smith    30 April 2004                                            //
//                                                                         //
/////////////////////////////////////////////////////////////////////////////

#include <unistd.h>
#include <cvd/image.h>
#include <typeinfo>
#include <cstdlib>
#include <thread>
#include <condition_variable>
#include <atomic>
#include <mutex>
#include <cvd/rgb.h>
#include <cvd/byte.h>
#include <cvd/glwindow.h>
#include <cvd/gl_helpers.h>
#include <cvd/videosource.h>

#include "progs/tinyformat.h"

#ifdef CVD_HAVE_V4LBUFFER
	#include <cvd/Linux/v4lbuffer.h>
#endif

using namespace std;
using namespace CVD;


class Actions: public GLWindow::EventHandler
{	
	public:
		int paused;
		int advance_one;
		int back_one;
		int expose;
		int quit;
		int recording;
		int new_recording;


		Actions()
		:paused(0),advance_one(0),expose(0),quit(0),recording(0),new_recording(0)
		{}

		void clear()
		{
			advance_one=0;
			expose=0;
			back_one=0;
			new_recording=0;
		}

		virtual void on_key_down(GLWindow&, int key)
		{
			if(key == ' ' || key == 'p')
				paused = !paused;
			else if (key == 'q' || key== 27)
			{
				recording=0;
				quit=1;
			}
			else if(key == '.')
			{
				advance_one=1;
				paused=1;
			}
			else if(key == ',')
			{
				back_one=1;
				paused=1;
			}
			else if(key == 'r')
			{
				recording=!recording;
				if(recording)
					new_recording = true;
			}
		}

		virtual void on_event(GLWindow&, int e)
		{
			if(e == GLWindow::EVENT_EXPOSE)
				expose=1;
			else if(e == GLWindow::EVENT_CLOSE)
				quit=1;
		}

		virtual void on_resize(GLWindow&, ImageRef size)
		{
			glViewport(0, 0, size.x, size.y);
			glMatrixMode(GL_PROJECTION);
			glLoadIdentity();
			glRasterPos2f(-1, 1);
			glOrtho(-0.375, size.x-0.375, size.y-0.375, -0.375, -1 , 1); //offsets to make (0,0) the top left pixel (rather than off the display)
		}

};


template<class C>
class MessageQueue
{
	private:
		std::deque<C> data;
		std::mutex queue_mutex;
		std::atomic<int> length;
		std::condition_variable empty;


	public:
		MessageQueue()
		{
			length=0;
		}
		void push(const C& a)
		{
			lock_guard<mutex> lock(queue_mutex);
			data.push_back(a);
			length = data.size();
			empty.notify_one();
		}

		void push(C&& a)
		{
			lock_guard<mutex> lock(queue_mutex);
			data.emplace_back(move(a));
			length = data.size();
			empty.notify_one();
		}

		C pop()
		{
			unique_lock<mutex> lock(queue_mutex);

			while(data.empty())
				empty.wait(lock);

			C a = move(data.front());
			data.pop_front();
			length = data.size();
			return a;
		}

		int get_length() const
		{
			return length;
		}
};



template<class C> void play(string s, string fmt, unsigned int decimate)
{
	VideoBuffer<C> *buffer = open_video_source<C>(s);
	
	GLWindow display(buffer->size());
	glDrawBuffer(GL_BACK);
	
	double dt = 1.001/buffer->frame_rate();
	
	cout << "FPS: " << buffer->frame_rate() << " " << dt << endl;
	cout << "Size: " << buffer->size() << endl;

	RawVideoBuffer* root = buffer->root_buffer();

	cout << typeid(*root).name() << endl;

	#ifdef CVD_HAVE_V4LBUFFER
		if(dynamic_cast<V4L::RawV4LBuffer*>(root))
		{
			cout << "Using V4LBuffer.\n";
		}
	#endif

	GLenum texTarget;
	#ifdef GL_TEXTURE_RECTANGLE_ARB
	texTarget=GL_TEXTURE_RECTANGLE_ARB;
	#else
	#ifdef GL_TEXTURE_RECTANGLE_NV
	texTarget=GL_TEXTURE_RECTANGLE_NV;
	#else
	texTarget=GL_TEXTURE_RECTANGLE_EXT;
	#endif
	#endif



	VideoFrame<C>* frame=0;
	bool f=1;
	GLWindow::EventSummary e;
	glDisable(GL_BLEND);
	glEnable(texTarget);
	glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
	glTexParameterf( texTarget, GL_TEXTURE_MIN_FILTER, GL_NEAREST );
	glTexParameterf( texTarget, GL_TEXTURE_MAG_FILTER, GL_NEAREST );
	glPixelStorei(GL_UNPACK_ALIGNMENT,1);
	bool new_frame=0;

	Actions a;

	string action="Playing";
	string rec;
	bool old_paused = 0;

	int rec_sequence=-1;
	int rec_number=0;

	MessageQueue<pair<unique_ptr<Image<C>>,string>> save_queue;

	vector<thread> savers;
	
	//Spawn some saver threads
	for(unsigned int i=0; i < thread::hardware_concurrency(); i++)
	{
		savers.push_back(thread([&](){
			for(;;)
			{
				auto s=save_queue.pop();
				
				//This essentially indicates a quit.
				if(s.first == nullptr)
					break;
				
				try{
					img_save(*(s.first), s.second);
				}
				catch(CVD::Exceptions::All e)
				{
					cerr << "Error saving: " << fmt << ": " << e.what << endl;
				}
			}
		}));
	}

	for(;;)
	{
		a.clear();
		display.handle_events(a);

		if(a.new_recording)
		{
			rec_sequence++;
			rec_number=0;
		}

		if(a.quit && save_queue.get_length() == 0)
			break;

		new_frame=0;
		if(!a.paused || a.advance_one || a.back_one)
		{
			if(a.back_one)
			{
				buffer->seek_to(frame->timestamp() - dt);
			}

			if(frame)
				buffer->put_frame(frame);
			frame = buffer->get_frame();
			new_frame=1;

			if(a.recording && (rec_number % decimate) == 0)
			{
				rec = tfm::format(fmt.c_str(), rec_sequence, rec_number);
				unique_ptr<Image<C>> img = make_unique<Image<C>>();
				img->copy_from(*frame);
				save_queue.push(move(make_pair(move(img), rec)));
			}
			else
				rec = "";

			rec_number++;

			glTexImage2D(*frame, 0, GL_TEXTURE_RECTANGLE_NV);
		}


		if(old_paused != a.paused)
			new_frame = true;
		old_paused = a.paused;

		if(a.paused)
			action = "Paused";
		else if (a.recording)
			action = "Recording";
		else if (a.quit)
			action = "Quitting";
		else
			action = "Playing";


		if(a.expose || new_frame)
		{
			glEnable(texTarget);
			glBegin(GL_QUADS);
			glTexCoord2i(0, 0);
			glVertex2i(0,0);
			glTexCoord2i(frame->size().x, 0);
			glVertex2i(display.size().x,0);
			glTexCoord2i(frame->size().x,frame->size().y);
			glVertex2i(display.size().x,display.size().y);
			glTexCoord2i(0, frame->size().y);
			glVertex2i(0, display.size().y);
			glEnd ();
			glDisable(texTarget);


			glPushMatrix();
			glTranslatef(10,30,0);
			glScalef(20,-20,20);
			glColor3f(1,0,0);
			glDrawText(action + " " + rec);
			glTranslatef(0,-1.5,0);
			if(save_queue.get_length() > 0)
				glDrawText(tfm::format("%03i queued writes", save_queue.get_length()));
			glPopMatrix();
				
			glFlush();
			display.swap_buffers();
		}

		if(f)
		{
			cout << "frame size: " << frame->size() << endl;
			f=0;
		}

		if(a.paused)
			usleep(100000);
	}

	if(frame)
		buffer->put_frame(frame);
	
	cout << "Joining all threads...\n";
	//Quit all the threads.
	for(unsigned int i=0; i < savers.size(); i++)
		save_queue.push(make_pair(nullptr, ""));

	for(auto& t:savers)
		t.join();

	cout << "Exiting\n";
}

int main(int argc, char* argv[])
{
	int help = 0;
	int type=0;
	int error=0;
	unsigned int decimate=1;

	string fmt;
	int c;
		
	opterr = 0;
	while((c=getopt(argc, argv, "hmf:d:")) != -1)
	{
		if(c == 'h')
			help=1;
		else if(c == 'm')
			type=1;
		else if(c == 'f')
			fmt = optarg;	
		else if(c == 'd')
		{
			istringstream d(optarg);
			d >> decimate;

			if(!d.good())
			{
				cerr << "Error: could not parse " << optarg << " as argument for -d\n";
			}
		}
		else if(c == '?')
		{
			error=1;
			if(optopt == 'f' || optopt == 'd')
				cerr << "Error: Option -f requires an argumnt\n";
			else 
				cerr << "Error: Unknown option -" << optopt << endl;
		}
		else
			abort();

	}
	
	if(optind != argc-1)
	{	
		cerr << "Error: specify the video source\n";
		error = 1;
	}

	if(help || error)
	{
		clog << "Usage: [-h] [-m] [-f fmt] buffer \n"
			 << " -h      Display this message\n"
			 << " -m      Run the buffer in mono mode\n"
			 << " -f      Format string for recording video frames, e.g. image-%03i-%05i.png\n"
			 << "         The numbers are respectively the sequence number and the frame number\n"
			 << "         within the sequence.\n"
			 << " -d      Decimation factor for recording. 1 = record every frame.\n"
			 << "Keys: \n"
			 << "<space> pause\n"
			 << ",       reverse one frame (requires seekable buffer)\n"
			 << ".       advance one frame (requires seekable buffer)\n"
			 << "r       start/stop recording.\n"
			 << "q       quit (this may take a while if writing is in progress)\n"; 

		return (error != false);
	}

	try
	{
		if(type == 1)
			play<byte>(argv[optind], fmt, decimate);
		else
			play<Rgb<byte> >(argv[optind], fmt, decimate);
	}
	catch(CVD::Exceptions::All& e)
	{
		cout << "Error: " << e.what << endl;
	}
}
