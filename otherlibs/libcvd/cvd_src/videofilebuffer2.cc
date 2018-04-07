// IMPORTANT LICENSING NOTE!
//
// This file is licensed under the BSD license.
//
// However, if you use it in its current form you MAY be including some
// GPL licensed code from FFMPEG. Therefore any binary made using this file
// MUST be covered under the GPL.

//A mix of videofilebuffer.cc and documentation from http://plagatux.es/2011/07/using-libav-library/

#ifndef __STDC_CONSTANT_MACROS
#define __STDC_CONSTANT_MACROS
#endif

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libswscale/swscale.h>
#include <libavdevice/avdevice.h>
}

#include <string>
#include <map>
#include <utility>
#include <cvd/exceptions.h>
#include <cvd/videofilebuffer.h>


#include <iostream>
#include <cvd/image_io.h>

using namespace std;
using namespace CVD;

//#define DEBUG

#ifdef DEBUG
	#define DS(X)   do{cerr << "VideoFileBuffer2: " << X << endl;} while(0)
	#define DV(X,Y) do{cerr << "VideoFileBuffer2: " << X << " = " << Y << endl;} while(0)
	#define Dv(X)   do{cerr << "VideoFileBuffer2: " << #X << " = " << (X) << endl;} while(0)
	#define DR(X)   do{cerr << "VideoFileBuffer2: " << #X << " returned " << r << endl;} while(0)
	#define DE(X)   do{cerr << X;}while(0)
#else
	#define DS(X)   do{}while(0)
	#define DV(X,Y) do{}while(0)
	#define Dv(X)   do{}while(0)
	#define DR(X)   do{}while(0)
	#define DR(X)   do{}while(0)
	#define DE(X)   do{}while(0)
#endif

#define VS(X) do{if(verbose) cerr << "VideoFileBuffer2: " << X << endl;} while(0)
#define VV(X,Y) do{if(verbose) cerr << "VideoFileBuffer2: " << X << " = " << Y << endl;} while(0)
#define Vv(X) do{if(verbose) cerr << "VideoFileBuffer2: " << #X << " = " << (X) << endl;} while(0)
#define VR(X) do{if(verbose) cerr << "VideoFileBuffer2: " << #X << " returned " << r << endl;} while(0)
namespace CVD{

namespace Exceptions
{
	struct VideoFileBuffer2
	{
		string what;
		VideoFileBuffer2(string s_)
		:what(s_)
		{}
	};
}

template<class T>
struct PixFmt
{
};

template<>
struct PixFmt<byte>
{
	static AVPixelFormat get()
	{
		return AV_PIX_FMT_GRAY8;
	}
};

template<>
struct PixFmt<Rgb<byte> >
{
	static AVPixelFormat get()
	{
		return AV_PIX_FMT_RGB24;
	}
};


namespace VFB{




class VFHolderBase
{
	protected:
	void* frame;

	virtual void remove()=0;

	public:
	virtual ~VFHolderBase()
	{}

	void* release()
	{
		void* f=frame;	
		frame=0;
		return f;
	};

	virtual unique_ptr<VFHolderBase> duplicate()=0;
};

template<class C>
class VFHolder: public VFHolderBase
{
	using VFHolderBase::frame;

	void remove()
	{
		if(frame)
			static_cast<VideoFileFrame<C>*>(frame)->delete_self();
	};

	VFHolder(const VFHolder&);
	void operator=(const VFHolder&);

	public:
	~VFHolder()
	{
		remove();
	}

	VFHolder(VideoFileFrame<C>* f)
	{
		frame=f;
	}

	unique_ptr<VFHolderBase> duplicate();


};


class RawVideoFileBufferPIMPL
{
	bool rgb;
	AVPixelFormat           output_fmt;
	AVFormatContext *       input_format_context;
	
	int                     video_stream_index;
	AVCodecContext *        video_codec_context;

	AVFrame *               raw_image;
	AVFrame *               converted_image;

	AVPacket                packet;
	ImageRef                size;
	
	double                  frame_rate;
	double                  stream_time_base;
	double                  approx_next_timestamp;
	
	AVCodec*                video_codec;
	SwsContext*             img_convert_context;

	bool                    verbose;
	bool                    ready;
			
	VideoBufferFlags::OnEndOfBuffer end_of_buffer_behaviour;
			
	public:	
	template<class T> static VideoFileFrame<T>* generate_frame(double t, Image<T>&& im)
	{
		return new VideoFileFrame<T>(t, move(im));
	}
	private:

	unique_ptr<VFHolderBase> next_frame;

	string codec;
		
	string err(int e)
	{
		char buf[1024];
		av_strerror(e, buf, sizeof(buf)-1);
		buf[sizeof(buf)-1] = 0;
		return buf;

	}

	static double to_double(const AVRational& r)
	{
		return 1.0 * r.num / r.den;
	}

	public:

	RawVideoFileBufferPIMPL(const string& fname, const string& formatname, bool rgb_, bool verbose_, const map<string,string>& options)
	:rgb(rgb_),
	 output_fmt(rgb?PixFmt<Rgb<byte> >::get():PixFmt<byte>::get()),
	 input_format_context(0),
	 video_stream_index(-1),
	 video_codec_context(0),
	 raw_image(0),
	 converted_image(0),
	 video_codec(0),
	 img_convert_context(0),
	 verbose(verbose_),
	 ready(false),
	 end_of_buffer_behaviour(VideoBufferFlags::RepeatLastFrame)
	{

		try
		{ 
			av_register_all();
			avcodec_register_all();
			avdevice_register_all();
			DS("starting");

			int r;

			input_format_context = avformat_alloc_context();
			if(input_format_context == NULL)
				throw Exceptions::VideoFileBuffer2("Out of memory.");
			

			AVInputFormat* fmt = nullptr;
			if(formatname != "")
				fmt = av_find_input_format(formatname.c_str());

			VS("av_find_input_format(" << formatname << ") = " << fmt);	
			if(fmt != nullptr)
			{
				Vv(fmt->name);
				Vv(fmt->long_name);
				Vv(fmt->flags & AVFMT_NOFILE);
				Vv(fmt->flags & AVFMT_NEEDNUMBER);
			}
				

			AVDictionary* opts = nullptr;
			for(const auto& o: options)
				av_dict_set(&opts, o.first.c_str(), o.second.c_str(), 0);


			//avformat_open_input semes to be the latest non-depracated method for this task.
			//It requires a pre-allocated context.
			r = avformat_open_input(&input_format_context, fname.c_str(), fmt, &opts);
			VR(avformat_open_input);

			av_dict_free(&opts);

			if(r < 0)
				throw Exceptions::VideoFileBuffer::FileOpen(fname, err(r));


			r = avformat_find_stream_info(input_format_context, NULL);
			VR(av_find_stream_info);
			if(r < 0)
				throw Exceptions::VideoFileBuffer::FileOpen(fname, err(r));
			
			VV("number of streams", input_format_context->nb_streams);
			VS("Enumerating streams:");
			video_stream_index = -1;
			//Trawl through the various streams to get some nice
			//verbosity and to find the first video stream.
			for (unsigned int i=0; i< input_format_context->nb_streams; i++)
			{
				AVMediaType t = input_format_context->streams[i]->codec->codec_type;
				AVCodecContext* vc = input_format_context->streams[i]->codec;

				if(t == AVMEDIA_TYPE_UNKNOWN)
					VS("    " << i << ": " << "unknown");
				else if(t == AVMEDIA_TYPE_VIDEO)
				{
					VS("    " << i << ": " << "video");

					if(video_stream_index != -1)
					{
						VS("        warning, multiple video streams.");
					}
					else
					{
						VS("        selecting this stream.");
						video_stream_index=i;

						video_codec_context = input_format_context->streams[i]->codec;
						size.x = video_codec_context->width;
						size.y = video_codec_context->height;
					}

					AVStream& s = *(input_format_context->streams[i]);
					
					VS("        stream");
					VV("            frame rate (num)", s.r_frame_rate.num);
					VV("            frame rate (num)", s.r_frame_rate.den);
					VV("            time base (num) ", s.time_base.num);
					VV("            time base (den) ", s.time_base.den);

					VS("        codec");
					VV("            width             ", vc->width);
					VV("            height            ", vc->height);
					VV("            bit rate          ", vc->bit_rate);
					VV("            timebase (num)    ", vc->time_base.num);
					VV("            timebase (denom)  ", vc->time_base.den);
					VV("            ticks per frame   ", vc->ticks_per_frame);

					//Frame timestamps are represented as integer multiples of
					//rationals with the obvious definition.  The number of
					//incrementes between frames is not necessarily 1, so this
					//number cannot be used to compute the frame rate.

					//The number of ticks per frame gives the frame rate.
					//frame_rate = 1. / ( 1.0 * vc->ticks_per_frame*  vc->time_base.num / vc->time_base.den);

					//The frame rate is also recorded in the stream...
					frame_rate = to_double(s.r_frame_rate);
					VV("        frame rate (?)    ", frame_rate);

					//The ticks is usually 2 for interlaced video and 1 otherwise.
					//However, some progressive video converted from interlaced also has
					//ticks=2

					//But the frame rate is also recorded in the 
					
					//Get the stream time base, since this is required to 
					//turn the packet timestamps into seconds.
					stream_time_base  = to_double(s.time_base);

				}
				else if(t == AVMEDIA_TYPE_AUDIO)
					VS("    " << i << ": " << "audio");
				else if(t == AVMEDIA_TYPE_DATA)
					VS("    " << i << ": " << "data");
				else if(t == AVMEDIA_TYPE_SUBTITLE)
					VS("    " << i << ": " << "subtitle");
				else if(t == AVMEDIA_TYPE_ATTACHMENT)
					VS("    " << i << ": " << "attachment");

				//For some reason vc->codec_name is empty.
				//This seems to work.
				char buf[1024];
				avcodec_string(buf, sizeof(buf)-1, vc, false);
				buf[sizeof(buf)-1]=0;
				VV("        codec name        ", buf << "*"); 
				codec = buf;

			}

			if(video_stream_index == -1)
				throw Exceptions::VideoFileBuffer2("VideoFileBuffer2: no video stream found.");
			
			//Get hold of the video decoding mechanism
			video_codec = avcodec_find_decoder(video_codec_context->codec_id);
			if(video_codec == NULL)
				throw Exceptions::VideoFileBuffer2("VideoFileBuffer2: no decoder found");

			r = avcodec_open2(video_codec_context, video_codec, 0);
			VR(avcodec_open2);
			if(r < 0)
				throw Exceptions::VideoFileBuffer::FileOpen(fname, err(r));
			

			raw_image = av_frame_alloc();

			//Get hold of the mechanism to convert the video frame into
			//a useful pixel format. The sws part is GPL, not LGPL.
			//
			//In principle, this could be done using libCVD's internal conversion
			//code in order to break the GPL dependence.
			converted_image= av_frame_alloc();
			img_convert_context  = sws_getContext(size.x, size.y, video_codec_context->pix_fmt, //Input
			                                      size.x, size.y, output_fmt,                   //Output
			                                      SWS_POINT, //The nastiest scaler should be OK, since we're not scaling. Right? right?
												  NULL, NULL, NULL);

			if(img_convert_context == 0)
				throw Exceptions::VideoFileBuffer2("Software scaler not found! This is very strange.");

			
			//FIXME: streams do not always start at 0
			approx_next_timestamp = 0;
			
			//Load the first frame...
			load_next_frame();

			return;
		}
		catch(...)
		{
			free_all();
			throw;
		}
	}

	void on_end_of_buffer(VideoBufferFlags::OnEndOfBuffer f)
	{
		end_of_buffer_behaviour = f;
	}

	void seek_to(double t)
	{
		t = max(0., t);

		uint64_t stamp = floor(t / stream_time_base + 0.5);
		
		DE(endl);
		DS("seeking");
		Dv(stamp);

		int r = av_seek_frame(input_format_context, video_stream_index, stamp, AVSEEK_FLAG_ANY);
		approx_next_timestamp = t;

		DE(endl);
		
		if(r < 0)
			throw Exceptions::VideoFileBuffer::BadSeek(t, err(r));
	}

	template<typename T>
	unique_ptr<VFHolderBase> read_frame_from_video()
	{

		if(PixFmt<T>::get() != output_fmt)
			throw Exceptions::VideoFileBuffer2("Mismatched format: probably an internal library error.");



		while(true)
		{
			av_init_packet(&packet);
			int r = av_read_frame(input_format_context, &packet);	
			DR(av_read_frame);

			if(r == AVERROR_EOF)
			{
				DS("EOF");
				return unique_ptr<VFHolderBase>();
			}
			else if(r < 0)
			{
				throw Exceptions::VideoFileBuffer::BadDecode(approx_next_timestamp, err(r));
			}

			Dv(packet.stream_index);

			Dv(packet.pts); //Presentation time stamp (pts) is sometimes junk in keyframes.
			Dv(packet.dts); //decode time stamp
			double timestamp = packet.dts * stream_time_base;
			approx_next_timestamp = timestamp + 1./ frame_rate;
			Dv(timestamp);

			//The duration can lie and be set to 0
			Dv(packet.duration);
			DV("duration", packet.duration * stream_time_base);
	

			if(packet.stream_index == video_stream_index)
			{
				int got_picture;
				avcodec_decode_video2(video_codec_context, raw_image, &got_picture, &packet);
				Dv(got_picture);
				Dv(raw_image->key_frame);
				Dv(raw_image->pts); //presentation timestamp. Time to present frame. Seems to contain junk. Do not use.
				Dv(raw_image->interlaced_frame);
				Dv(raw_image->top_field_first);

				if(!got_picture)
					goto cont;
			
				//Allocate memory for the converted image
				Image<T> ret(size);

				//Set up converted_image so it uses the data in ret as its data buffer.
				avpicture_fill((AVPicture*)converted_image, reinterpret_cast<uint8_t*>(ret.data()), output_fmt, size.x, size.y);
				sws_scale(img_convert_context, raw_image->data, raw_image->linesize, 0, size.y, converted_image->data, converted_image->linesize);
				

				ready=true;
				return unique_ptr<VFHolderBase>(new VFHolder<T>(new VideoFileFrame<T>(timestamp, std::move(ret))));
			}
			
			cont:

			//Free the data owned by packet, not the struct tiself
			av_free_packet(&packet);
			DE(endl);
		}
	}

	void load_next_frame()
	{
		if(output_fmt == AV_PIX_FMT_GRAY8)
			next_frame= read_frame_from_video<byte>();
		else// if(output_fmt == PIX_FMT_RGB24)
			next_frame= read_frame_from_video<Rgb<byte> >();
	}
	
	void put_frame(void* f)
	{
		if(output_fmt == AV_PIX_FMT_GRAY8)
			delete static_cast<VideoFileFrame<byte>*>(f);
		else
			delete static_cast<VideoFileFrame<Rgb<byte> >*>(f);

	};

	
	void* get_frame()
	{
		if(!frame_pending()) 
			throw Exceptions::VideoFileBuffer::EndOfFile();

		//Safely grab the frame
		unique_ptr<VFHolderBase> fr = move(next_frame);
		load_next_frame();

		if(next_frame.get() == NULL)
		{
			switch(end_of_buffer_behaviour)
			{
				case VideoBufferFlags::RepeatLastFrame:
					next_frame = (*fr).duplicate();
					break;
				
				case VideoBufferFlags::UnsetPending:
					ready=false;
				   break;
				
				case VideoBufferFlags::Loop:
					seek_to(0.0);
					load_next_frame();
					break;
			}


		}
		
		void* f = fr->release();
		fr.release();

		return f;
	}

	bool frame_pending()
	{
		return ready;
	}

	~RawVideoFileBufferPIMPL()
	{
		free_all();
	}		
	
	void free_all()
	{
		if(input_format_context != NULL)
			avformat_free_context(input_format_context);

		if(raw_image != NULL)
			av_free(raw_image);

		if(converted_image != NULL)
			av_free(converted_image);

		if(img_convert_context != NULL)
			sws_freeContext(img_convert_context);

	}

	double frames_per_second() const
	{
		return frame_rate;
	}

	string codec_name() const
	{
		return codec;
	}

	ImageRef get_size() const
	{
		return size;
	}
};

template<class C>
unique_ptr<VFHolderBase> VFHolder<C>::duplicate()
{
	VideoFileFrame<C> *fr = static_cast<VideoFileFrame<C>*>(frame);

	Image<C> copy;
	copy.copy_from(*fr);

	return unique_ptr<VFHolderBase>(new VFHolder(RawVideoFileBufferPIMPL::generate_frame<C>(fr->timestamp(), move(copy))));
}

///Public implementation of RawVideoFileBuffer
RawVideoFileBuffer::RawVideoFileBuffer(const std::string& file, const std::string& fmt, bool is_rgb, bool verbose, const map<string,string>& opts)
:p(new RawVideoFileBufferPIMPL(file, fmt, is_rgb, verbose, opts))
{}

RawVideoFileBuffer::~RawVideoFileBuffer()
{}

ImageRef RawVideoFileBuffer::size()
{
	return p->get_size();
}

bool RawVideoFileBuffer::frame_pending()
{
	return p->frame_pending();
}

void* RawVideoFileBuffer::get_frame()
{
	return p->get_frame();
}

void RawVideoFileBuffer::put_frame(void* f)
{
	p->put_frame(f);
}

void RawVideoFileBuffer::seek_to(double t)
{
	p->seek_to(t);
}

void RawVideoFileBuffer::on_end_of_buffer(VideoBufferFlags::OnEndOfBuffer b)
{
	p->on_end_of_buffer(b);
}

double RawVideoFileBuffer::frames_per_second()
{
	return p->frames_per_second();
}

string RawVideoFileBuffer::codec_name()
{
	return p->codec_name();
}


}
}
