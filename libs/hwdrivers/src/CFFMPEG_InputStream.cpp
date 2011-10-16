/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                       http://www.mrpt.org/                                |
   |                                                                           |
   |   Copyright (C) 2005-2011  University of Malaga                           |
   |                                                                           |
   |    This software was written by the Machine Perception and Intelligent    |
   |      Robotics Lab, University of Malaga (Spain).                          |
   |    Contact: Jose-Luis Blanco  <jlblanco@ctima.uma.es>                     |
   |                                                                           |
   |  This file is part of the MRPT project.                                   |
   |                                                                           |
   |     MRPT is free software: you can redistribute it and/or modify          |
   |     it under the terms of the GNU General Public License as published by  |
   |     the Free Software Foundation, either version 3 of the License, or     |
   |     (at your option) any later version.                                   |
   |                                                                           |
   |   MRPT is distributed in the hope that it will be useful,                 |
   |     but WITHOUT ANY WARRANTY; without even the implied warranty of        |
   |     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         |
   |     GNU General Public License for more details.                          |
   |                                                                           |
   |     You should have received a copy of the GNU General Public License     |
   |     along with MRPT.  If not, see <http://www.gnu.org/licenses/>.         |
   |                                                                           |
   +---------------------------------------------------------------------------+ */

#include <mrpt/hwdrivers.h> // Precompiled headers


#include <mrpt/config.h>
#include <mrpt/utils/utils_defs.h>

#if MRPT_HAS_FFMPEG
	extern "C"
	{
	#define _MSC_STDINT_H_    // We already have pstdint.h in MRPT
	#include <avformat.h>
	#include <avcodec.h>
	#include <swscale.h>
	}
#endif


#include <mrpt/hwdrivers/CFFMPEG_InputStream.h>



using namespace mrpt;
using namespace mrpt::hwdrivers;

// JLBC: This file takes portions of code from the example "avcodec_sample.0.4.9.cpp"
#if MRPT_HAS_FFMPEG
namespace mrpt
{
	namespace hwdrivers
	{
		// All context for ffmpeg:
		struct TFFMPEGContext
		{
			AVFormatContext *pFormatCtx;
			int             videoStream;
			AVCodecContext  *pCodecCtx;
			AVCodec         *pCodec;
			AVFrame         *pFrame;
			AVFrame         *pFrameRGB;
			SwsContext		*img_convert_ctx;
			std::vector<uint8_t>  buffer;
		};
	}
}
#endif

#define MY_FFMPEG_STATE	const_cast<TFFMPEGContext*>(static_cast<const TFFMPEGContext*>(m_state.get()))



/* --------------------------------------------------------
					Ctor
   -------------------------------------------------------- */
CFFMPEG_InputStream::CFFMPEG_InputStream()
{
#if MRPT_HAS_FFMPEG
	m_state.set( new TFFMPEGContext[1] );
	TFFMPEGContext *ctx = MY_FFMPEG_STATE;

	ctx->pFormatCtx = NULL;
	ctx->pCodecCtx = NULL;
	ctx->pCodec = NULL;
	ctx->videoStream = 0;
	ctx->pFrame = NULL;
	ctx->pFrameRGB = NULL;
	ctx->img_convert_ctx = NULL;

    // Register all formats and codecs
    av_register_all();
#else
	THROW_EXCEPTION("MRPT has been compiled without FFMPEG libraries.")
#endif
}

/* --------------------------------------------------------
					Dtor
   -------------------------------------------------------- */
CFFMPEG_InputStream::~CFFMPEG_InputStream()
{
#if MRPT_HAS_FFMPEG
	// Close everything:
	this->close();

	// Free context struct. memory
	delete[] MY_FFMPEG_STATE;
	m_state.set(NULL);
#endif
}

/* --------------------------------------------------------
					isOpen
   -------------------------------------------------------- */
bool CFFMPEG_InputStream::isOpen() const
{
#if MRPT_HAS_FFMPEG
	TFFMPEGContext *ctx = MY_FFMPEG_STATE;
	return ctx->pFormatCtx != NULL;
#else
	return false;
#endif
}

/* --------------------------------------------------------
					openURL
   -------------------------------------------------------- */
bool CFFMPEG_InputStream::openURL( const std::string &url, bool grab_as_grayscale, bool verbose  )
{
#if MRPT_HAS_FFMPEG
	this->close();	// Close first

	TFFMPEGContext *ctx = MY_FFMPEG_STATE;

	this->m_url = url;
	this->m_grab_as_grayscale = grab_as_grayscale;

    // Open video file
#if LIBAVCODEC_VERSION_INT >= AV_VERSION_INT(53,2,0)
     if(avformat_open_input( &ctx->pFormatCtx, url.c_str(), NULL, NULL)!=0)
#else
     if(av_open_input_file( &ctx->pFormatCtx, url.c_str(), NULL, 0, NULL)!=0)
#endif

     {
          ctx->pFormatCtx = NULL;
          std::cerr << "[CFFMPEG_InputStream::openURL] Cannot open video: " << url << std::endl;
               return false;
     }

    // Retrieve stream information
    if(av_find_stream_info(ctx->pFormatCtx)<0)
    {
    	std::cerr << "[CFFMPEG_InputStream::openURL] Couldn't find stream information: " << url << std::endl;
        return false;
    }

    // Dump information about file onto standard error
    if (verbose)
    {
#if LIBAVCODEC_VERSION_INT >= AV_VERSION_INT(53,2,0)
          av_dump_format(ctx->pFormatCtx, 0, url.c_str(), false);
#else
		dump_format(ctx->pFormatCtx, 0, url.c_str(), false);
#endif
    }

    // Find the first video stream
    ctx->videoStream=-1;
    for(unsigned int i=0; i<ctx->pFormatCtx->nb_streams; i++)
    {
		if(ctx->pFormatCtx->streams[i]->codec->codec_type==
#if LIBAVCODEC_VERSION_INT<AV_VERSION_INT(53,0,0)
			CODEC_TYPE_VIDEO
#else
			AVMEDIA_TYPE_VIDEO
#endif
			)
        {
            ctx->videoStream=(int)i;
            break;
        }
    }
    if(ctx->videoStream==-1)
    {
    	std::cerr << "[CFFMPEG_InputStream::openURL] Didn't find a video stream: " << url << std::endl;
        return false;
    }

    // Get a pointer to the codec context for the video stream
    ctx->pCodecCtx= ctx->pFormatCtx->streams[ctx->videoStream]->codec;

    // Find the decoder for the video stream
    ctx->pCodec=avcodec_find_decoder(ctx->pCodecCtx->codec_id);
    if(ctx->pCodec==NULL)
    {
    	std::cerr << "[CFFMPEG_InputStream::openURL] Codec not found: " << url << std::endl;
        return false;
    }

    // Open codec
    if(avcodec_open(ctx->pCodecCtx, ctx->pCodec)<0)
    {
    	std::cerr << "[CFFMPEG_InputStream::openURL] Could not open codec: " << url << std::endl;
        return false;
    }

    // Allocate video frame
    ctx->pFrame=avcodec_alloc_frame();

    // Allocate an AVFrame structure
    ctx->pFrameRGB=avcodec_alloc_frame();

    if(ctx->pFrameRGB==NULL || ctx->pFrame==NULL)
    {
    	std::cerr << "[CFFMPEG_InputStream::openURL] Could not alloc memory for frame buffers: " << url << std::endl;
        return false;
    }

    // Determine required buffer size and allocate buffer
    size_t numBytes=avpicture_get_size(
		m_grab_as_grayscale ? PIX_FMT_GRAY8 : PIX_FMT_BGR24,   // BGR vs. RGB for OpenCV
		ctx->pCodecCtx->width,
		ctx->pCodecCtx->height);

    ctx->buffer.resize(numBytes);

    // Assign appropriate parts of buffer to image planes in pFrameRGB
    avpicture_fill(
		(AVPicture *)ctx->pFrameRGB,
		&ctx->buffer[0],
		m_grab_as_grayscale ? PIX_FMT_GRAY8 : PIX_FMT_BGR24,   // BGR vs. RGB for OpenCV
		ctx->pCodecCtx->width,
		ctx->pCodecCtx->height);


	return true; // OK.
#else
	return false;
#endif
}

/* --------------------------------------------------------
					close
   -------------------------------------------------------- */
void CFFMPEG_InputStream::close()
{
#if MRPT_HAS_FFMPEG
	if (!this->isOpen()) return;

	TFFMPEGContext *ctx = MY_FFMPEG_STATE;

    // Close the codec
    if (ctx->pCodecCtx)
    {
		avcodec_close(ctx->pCodecCtx);
		ctx->pCodecCtx=NULL;
    }

    // Close the video file
    if (ctx->pFormatCtx)
    {
		av_close_input_file(ctx->pFormatCtx);
		ctx->pFormatCtx = NULL;
    }

    // Free frames memory:
    ctx->buffer.clear();

    if (ctx->pFrameRGB)
    {
		av_free(ctx->pFrameRGB);
		ctx->pFrameRGB=NULL;
    }
    if (ctx->pFrame)
    {
		av_free(ctx->pFrame);
		ctx->pFrame = NULL;
    }

	if (ctx->img_convert_ctx)
	{
		sws_freeContext( ctx->img_convert_ctx );
		ctx->img_convert_ctx = NULL;
	}

#endif
}

/* --------------------------------------------------------
					retrieveFrame
   -------------------------------------------------------- */
bool CFFMPEG_InputStream::retrieveFrame( mrpt::utils::CImage &out_img )
{
#if MRPT_HAS_FFMPEG
	if (!this->isOpen()) return false;

	TFFMPEGContext *ctx = MY_FFMPEG_STATE;

    AVPacket        packet;
    int             frameFinished;

    while(av_read_frame(ctx->pFormatCtx, &packet)>=0)
    {
        // Is this a packet from the video stream?
        if(packet.stream_index==ctx->videoStream)
        {
            // Decode video frame
#if LIBAVCODEC_VERSION_MAJOR>52 || (LIBAVCODEC_VERSION_MAJOR==52 && LIBAVCODEC_VERSION_MINOR>=72)
            avcodec_decode_video2(
				ctx->pCodecCtx,
				ctx->pFrame,
				&frameFinished,
                &packet);
#else
            avcodec_decode_video(
				ctx->pCodecCtx,
				ctx->pFrame,
				&frameFinished,
                packet.data,
                packet.size);
#endif
            // Did we get a video frame?
            if(frameFinished)
            {
                // Convert the image from its native format to RGB:
				ctx->img_convert_ctx = sws_getCachedContext(
					ctx->img_convert_ctx,
					ctx->pCodecCtx->width,
					ctx->pCodecCtx->height,
					ctx->pCodecCtx->pix_fmt,
					ctx->pCodecCtx->width,
					ctx->pCodecCtx->height,
					m_grab_as_grayscale ? PIX_FMT_GRAY8 : PIX_FMT_BGR24,   // BGR vs. RGB for OpenCV
					SWS_BICUBIC,
					NULL, NULL, NULL);

				sws_scale(
					ctx->img_convert_ctx,
					ctx->pFrame->data,
					ctx->pFrame->linesize,0,
					ctx->pCodecCtx->height,
					ctx->pFrameRGB->data,
					ctx->pFrameRGB->linesize);

				/*	JL: Old code (deprecated)
				img_convert(
					(AVPicture *)ctx->pFrameRGB,
					m_grab_as_grayscale ? PIX_FMT_GRAY8 : PIX_FMT_BGR24,   // BGR vs. RGB for OpenCV
                    (AVPicture*)ctx->pFrame,
                    ctx->pCodecCtx->pix_fmt,
                    ctx->pCodecCtx->width,
                    ctx->pCodecCtx->height
                    ); */

				//std::cout << "[retrieveFrame] Generating image: " << ctx->pCodecCtx->width << "x" << ctx->pCodecCtx->height << std::endl;
				//std::cout << "  linsize: " << ctx->pFrameRGB->linesize[0] << std::endl;

				if( ctx->pFrameRGB->linesize[0]!= ((m_grab_as_grayscale ? 1:3)*ctx->pCodecCtx->width) )
					THROW_EXCEPTION("FIXME: linesize!=width case not handled yet.")

				out_img.loadFromMemoryBuffer(
					ctx->pCodecCtx->width,
					ctx->pCodecCtx->height,
					!m_grab_as_grayscale, // Color
					ctx->pFrameRGB->data[0]
					);

				// Free the packet that was allocated by av_read_frame
				av_free_packet(&packet);
				return true;
            }
        }

        // Free the packet that was allocated by av_read_frame
        av_free_packet(&packet);
    }

    return false; // Error reading/ EOF
#else
	return false;
#endif
}

/* --------------------------------------------------------
					getVideoFPS
   -------------------------------------------------------- */
double CFFMPEG_InputStream::getVideoFPS() const
{
#if MRPT_HAS_FFMPEG
	if (!this->isOpen()) return -1;

	TFFMPEGContext *ctx = MY_FFMPEG_STATE;
	if (!ctx) return -1;
	if (!ctx->pCodecCtx) return -1;

	return static_cast<double>(ctx->pCodecCtx->time_base.den) / ctx->pCodecCtx->time_base.num;
#else
	return false;
#endif
}

