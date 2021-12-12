/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2021, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#if defined(__GNUC__)  // Needed for ffmpeg headers. Only allowed here when not
// using precomp. headers
#define __STDC_CONSTANT_MACROS	// Needed for having "UINT64_C" and so
#endif
//
#include "hwdrivers-precomp.h"	// Precompiled headers
//
#include <mrpt/config.h>

#if MRPT_HAS_FFMPEG
extern "C"
{
#define _MSC_STDINT_H_	// We already have pstdint.h in MRPT
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavutil/imgutils.h>
#include <libswscale/swscale.h>
}
#endif

#include <mrpt/hwdrivers/CFFMPEG_InputStream.h>

using namespace mrpt;
using namespace mrpt::hwdrivers;

// JLBC: This file takes portions of code from the example
// "avcodec_sample.0.4.9.cpp"
//
// Minimum ffmpeg libs versions we want to support:
// Ubuntu 16.04 LTS: avcodec 56.60.100, avutil 54.31.100, avformat 56.40.101
// Ubuntu 20.04 LTS: avcodec 58.54.100, avutil 56.31.100, avformat 58.29.100
//
#if MRPT_HAS_FFMPEG
namespace mrpt::hwdrivers
{
// All context for ffmpeg:
struct TFFMPEGContext
{
	AVFormatContext* pFormatCtx{nullptr};
	int videoStream{0};
#if LIBAVFORMAT_VERSION_MAJOR >= 57
	AVCodecParameters* pCodecPars{nullptr};
#endif
	AVCodec* pCodec{nullptr};
	AVCodecContext* pCodecCtx{nullptr};
	AVFrame* pFrame{nullptr};
	AVFrame* pFrameRGB{nullptr};
	SwsContext* img_convert_ctx{nullptr};
	std::vector<uint8_t> buffer;
};
}  // namespace mrpt::hwdrivers
#endif

struct CFFMPEG_InputStream::Impl
{
#if MRPT_HAS_FFMPEG
	TFFMPEGContext m_state;
#endif
};

/* --------------------------------------------------------
					Ctor
   -------------------------------------------------------- */
CFFMPEG_InputStream::CFFMPEG_InputStream()
#if MRPT_HAS_FFMPEG
	: m_impl(mrpt::make_impl<CFFMPEG_InputStream::Impl>())
{
// av_register_all() not needed in ffmpeg >=4.0
#if LIBAVFORMAT_VERSION_MAJOR < 58
	// Register all formats and codecs
	av_register_all();
#endif
}
#else
{
	THROW_EXCEPTION("MRPT has been compiled without FFMPEG libraries.");
}
#endif

/* --------------------------------------------------------
					Dtor
   -------------------------------------------------------- */
CFFMPEG_InputStream::~CFFMPEG_InputStream()
{
#if MRPT_HAS_FFMPEG
	// Close everything:
	this->close();
#endif
}

/* --------------------------------------------------------
					isOpen
   -------------------------------------------------------- */
bool CFFMPEG_InputStream::isOpen() const
{
#if MRPT_HAS_FFMPEG
	const TFFMPEGContext* ctx = &m_impl->m_state;
	return ctx->pFormatCtx != nullptr;
#else
	return false;
#endif
}

/* --------------------------------------------------------
					openURL
   -------------------------------------------------------- */
bool CFFMPEG_InputStream::openURL(
	const std::string& url, bool grab_as_grayscale, bool verbose,
	const std::map<std::string, std::string>& optionsMap)
{
#if MRPT_HAS_FFMPEG
	this->close();	// Close first

	TFFMPEGContext* ctx = &m_impl->m_state;

	this->m_url = url;
	this->m_grab_as_grayscale = grab_as_grayscale;

	AVDictionary* options = nullptr;  // "create" an empty dictionary
	for (const auto& kv : optionsMap)
		av_dict_set(&options, kv.first.c_str(), kv.second.c_str(), 0);

	// Open video file
	if (avformat_open_input(&ctx->pFormatCtx, url.c_str(), nullptr, &options) !=
		0)
	{
		ctx->pFormatCtx = nullptr;
		std::cerr << "[CFFMPEG_InputStream::openURL] Cannot open video: " << url
				  << std::endl;
		return false;
	}

	// Retrieve stream information
	if (avformat_find_stream_info(ctx->pFormatCtx, nullptr) < 0)
	{
		std::cerr << "[CFFMPEG_InputStream::openURL] Couldn't find stream "
					 "information: "
				  << url << std::endl;
		return false;
	}

	// Dump information about file onto standard error
	if (verbose) { av_dump_format(ctx->pFormatCtx, 0, url.c_str(), false); }

	// Find the first video stream
	ctx->videoStream = -1;
	for (unsigned int i = 0; i < ctx->pFormatCtx->nb_streams; i++)
	{
#if LIBAVFORMAT_VERSION_MAJOR >= 57
		auto codecType = ctx->pFormatCtx->streams[i]->codecpar->codec_type;
#else
		auto codecType = ctx->pFormatCtx->streams[i]->codec->codec_type;
#endif
		if (codecType == AVMEDIA_TYPE_VIDEO)
		{
			ctx->videoStream = (int)i;
			break;
		}
	}
	if (ctx->videoStream == -1)
	{
		std::cerr
			<< "[CFFMPEG_InputStream::openURL] Didn't find a video stream: "
			<< url << std::endl;
		return false;
	}

	// Get a pointer to the codec context for the video stream
#if LIBAVFORMAT_VERSION_MAJOR >= 57
	ctx->pCodecPars = ctx->pFormatCtx->streams[ctx->videoStream]->codecpar;
	// Find the decoder for the video stream
	ctx->pCodec = avcodec_find_decoder(ctx->pCodecPars->codec_id);
#else
	ctx->pCodecCtx = ctx->pFormatCtx->streams[ctx->videoStream]->codec;
	// Find the decoder for the video stream
	ctx->pCodec = avcodec_find_decoder(ctx->pCodecCtx->codec_id);
#endif
	if (ctx->pCodec == nullptr)
	{
		std::cerr << "[CFFMPEG_InputStream::openURL] Codec not found: " << url
				  << std::endl;
		return false;
	}

#if LIBAVFORMAT_VERSION_MAJOR >= 57
	ctx->pCodecCtx = avcodec_alloc_context3(nullptr /*ctx->pCodec*/);
	if (!ctx->pCodecCtx)
	{
		std::cerr << "[CFFMPEG_InputStream::openURL] Cannot alloc avcodec "
					 "context for: "
				  << url << std::endl;
		return false;
	}

	// Add stream parameters to context
	if (avcodec_parameters_to_context(
			ctx->pCodecCtx,
			ctx->pFormatCtx->streams[ctx->videoStream]->codecpar))
	{
		std::cerr << "[CFFMPEG_InputStream::openURL] Failed "
					 "avcodec_parameters_to_context() for: "
				  << url << std::endl;
		return false;
	}

	// Make sure that Codecs are identical or  avcodec_open2 fails.
	ctx->pCodecCtx->codec_id = ctx->pCodec->id;
#endif

	// Open codec
	if (avcodec_open2(ctx->pCodecCtx, ctx->pCodec, nullptr) < 0)
	{
		std::cerr
			<< "[CFFMPEG_InputStream::openURL] avcodec_open2() failed for: "
			<< url << std::endl;
		return false;
	}

	// Allocate video frame
	ctx->pFrame = av_frame_alloc();
	// Allocate an AVFrame structure
	ctx->pFrameRGB = av_frame_alloc();

	if (ctx->pFrameRGB == nullptr || ctx->pFrame == nullptr)
	{
		std::cerr << "[CFFMPEG_InputStream::openURL] Could not alloc memory "
					 "for frame buffers: "
				  << url << std::endl;
		return false;
	}

	// Determine required buffer size and allocate buffer
#if LIBAVFORMAT_VERSION_MAJOR >= 57
	const auto width = ctx->pCodecPars->width, height = ctx->pCodecPars->height;
#else
	const auto width = ctx->pCodecCtx->width, height = ctx->pCodecCtx->height;
#endif
	int numBytes = av_image_get_buffer_size(
		m_grab_as_grayscale ? AV_PIX_FMT_GRAY8 : AV_PIX_FMT_BGR24, width,
		height, 1);
	if (numBytes < 0)
	{
		std::cerr << "[CFFMPEG_InputStream::openURL] av_image_get_buffer_size "
					 "error code: "
				  << numBytes << std::endl;
		return false;
	}

	ctx->buffer.resize(numBytes);

	// Assign appropriate parts of buffer to image planes in pFrameRGB

	av_image_fill_arrays(
		ctx->pFrameRGB->data, ctx->pFrameRGB->linesize, &ctx->buffer[0],
		m_grab_as_grayscale ? AV_PIX_FMT_GRAY8 : AV_PIX_FMT_BGR24, width,
		height, 1);

	return true;  // OK.
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

	TFFMPEGContext* ctx = &m_impl->m_state;

	// Close the codec
	if (ctx->pCodecCtx)
	{
		avcodec_close(ctx->pCodecCtx);
		ctx->pCodecCtx = nullptr;
	}

	// Close the video file
	if (ctx->pFormatCtx)
	{
		avformat_close_input(&ctx->pFormatCtx);
		ctx->pFormatCtx = nullptr;
	}

	// Free frames memory:
	ctx->buffer.clear();

	if (ctx->pFrameRGB)
	{
		av_frame_free(&ctx->pFrameRGB);
		ctx->pFrameRGB = nullptr;
	}
	if (ctx->pFrame)
	{
		av_frame_free(&ctx->pFrame);
		ctx->pFrame = nullptr;
	}

	if (ctx->img_convert_ctx)
	{
		sws_freeContext(ctx->img_convert_ctx);
		ctx->img_convert_ctx = nullptr;
	}

#endif
}

/* --------------------------------------------------------
					retrieveFrame
   -------------------------------------------------------- */
bool CFFMPEG_InputStream::retrieveFrame(mrpt::img::CImage& out_img)
{
#if MRPT_HAS_FFMPEG
	if (!this->isOpen()) return false;

	TFFMPEGContext* ctx = &m_impl->m_state;

	AVPacket packet;

#if LIBAVFORMAT_VERSION_MAJOR < 57
	int frameFinished;
#endif

#if LIBAVFORMAT_VERSION_MAJOR >= 57
	const auto width = ctx->pCodecPars->width, height = ctx->pCodecPars->height;
#else
	const auto width = ctx->pCodecCtx->width, height = ctx->pCodecCtx->height;
#endif

	while (av_read_frame(ctx->pFormatCtx, &packet) >= 0)
	{
		// Is this a packet from the video stream?
		if (packet.stream_index != ctx->videoStream)
		{
			av_packet_unref(&packet);
			continue;
		}

		// Decode video frame
#if LIBAVFORMAT_VERSION_MAJOR >= 57
		int ret = avcodec_send_packet(ctx->pCodecCtx, &packet);
		if (ret < 0)
		{
			std::cerr << std::endl
					  << "[CFFMPEG_InputStream] avcodec_send_packet error code="
					  << ret << std::endl
					  << std::endl;
			return false;
		}
		while (ret >= 0)
		{
			ret = avcodec_receive_frame(ctx->pCodecCtx, ctx->pFrame);
			if (ret == AVERROR(EAGAIN)) continue;

			if (ret == AVERROR_EOF) return false;
			else if (ret < 0)
			{
				std::cerr << std::endl
						  << "[CFFMPEG_InputStream] avcodec_receive_frame "
							 "error code="
						  << ret << std::endl
						  << std::endl;
				return false;
			}

#else
		avcodec_decode_video2(
			ctx->pCodecCtx, ctx->pFrame, &frameFinished, &packet);
		if (!frameFinished)
		{
			// Free the packet that was allocated by av_read_frame
			av_packet_unref(&packet);
			continue;
		}
#endif
			// Convert the image from its native format to RGB:
			ctx->img_convert_ctx = sws_getCachedContext(
				ctx->img_convert_ctx, width, height, ctx->pCodecCtx->pix_fmt,
				width, height,
				m_grab_as_grayscale ?  // BGR vs. RGB for OpenCV
					AV_PIX_FMT_GRAY8
									: AV_PIX_FMT_BGR24,
				SWS_BICUBIC, nullptr, nullptr, nullptr);

			sws_scale(
				ctx->img_convert_ctx, ctx->pFrame->data, ctx->pFrame->linesize,
				0, height, ctx->pFrameRGB->data, ctx->pFrameRGB->linesize);

			// std::cout << "[retrieveFrame] Generating image: " <<
			// ctx->pCodecPars->width << "x" << ctx->pCodecPars->height
			// << std::endl; std::cout << "  linsize: " <<
			// ctx->pFrameRGB->linesize[0] << std::endl;

			if (ctx->pFrameRGB->linesize[0] !=
				((m_grab_as_grayscale ? 1 : 3) * width))
				THROW_EXCEPTION("FIXME: linesize!=width case not handled yet.");

			out_img.loadFromMemoryBuffer(
				width, height, !m_grab_as_grayscale, ctx->pFrameRGB->data[0]);

			// Free the packet that was allocated by av_read_frame
			av_packet_unref(&packet);
			return true;
		}
#if LIBAVFORMAT_VERSION_MAJOR >= 57
	}
#endif
	return false;  // Error reading/ EOF
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

	const TFFMPEGContext* ctx = &m_impl->m_state;
	if (!ctx) return -1;
	if (!ctx->pCodecCtx) return -1;

	return double(ctx->pCodecCtx->framerate.num) /
		ctx->pCodecCtx->framerate.den;
#else
	return false;
#endif
}
