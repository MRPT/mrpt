
#ifndef CVD_READAHEADVIDEOBUFFER_H
#define CVD_READAHEADVIDEOBUFFER_H

#include <cvd/config.h>
#include <cvd/videobuffer.h>
#include <cvd/internal/concurrency_utilities.h>
#include <atomic>

namespace CVD {
#ifndef CVD_HAVE_PTHREAD
#pragma message("ReadAheadVideoBuffer will not do any read - ahead because threads are not supported in this build")
	template <class T> 
		class ReadAheadVideoBuffer : public VideoBuffer<T>
	{
		private:
			VideoBuffer<T>& vbuffer;
		public:
			virtual ~ReadAheadVideoBuffer() {}
			ReadAheadVideoBuffer(VideoBuffer<T>& vb, size_t=10) 
				: VideoBuffer<T>(vb.type()),vbuffer(vb) {}
			/// The size of the VideoFrames returned by this buffer
			ImageRef size() { return vbuffer.size(); }
			/// Returns the next frame from the buffer. This function blocks until a frame is ready.
			VideoFrame<T>* get_frame() { return vbuffer.get_frame(); }
			/// Tell the buffer that you are finished with this frame. Typically the VideoBuffer then destroys the frame.
			/// \param f The frame that you are finished with.
			void put_frame(VideoFrame<T>* f) { vbuffer.put_frame(f); }
			/// Is there a frame waiting in the buffer? This function does not block. 
			bool frame_pending() { return vbuffer.frame_pending(); }
			/// What is the (expected) frame rate of this video buffer, in frames per second?
			double frame_rate() { return vbuffer.frame_rate(); }
			/// Go to a particular point in the video buffer (only implemented in buffers of recorded video)
			/// \param t The frame time in seconds
			void seek_to(double t){ vbuffer.seek_to(t); }
	};
#else    
	/// Decorator video buffer that preloads frames asynchronously in a separate thread.
	/// @param T The pixel type of the video frames
	/// @param vb The video buffer to wrap/preload
	/// @param maxReadAhead The maximum number of frames to read ahead asynchronously; 
	///                     the underlying VideoBuffer must support this many concurrently existing VideoFrame's
	/// @ingroup gVideoBuffer
	template <class T> 
		class ReadAheadVideoBuffer : public VideoBuffer<T>, public Runnable
	{
		private:
			struct Command
			{
				enum 
				{
					STOP, FLUSH, PUT, SEEK
				} code;

				VideoBuffer<T>* frame;
				double time;

			};


			VideoBuffer<T>& vbuffer;
			std::thread       reader_thread;
			Internal::MessageQueue<VideoFrame<T>*> captured;
			Internal::MessageQueue<Command> returned;


			static VideoBufferType::Type type_update(VideoBufferType::Type t)
			{
				if(t== VideoBufferType::NotLive)
					return t; 
				else
					return VideoBufferType::Flushable;
			}

			void flush_captured()
			{
				VideoFrame<T>* vf;
				while(captured.maybe_pop(vf))
					vbuffer.put_frame(vf);
			}

		public:
			virtual ~ReadAheadVideoBuffer() 
			{
				returned.push({Command::Stop});

				//If we're waiting on a get_frame, then the captured frame will get pushed,
				//then the commands will be read and everything will get flushed out.

				//If we're waiting on inserting the captured frame, things will stall.
				VideoFrame<T>* vf=nullptr;
				captured.maybe_pop(vf);

				//At this point, the other thread will guarantee to restart and flush other stuff.
				//So, wait for it to finish.
				reader_thread.join();

				//Now return the final frame
				if(vf)
					vbuffer.put_frame(vf);
				
			}
			ReadAheadVideoBuffer(VideoBuffer<T>& vb, size_t maxReadAhead=10) 
			: VideoBuffer<T>(type_update(vb.type())), 
			  vbuffer(vb), 
			  captured(maxReadAhead)
			{
				//Start the thread
				reader_thread = move(thread([this](){this->run();}));
			}
			


			void run() {
				for(;;)
				{
					Command com;
					while(maybe_pop(com))
					{
						if(com.code == Command::PUT)
							vbuffer.put_frame(com.frame);
						else if(com.code == Command::FLUSH)
							flush_captured();
						else if(com.code == Command::SEEK)
						{
							flush_captured();
							vbuffer.seek_to(com.time);
						}
						else if(com.code == STOP)
							goto done;
					}

					VideoFrame<T>* frame = vbuffer.get_frame();
					captured.push(frame);
				}
				
				//At this point, this should be the only thread doing anything
				//with the buffer since the other thread is waiting in the destructor.
				done:

				//Empty the command queue
				Command com;
				while(maybe_pop(com))
					if(com.code == Command::PUT)
						vbuffer.put_frame(com.frame);
				
				flush_captured();
			}


			/// The size of the VideoFrames returned by this buffer
			ImageRef size() 
			{ 
				return vbuffer.size(); 
			}

			/// Returns the next frame from the buffer. This function blocks until a frame is ready.
			VideoFrame<T>* get_frame() 
			{
				return captured.pop();
			}
			/// Tell the buffer that you are finished with this frame. Typically the VideoBuffer then destroys the frame.
			/// \param f The frame that you are finished with.
			void put_frame(VideoFrame<T>* f)
			{
				returned.push({Command::PUT, f});
			}
			/// Is there a frame waiting in the buffer? This function does not block. 
			bool frame_pending() {
				return !captured.empty();
			}
			/// What is the (expected) frame rate of this video buffer, in frames per second?		
			double frame_rate() 
			{ 
				return vbuffer.frame_rate(); 
			}

			/// Go to a particular point in the video buffer (only implemented in buffers of recorded video)
			/// \param t The frame time in seconds
			void seek_to(double t)
			{
				returned.push({Command::SEEK, 0, t});
			}
	};
#endif
}

#endif
