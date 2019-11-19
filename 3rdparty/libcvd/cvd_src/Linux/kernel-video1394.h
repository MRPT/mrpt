#ifndef CVD_KERNEL_VIDEO1394_H
#define CVD_KERNEL_VIDEO1394_H

//Header file which contains stuff which is 100% compatible with 
//drivers/ieee1394/video1394.h
//without including any GPL code.


#include <sys/ioctl.h>
#include <sys/time.h>

#define SYNC_FRAMES      1

struct cvd_video1394_mmap 
{
	int channel_number;
	unsigned int syncronization_tag;
	unsigned int num_buffers;
	unsigned int buffer_size;
	unsigned int size_of_packet;
	unsigned int frames_per_second;
	unsigned int syt_time_offset;
	unsigned int capture_flags;		
};

struct cvd_video1394_wait 
{
	unsigned int channel_number;
	unsigned int buffer;
	struct timeval time;				//Buffer filled up at this time
};


//Define the ioctls

#if (((CVD_KERNEL_MAJOR==2) && (CVD_KERNEL_MINOR==6)) || (CVD_KERNEL_MAJOR>=3))
    #define LISTEN_CHANNEL          _IOWR('#', 0x10, struct cvd_video1394_mmap)
    #define UNLISTEN_CHANNEL        _IOW ('#', 0x11, int)
    #define LISTEN_QUEUE_BUFFER     _IOW ('#', 0x12, struct cvd_video1394_wait)
    #define LISTEN_WAIT_BUFFER      _IOWR('#', 0x13, struct cvd_video1394_wait)
    #define LISTEN_POLL_BUFFER      _IOWR('#', 0x18, struct cvd_video1394_wait)
#elif ((CVD_KERNEL_MAJOR==2) && (CVD_KERNEL_MINOR==4))
    #define LISTEN_CHANNEL          0
    #define UNLISTEN_CHANNEL        1
    #define LISTEN_QUEUE_BUFFER     2
    #define LISTEN_WAIT_BUFFER      3
    #define LISTEN_POLL_BUFFER      8
#else 
    #define UNKNOWN_KERNEL
#endif

#endif
