#ifndef _VIDEO_CAPTURE_H_
#define _VIDEO_CAPTURE_H_

#define VC_MAX_PLANES 4

typedef struct video_capture* vcapture;

enum vcapture_format_type {
  VC_TYPE_UNKNOWN,
  VC_TYPE_RGB,         // packed: bytes = R0, G0, B0, ...
  VC_TYPE_ARGB,        // packed: bytes = A0, R0, G0, B0, ...
  VC_TYPE_RGBA,        // packed: bytes = R0, G0, B0, A0, ...
  VC_TYPE_FOURCC_2uvy, // packed: bytes = Cb, Y0, Cr, Y1, ... pixels 0 & 1 share Cb, Cr
  VC_TYPE_FOURCC_yuvs, // packed: bytes = Y0, Cb, Y1, Cr, ... pixels 0 & 1 share Cb, Cr
  VC_TYPE_FOURCC_YVYU, // packed: bytes = Y1, Cr, Y0, Cb, ... pixels 0 & 1 share Cb, Cr
  VC_TYPE_FOURCC_yuvu, // packed: bytes = Y0, Cb, Y1, Cr, ... pixels 0 & 1 share Cb, Cr
};

enum vcapture_error {
  VC_SUCCESS = 0,
  VC_INVALID_DEVICE = -1,
  VC_DEVICE_OPEN_ERROR = -2,
  VC_NO_ACTIVE_DEVICE = -3,
  VC_DEVICE_RUNNING = -4
};

struct vcapture_frame {
  int count; // 0 if component/packed video, otherwise number of planes
  void* planes[VC_MAX_PLANES];
};


// zero means unspecified
// only type, fourcc, width, and height are observed in set_format
// if type and fourcc are both set, fourcc wins
struct vcapture_frame_format {
  enum vcapture_format_type type;
  unsigned int fourcc;
  int width;
  int height;
  int bytes_per_row;
  int pad_left;
  int pad_top;
  int pad_right;
  int pad_bottom;
};

typedef void (*vcapture_frame_callback)(const struct vcapture_frame_format* format,
                                        const struct vcapture_frame* frame,
                                        unsigned long long int timestamp,
                                        void* userdata);


    
#ifdef __cplusplus
extern "C" {
#endif

unsigned int vcapture_type_to_fourcc(enum vcapture_format_type type);
enum vcapture_format_type vcapture_fourcc_to_type(unsigned int fourcc);

void vcapture_poll(int msec);

vcapture vcapture_alloc();
void vcapture_free(vcapture v);

int vcapture_device_count(vcapture v);
const char* vcapture_device_name(vcapture v, int device);

int vcapture_get_device(vcapture v);
int vcapture_set_device(vcapture v, int device);

int vcapture_get_requested_format(vcapture v, struct vcapture_frame_format* f);
int vcapture_set_requested_format(vcapture v, const struct vcapture_frame_format* f);

vcapture_frame_callback vcapture_get_callback(vcapture v);
int vcapture_set_callback(vcapture v, 
                          vcapture_frame_callback callback, 
                          void* userdata);

int vcapture_active(vcapture v);
int vcapture_start(vcapture v);
int vcapture_stop(vcapture v);

#ifdef __cplusplus
}
#endif

#endif
