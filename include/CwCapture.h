#ifndef CW_V4L2_H__
#define CW_V4L2_H__

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

#include <stdint.h>
#define UYVY_BYTES_PER_PIXEL 2U

struct CaptureData {
  uint8_t * imgbuf;
  uint64_t usec;
};

struct CaptureDevice {
  int32_t fd;
  int32_t buf_type; // V4L2_BUF_TYPE_VIDEO_CAPTURE (UserFixed)
  int32_t mem_type; // V4L2_MEMORY_USERPTR (UserFixed)

  uint8_t v_idx;    // video node index.
  uint8_t f_idx;    // selected format option index. 
  uint8_t debug;    // verbose or not (not used now)
  uint8_t reserved; // reserverd. (not used now)

  uint32_t pixel_format; // UYVY Format. (StrongAssumption)
  uint32_t width;
  uint32_t height;
  uint32_t img_bytes;

  char node_name[16]; // /dev/video[x]
  struct v4l2_buffer * buffer;
  struct CaptureData data;
};

// Initialize CaptureDevice instance for /dev/video[v_idx] node.
// At first time, cam has no fixed format option, so cannot caputre image.
int initialize(struct CaptureDevice * cam, int v_idx);

// List availble format options
// After check, call set_option() with the option you want.
int list_options(struct CaptureDevice * cam);

// Set fixed device format with f_idx-th format option.
int set_option(struct CaptureDevice * cam, int f_idx);

// Prepare user pointer buffer for image capture.
// Don't forget to call free_buffer at the end of the app.
int alloc_buffer(struct CaptureDevice * cam);

// Free dyamically allocated memory in cam
int free_buffer(struct CaptureDevice * cam);

// Start streaming at ${cam} device
int stream_on(struct CaptureDevice * cam);

// Stop streaming at ${cam} device
int stream_off(struct CaptureDevice * cam);

// Capture image and fill data in cam->data.
// Also, return rgb888 format image through img pointer.
// img must be allocated beforehand. 
int capture(struct CaptureDevice * cam);

int convert(struct CaptureDevice * cam, uint8_t * img);

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // CW_V4L2_H__
