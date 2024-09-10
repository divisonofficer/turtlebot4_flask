#include <PvPixelType.h>

#define DEVICE_LEFT_NAME "jai_1600_left"
#define DEVICE_RIGHT_NAME "jai_1600_right"
#define DEVICE_LEFT_ADDRESS "00:0c:df:0a:b9:62"
#define DEVICE_RIGHT_ADDRESS "00:0c:df:0a:c6:c8"
#define BUFFER_COUNT (128)
#define BUFFER_SIZE (1440 * 1080)
#define ROS_MSG_BUFFER_SIZE (720 * 540)  // half size : 720 540
#define ROS_SCALE_DOWN true
#define VIZ_PIXEL_ACQUIRE_FORMAT PvPixelBayerRG8
#define NIR_PIXEL_ACQUIRE_FORMAT PvPixelMono8

#define FRAME_RATE 12.0f

#define TRIGGER_SYNC true
#define MULTIFRAME_COUNT 50
#define HDR_CAPTURE_MODE false
#define STEREO_EXPOSURE_SYNC true
#define MIN_DIFFS 1