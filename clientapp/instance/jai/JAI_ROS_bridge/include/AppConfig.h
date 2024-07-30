#include <PvPixelType.h>

#define DEVICE_LEFT_NAME "jai_1600_left"
#define DEVICE_RIGHT_NAME "jai_1600_right"
#define DEVICE_LEFT_ADDRESS "00:0c:df:0a:b9:62"
#define DEVICE_RIGHT_ADDRESS "00:0c:df:0a:c6:c8"
#define BUFFER_COUNT (128)
#define BUFFER_SIZE (1440 * 1080)

#define VIZ_PIXEL_ACQUIRE_FORMAT PvPixelBayerRG8
#define NIR_PIXEL_ACQUIRE_FORMAT PvPixelMono8

#define FRAME_RATE 5.0f

#define TRIGGER_SYNC true
#define MULTIFRAME_COUNT 5
#define HDR_CAPTURE_MODE false
