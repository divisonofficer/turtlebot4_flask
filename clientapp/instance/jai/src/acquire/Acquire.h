#include <DualDevice.h>
#include <PvBufferWriter.h>
#include <PvDecompressionFilter.h>
#include <PvDevice.h>
#include <PvGenCommand.h>
#include <PvGenFloat.h>
#include <PvStream.h>

#include <map>

struct PvGenParam {
  PvGenCommand *lStart, *lStop;
  PvGenFloat *lFrameRate, *lBandwidth;
};

struct StreamMonitor {
  char lDoodle[7] = "|\\-|-/";
  int lDoodleIndex = 0;
  double lFrameRateVal = 0.0;
  double lBandwidthVal = 0.0;
  int lErrors = 0;
};

class AcquireManager {
 public:
  AcquireManager();
  ~AcquireManager();

  /**
   * @brief Get the AcquireManager singleton instance
   * @return AcquireManager
   */
  static AcquireManager* getInstance();

  PvBuffer* AcquireBuffer(PvStream* aStream);

  /**
   * @brief Acquire images from the device
   * @param aDevice Device to acquire images from. Obtained from DeviceManager
   * @param aStream Stream to acquire images from. Obtained from StreamManager
   *
   *
   */
  void AcquireImages(PvDevice* aDevice, PvStream* aStream);

  /**
   * @brief Execute the GenCommand to start the streaming
   * @param aDevice Device to start the streaming
   * @param params GenParams to start the streaming
   */
  void streamExecute(PvGenParam& params);

  /**
   * @brief Consume the stream data, in while loop until the user stops the
   * streaming
   * @param aDevice Device to consume the stream data
   * @param aStream Stream to consume the stream data
   * @param params GenParams to consume the stream data
   * @param monitor StreamMonitor to monitor the stream data
   */
  void streamConsume(PvDevice* aDevice, PvStream* aStream, PvGenParam& params,
                     StreamMonitor& monitor);

  void streamPause(PvGenParam& params);

  void AcquireSingleImage(PvDevice* aDevice, PvStream* aStream);

  void AcquireSingleImageDual(DualDevice* device);

  /**
   * @brief Destroy the stream data
   */
  void streamDestroy(PvDevice* aDevice, PvStream* aStream);

  /**
   * @brief Destroy the stream buffer
   */
  void bufferDestroy(PvDevice* aDevice, PvStream* aStream);
  /**
   * @brief DecompressionFilter instance
   * Used to decompress payload data which is compressed
   * Used to validate the payload data is compressed or not
   */
  PvDecompressionFilter* lDecompressionFilter;
  bool queueBuffer(PvStream* aStream, PvBuffer* aBuffer);

 private:
  PvBufferWriter* writer;

  std::map<PvDevice*, PvGenParam> deviceGenParams;
  static AcquireManager instance;

  /**
   * @brief Parse the GenCommand and GenFloat parameters
   * @param aDevice Device to parse the GenParams
   * @param aStream Stream to parse the GenParams
   */
  PvGenParam parseGenParams(PvDevice* aDevice, PvStream* aStream);

  void bufferProcess(PvDevice* aDevice, PvStream* aStream, PvGenParam& params,
                     StreamMonitor& monitor);

  void bufferProcess(PvStream* aStream, PvBuffer*& buffer);

  /**
   * @brief Process the payload data
   * @param lBuffer Buffer to process
   * @param lResult Result of the buffer
   * @param params GenParams to process the buffer
   * @param monitor StreamMonitor to monitor the stream data
   */
  void payloadProcess(PvBuffer* lBuffer, PvResult& lResult, PvGenParam& params,
                      StreamMonitor& monitor);
  /**
   * @brief Process the image payload data
   */
  void payloadImageProcess(PvBuffer* lBuffer);
  /**
   * @brief Process the chunk data payload
   */
  void payloadChuckDataProcess(PvBuffer* lBuffer);
  /**
   * @brief Process the raw data payload
   */
  void payloadRawDataProcess(PvBuffer* lBuffer);
  /**
   * @brief Process the multipart data payload
   */
  void payloadMultipartProcess(PvBuffer* lBuffer);
  /**
   * @brief Process the pleora compressed data payload
   */
  void payloadPleoraCompressedProcess(PvBuffer* lBuffer, PvResult& lResult,
                                      PvGenParam& params,
                                      StreamMonitor& monitor);
};