#include <PvBuffer.h>
#include <PvDevice.h>
#include <PvStream.h>

#include <vector>
class StreamManager {
 public:
  /**
   * @brief Open a stream
   * @param aConnectionID Connection ID of the device to open the stream
   */
  PvStream *OpenStream(const PvString &aConnectionID);
  /**
   * @brief Configure the stream
   * @param aDevice Device to configure the stream
   * @param aStream Stream to configure
   */
  void ConfigureStream(PvDevice *aDevice, PvStream *aStream, u_int32_t channel);
  /**
   * @brief Create stream buffers
   * @param aDevice Device to create the stream buffers
   * @param aStream Stream to create the buffers
   * @param aBufferList List of buffers to create
   */
  void CreateStreamBuffers(PvDevice *aDevice, PvStream *aStream,
                           std::vector<PvBuffer *> *aBufferList);
  /**
   * @brief Free the stream buffers
   * @param aBufferList List of buffers to free
   */
  void FreeStreamBuffers(std::vector<PvBuffer *> *aBufferList);
  static StreamManager *getInstance();

 private:
  static StreamManager staticInstance;
};
