#pragma once

#include <PvBuffer.h>
#include <PvDevice.h>
#include <PvDeviceGEV.h>
#include <PvStream.h>
#include <PvStreamGEV.h>

#include <vector>

class DualDevice {
 public:
  DualDevice(PvString &connection_ID);

  PvDevice *getDevice(int source);
  PvStream *getStream(int source);
  std::vector<PvBuffer *> *getBufferList(int source);

  /**
   * Configure camera Sequencer Mode for HDR burst capture.
   * Sets up exposure sets based on config->HDR_EXPOSURE values.
   * Only active when config->ENABLE_SEQUENCER_MODE is true.
   */
  void configureSequencer();

 private:
  PvDeviceGEV *rgb_device;
  PvStreamGEV *rgb_stream;
  PvStreamGEV *nir_stream;
  std::vector<PvBuffer *> rgb_buffer_list;
  std::vector<PvBuffer *> nir_buffer_list;
};