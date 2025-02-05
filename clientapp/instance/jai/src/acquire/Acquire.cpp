#include <Acquire.h>
#include <AppConfig.h>
#include <Logger.h>
#include <PvSampleUtils.h>

AcquireManager::AcquireManager() {
  lDecompressionFilter = new PvDecompressionFilter();
  writer = new PvBufferWriter();
}
AcquireManager::~AcquireManager() {
  free(lDecompressionFilter);
  lDecompressionFilter = nullptr;
}

void AcquireManager::AcquireImages(PvDevice* aDevice, PvStream* aStream) {
  Debug << "Preparing PvGenParam";
  PvGenParam params = parseGenParams(aDevice, aStream);
  StreamMonitor monitor;
  Debug << "Sending AcquisitionStart command to the device";
  streamExecute(params);
  Debug << "Consume Stream in while loop";
  streamConsume(aDevice, aStream, params, monitor);

  // Flush key buffer for next stop.
  Debug << "\n\n";

  // Tell the device to stop sending images.
  Debug << "Sending AcquisitionStop command to the device" << "\n";
  streamPause(params);
}

void AcquireManager::AcquireSingleImageDual(DualDevice* device) {
  PvGenParam params =
      parseGenParams(device->getDevice(0), device->getStream(0));
  streamExecute(params);
  PvBuffer* result[2] = {nullptr, nullptr};
  int error_count = 0;
  PvGetChar();
  while (!PvKbHit() && (!result[0] || !result[1]) && error_count < 5) {
    error_count++;
    PvBuffer* lBuffer = nullptr;
    PvResult lOperationResult;

    // Retrieve next buffer
    for (int i = 0; i < 2; i++) {
      Debug << "Retrieving buffer from Stream " << i;
      result[i] = AcquireBuffer(device->getStream(i));
    }
  }

  for (int i = 0; i < 2; i++) {
    if (result[i]) {
      Debug << "Saving Image";
      std::string path = "output/single_channel" + std::to_string(i) + ".png";
      Debug << "Request Writer::Store";
      auto output = writer->Store(result[i], path.c_str(), PvBufferFormatPNG);
      Debug << "Writer::Store " << output.GetDescription().GetAscii();
      Debug << "Queue Buffer";
      queueBuffer(device->getStream(i), result[i]);
    }
    Debug << "Pause Stream";
    Debug << device->getStream(i)->GetQueuedBufferCount() << " Buffers left";
    streamPause(params);
  }
}

void AcquireManager::AcquireSingleImage(PvDevice* aDevice, PvStream* aStream) {
  PvGenParam params = parseGenParams(aDevice, aStream);
  Debug << "Sending AcquisitionStart command to the device";

  streamExecute(params);
  PvBuffer* result = nullptr;
  int error_count = 0;
  PvGetChar();
  while (!PvKbHit() && !result && error_count < 5) {
    error_count++;
    result = AcquireBuffer(aStream);
  }

  if (result) {
    Debug << "Saving Image";
    std::string path = "output/single.png";
    Debug << "Request Writer::Store";
    auto output = writer->Store(result, path.c_str(), PvBufferFormatPNG);
    Debug << "Writer::Store " << output.GetDescription().GetAscii();
    Debug << "Queue Buffer";
    queueBuffer(aStream, result);
  }
  Debug << "Pause Stream";
  Debug << aStream->GetQueuedBufferCount() << " Buffers left";
  streamPause(params);
}

bool AcquireManager::queueBuffer(PvStream* aStream, PvBuffer* buffer) {
  // buffer->Free();
  PvResult queueResult = aStream->QueueBuffer(buffer);
  if (!queueResult.IsOK() && !queueResult.IsPending()) {
    Debug << "Failed to queue buffer: "
          << queueResult.GetCodeString().GetAscii();
    return false;
  }
  return true;
}

PvBuffer* AcquireManager::AcquireBuffer(PvStream* aStream) {
  PvBuffer* lBuffer = nullptr;
  PvResult lOperationResult;

  // Retrieve next buffer
  PvResult lResult = aStream->RetrieveBuffer(&lBuffer, &lOperationResult,
                                             1000 / config->FRAME_RATE);

  if (!lResult.IsOK()) {
    if (lResult.GetCode() == PvResult::Code::TOO_MANY_CONSECUTIVE_RESENDS) {
    } else if (lResult.GetCode() == PvResult::Code::TOO_MANY_RESENDS) {
    } else if (lResult.GetCode() == PvResult::Code::RESENDS_FAILURE) {
    } else {
      ErrorLog << "Failed to retrieve buffer: "
               << lResult.GetCodeString().GetAscii() << " "
               << lResult.GetCode();
      if (lBuffer) queueBuffer(aStream, lBuffer);
      return nullptr;
    }
  }
  if (!lBuffer) {
    Debug << "Buffer is null";
    return nullptr;
  }
  if (!lOperationResult.IsOK()) {
    if (lOperationResult.IsSuccess()) {
      Debug << "Operation result is Success but not OK";
    }
    if (lOperationResult.IsPending()) {
      Debug << "Operation result is Pending";
    }
    if (lOperationResult.GetCode() == PvResult::Code::BUFFER_TOO_SMALL) {
      Debug << "Buffer too small" << lBuffer->GetPayloadSize();
    }
    switch (lOperationResult.GetCode()) {
      case PvResult::Code::TOO_MANY_CONSECUTIVE_RESENDS:
      case PvResult::Code::TOO_MANY_RESENDS:
      case PvResult::Code::RESENDS_FAILURE:
        Debug << "Payload is not complete";
        break;
      default:
        ErrorLog << "Operation result is not OK : "
                 << lOperationResult.GetCodeString().GetAscii();
        break;
    }

  } else if (lBuffer->GetPayloadType() != PvPayloadTypeImage) {
    Debug << "Payload type is not Image";

  } else {
    // Debug << "Image acquired";
    return lBuffer;
  }
  lBuffer->Free();
  lBuffer->Alloc(static_cast<uint32_t>(config->BUFFER_SIZE));
  queueBuffer(aStream, lBuffer);

  return nullptr;
}

void AcquireManager::streamExecute(PvGenParam& params) {
  params.lStart->Execute();
}

void AcquireManager::streamPause(PvGenParam& params) {
  params.lStop->Execute();
}

PvGenParam AcquireManager::parseGenParams(PvDevice* aDevice,
                                          PvStream* aStream) {
  if (deviceGenParams.find(aDevice) != deviceGenParams.end()) {
    return deviceGenParams[aDevice];
  }
  PvGenParam params;
  // Get device parameters need to control streaming
  PvGenParameterArray* lDeviceParams = aDevice->GetParameters();

  // Map the GenICam AcquisitionStart and AcquisitionStop commands
  params.lStart =
      dynamic_cast<PvGenCommand*>(lDeviceParams->Get("AcquisitionStart"));
  params.lStop =
      dynamic_cast<PvGenCommand*>(lDeviceParams->Get("AcquisitionStop"));

  // Get stream parameters
  PvGenParameterArray* lStreamParams = aStream->GetParameters();

  // Map a few GenICam stream stats counters
  params.lFrameRate =
      dynamic_cast<PvGenFloat*>(lStreamParams->Get("AcquisitionRate"));
  params.lBandwidth =
      dynamic_cast<PvGenFloat*>(lStreamParams->Get("Bandwidth"));

  deviceGenParams[aDevice] = params;
  return params;
}

void AcquireManager::streamConsume(PvDevice* aDevice, PvStream* aStream,
                                   PvGenParam& params, StreamMonitor& monitor) {
  PvGetChar();
  while (!PvKbHit()) {
    bufferProcess(aDevice, aStream, params, monitor);

    ++monitor.lDoodleIndex %= 6;
  }
}

void AcquireManager::bufferProcess(PvDevice* aDevice, PvStream* aStream,
                                   PvGenParam& params, StreamMonitor& monitor) {
  PvBuffer* lBuffer = NULL;
  PvResult lOperationResult;

  // Retrieve next buffer
  PvResult lResult = aStream->RetrieveBuffer(&lBuffer, &lOperationResult, 200);

  if (!lResult.IsOK()) {
    Debug << monitor.lDoodle[monitor.lDoodleIndex] << " "
          << lResult.GetCodeString().GetAscii() << "\r";
    return;
  }

  if (lOperationResult.IsOK()) {
    payloadProcess(lBuffer, lResult, params, monitor);
  } else {
    // Non OK operational result
    Debug << monitor.lDoodle[monitor.lDoodleIndex] << " "
          << lOperationResult.GetCodeString().GetAscii() << "\r";
  }

  // Re-queue the buffer in the stream object
  aStream->QueueBuffer(lBuffer);
}

void AcquireManager::streamDestroy(PvDevice* aDevice, PvStream* aStream) {
  // Disable streaming on the device
  cout << "Disable streaming on the controller." << endl;
  aDevice->StreamDisable();

  // Abort all buffers from the stream and dequeue
  cout << "Aborting buffers still in stream" << endl;
  aStream->AbortQueuedBuffers();
  bufferDestroy(aDevice, aStream);
}

void AcquireManager::bufferDestroy(PvDevice* aDevice, PvStream* aStream) {
  while (aStream->GetQueuedBufferCount() > 0) {
    PvBuffer* lBuffer = NULL;
    PvResult lOperationResult;
    Debug << "Destroying Buffer  " << aStream->GetQueuedBufferCount()
          << " Left";
    aStream->RetrieveBuffer(&lBuffer, &lOperationResult);
  }
}

void AcquireManager::payloadImageProcess(PvBuffer* lBuffer) {
  auto image = lBuffer->GetImage();
  Debug << " Image with " << image->GetWidth() << "x" << image->GetHeight()
        << " pixels" << " and " << image->GetPixelType() << " "
        << image->GetBitsPerPixel() << " bits per pixel";

  std::string path = "output/" + std::to_string(lBuffer->GetBlockID()) + ".png";
  writer->Store(lBuffer, path.c_str(), PvBufferFormatPNG);
}

void AcquireManager::payloadChuckDataProcess(PvBuffer* lBuffer) {
  Debug << " Chunk Data payload type" << " with " << lBuffer->GetChunkCount()
        << " chunks";
}

void AcquireManager::payloadRawDataProcess(PvBuffer* lBuffer) {
  Debug << " Raw Data with " << lBuffer->GetRawData()->GetPayloadLength()
        << " bytes";
}

void AcquireManager::payloadMultipartProcess(PvBuffer* lBuffer) {
  Debug << " Multi Part with "
        << lBuffer->GetMultiPartContainer()->GetPartCount() << " parts";
}

void AcquireManager::payloadPleoraCompressedProcess(PvBuffer* lBuffer,
                                                    PvResult& lResult,
                                                    PvGenParam& params,
                                                    StreamMonitor& monitor) {
  PvPixelType lPixelType = PvPixelUndefined;
  uint32_t lWidth = 0, lHeight = 0;
  PvDecompressionFilter::GetOutputFormatFor(lBuffer, lPixelType, lWidth,
                                            lHeight);
  uint32_t lCalculatedSize =
      PvImage::GetPixelSize(lPixelType) * lWidth * lHeight / 8;

  PvBuffer lDecompressedBuffer;
  // If the buffer is compressed, start by decompressing it
  if (lDecompressionFilter->IsCompressed(lBuffer)) {
    lResult = lDecompressionFilter->Execute(lBuffer, &lDecompressedBuffer);
    if (!lResult.IsOK()) {
      return;
    }
  }

  uint32_t lDecompressedSize = lDecompressedBuffer.GetSize();
  if (lDecompressedSize != lCalculatedSize) {
    monitor.lErrors++;
  }
  double lCompressionRatio = static_cast<double>(lDecompressedSize) /
                             static_cast<double>(lBuffer->GetAcquiredSize());
  Debug << dec << " Pleora compressed.   Compression Ratio "
        << lCompressionRatio;
  Debug << " Errors: " << monitor.lErrors;
}

void AcquireManager::payloadProcess(PvBuffer* lBuffer, PvResult& lResult,
                                    PvGenParam& params,
                                    StreamMonitor& monitor) {
  params.lFrameRate->GetValue(monitor.lFrameRateVal);
  params.lBandwidth->GetValue(monitor.lBandwidthVal);

  Debug << fixed << setprecision(1);
  Debug << monitor.lDoodle[monitor.lDoodleIndex];
  Debug << " BlockID: " << uppercase << hex << setfill('0') << setw(16)
        << lBuffer->GetBlockID();

  switch (lBuffer->GetPayloadType()) {
    case PvPayloadTypeImage:
      payloadImageProcess(lBuffer);
      break;

    case PvPayloadTypeChunkData:
      payloadChuckDataProcess(lBuffer);
      break;

    case PvPayloadTypeRawData:
      payloadRawDataProcess(lBuffer);
      break;

    case PvPayloadTypeMultiPart:
      payloadMultipartProcess(lBuffer);
      break;

    case PvPayloadTypePleoraCompressed:
      payloadPleoraCompressedProcess(lBuffer, lResult, params, monitor);
      break;

    default:
      cout << " Payload type not supported by this sample";
      break;
  }
  cout << "  " << monitor.lFrameRateVal << " FPS  "
       << (monitor.lBandwidthVal / 1000000.0) << " Mb/s   \r";
}

AcquireManager* AcquireManager::getInstance() { return &instance; }
AcquireManager AcquireManager::instance;