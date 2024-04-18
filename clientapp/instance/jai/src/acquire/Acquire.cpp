#include <Acquire.h>
#include <Logger.h>
#include <PvSampleUtils.h>

AcquireManager& AcquireManager::getInstance() { return instance; }
AcquireManager AcquireManager::instance;

void AcquireManager::AcquireImages(PvDevice* aDevice, PvStream* aStream) {
  PvGenParam params = parseGenParams(aDevice, aStream);
  StreamMonitor monitor;
  streamExecute(aDevice, params);
  streamConsume(aDevice, aStream, params, monitor);
  streamDestroy(aDevice, aStream, params, monitor);
}

void AcquireManager::streamExecute(PvDevice* aDevice, PvGenParam& params) {
  params.lStart->Execute();
}

PvGenParam AcquireManager::parseGenParams(PvDevice* aDevice,
                                          PvStream* aStream) {
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
  return params;
}

void AcquireManager::streamConsume(PvDevice* aDevice, PvStream* aStream,
                                   PvGenParam& params, StreamMonitor& monitor) {
  while (!PvKbHit()) {
    PvBuffer* lBuffer = NULL;
    PvResult lOperationResult;

    // Retrieve next buffer
    PvResult lResult =
        aStream->RetrieveBuffer(&lBuffer, &lOperationResult, 1000);
    if (lResult.IsOK()) {
      if (lOperationResult.IsOK()) {
        //
        // We now have a valid buffer. This is where you would typically process
        // the buffer.
        // -----------------------------------------------------------------------------------------
        // ...
        payloadProcess(lBuffer, lResult, params, monitor);
      } else {
        // Non OK operational result
        cout << monitor.lDoodle[monitor.lDoodleIndex] << " "
             << lOperationResult.GetCodeString().GetAscii() << "\r";
      }

      // Re-queue the buffer in the stream object
      aStream->QueueBuffer(lBuffer);
    } else {
      // Retrieve buffer failure
      cout << monitor.lDoodle[monitor.lDoodleIndex] << " "
           << lResult.GetCodeString().GetAscii() << "\r";
    }

    ++monitor.lDoodleIndex %= 6;
  }
}

void AcquireManager::streamDestroy(PvDevice* aDevice, PvStream* aStream,
                                   PvGenParam& params, StreamMonitor& monitor) {
  PvGetChar();  // Flush key buffer for next stop.
  cout << endl << endl;

  // Tell the device to stop sending images.
  cout << "Sending AcquisitionStop command to the device" << endl;
  params.lStop->Execute();

  // Disable streaming on the device
  cout << "Disable streaming on the controller." << endl;
  aDevice->StreamDisable();

  // Abort all buffers from the stream and dequeue
  cout << "Aborting buffers still in stream" << endl;
  aStream->AbortQueuedBuffers();
  while (aStream->GetQueuedBufferCount() > 0) {
    PvBuffer* lBuffer = NULL;
    PvResult lOperationResult;

    aStream->RetrieveBuffer(&lBuffer, &lOperationResult);
  }
}

void AcquireManager::payloadImageProcess(PvBuffer* lBuffer) {
  Debug << "  W: " << dec << lBuffer->GetImage()->GetWidth()
        << " H: " << lBuffer->GetImage()->GetHeight();
}

void AcquireManager::payloadChuckDataProcess(PvBuffer* lBuffer) {
  Debug << " Chunk Data payload type"
        << " with " << lBuffer->GetChunkCount() << " chunks";
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
  if (lDecompressionFilter.IsCompressed(lBuffer)) {
    lResult = lDecompressionFilter.Execute(lBuffer, &lDecompressedBuffer);
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