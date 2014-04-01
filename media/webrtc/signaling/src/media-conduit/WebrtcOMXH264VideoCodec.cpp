/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this file,
 * You can obtain one at http://mozilla.org/MPL/2.0/. */

#include "CSFLog.h"

#include "WebrtcOMXH264VideoCodec.h"

// Android/Stagefright
#include <avc_utils.h>
#include <binder/ProcessState.h>
#include <foundation/ABuffer.h>
#include <foundation/AMessage.h>
#include <gui/Surface.h>
#include <media/ICrypto.h>
#include <MediaCodec.h>
#include <MediaDefs.h>
#include <MediaErrors.h>
#include <MetaData.h>
#include <OMX_Component.h>
using namespace android;

// WebRTC
#include "video_engine/include/vie_external_codec.h"
#include "common_video/interface/texture_video_frame.h"

// Gecko
#include "GonkNativeWindow.h"
#include "GonkNativeWindowClient.h"
#include "mozilla/Atomics.h"
#include "mozilla/Monitor.h"
#include "mozilla/Scoped.h"
#include "OMXCodecWrapper.h"

#define DEQUEUE_BUFFER_TIMEOUT_US (100 * 1000ll) // 100ms
#define START_DEQUEUE_BUFFER_TIMEOUT_US (10 * DEQUEUE_BUFFER_TIMEOUT_US) // 1s

#define LOG_TAG "WebrtcOMXH264VideoCodec"
#define CODEC_LOGV(...) CSFLogInfo(LOG_TAG, __VA_ARGS__)
#define CODEC_LOGD(...) CSFLogDebug(LOG_TAG, __VA_ARGS__)
#define CODEC_LOGI(...) CSFLogInfo(LOG_TAG, __VA_ARGS__)
#define CODEC_LOGW(...) CSFLogWarn(LOG_TAG, __VA_ARGS__)
#define CODEC_LOGE(...) CSFLogError(LOG_TAG, __VA_ARGS__)

namespace mozilla {

// Link to Stagefright
class WebrtcOMX MOZ_FINAL
  : public AtomicRefCounted<WebrtcOMX>
  , public GonkNativeWindowNewFrameCallback
{
public:
  MOZ_DECLARE_REFCOUNTED_TYPENAME(WebrtcOMX)

  WebrtcOMX(const char* mime, bool encoder)
  {
    android::ProcessState::self()->startThreadPool();

    mLooper = new ALooper;
    mLooper->start();
    mCodec = MediaCodec::CreateByType(mLooper, mime, encoder);
  }

  virtual ~WebrtcOMX()
  {
    mCodec->release();
    mCodec.clear();
    mLooper.clear();
  }

  status_t Configure(const sp<AMessage>& format,
    const sp<Surface>& nativeWindow,
    const sp<ICrypto>& crypto, uint32_t flags)
  {
    return mCodec->configure(format, nativeWindow, crypto, flags);
  }

  status_t Start()
  {
    status_t err = mCodec->start();
    mStarted = err == OK;

    mCodec->getInputBuffers(&mInputBuffers);
    mCodec->getOutputBuffers(&mOutputBuffers);

    return err;
  }

  status_t Stop()
  {
    status_t err = mCodec->stop();
    mStarted = err == OK;
    return err;
  }

  // GonkNativeWindowNewFrameCallback.
  virtual void OnNewFrame()
  {
    // TODO: Get decoded frame buffer through native window to obsolete
    //       changes to stagefright code.
    CODEC_LOGD("GonkNativeWindow has new frame.");
  }

  sp<ALooper> mLooper;
  sp<MediaCodec> mCodec; // OMXCodec
  sp<AMessage> mConfig;
  int mWidth;
  int mHeight;
  bool mStarted;
  android::Vector<sp<ABuffer> > mInputBuffers;
  android::Vector<sp<ABuffer> > mOutputBuffers;

  sp<GonkNativeWindow> mNativeWindow;
  sp<GonkNativeWindowClient> mNativeWindowClient;
};

// Graphic buffer lifecycle management.
// Return buffer to OMX codec when renderer is done with it.
class VideoGraphicBuffer MOZ_FINAL : public layers::GraphicBufferLocked
{
public:
    VideoGraphicBuffer(const sp<MediaCodec>& aOmx, uint32_t bufferIndex,
                       layers::SurfaceDescriptor& aDescriptor)
      : layers::GraphicBufferLocked(aDescriptor)
      , mMonitor("VideoGraphicBuffer")
      , mOmx(aOmx)
      , mBufferIndex(bufferIndex)
    {}

    virtual ~VideoGraphicBuffer()
    {
      CODEC_LOGD("buffer destruct this:%p omx:%p", this, mOmx.get());
      MonitorAutoLock lock(mMonitor);
      if (mOmx.get()) {
        mOmx->releaseOutputBuffer(mBufferIndex);
        mOmx = nullptr;
      }
    }

    void Unlock()
    {
      CODEC_LOGD("buffer unlock this:%p omx:%p", this, mOmx.get());
      MonitorAutoLock lock(mMonitor);
      if (mOmx.get()) {
        mOmx->releaseOutputBuffer(mBufferIndex);
        mOmx = nullptr;
      }
    }

private:
  sp<MediaCodec> mOmx;
  uint32_t mBufferIndex;
  Monitor mMonitor;
};

// This class wraps VideoGraphicBuffer to pass frames through WebRTC rendering
// pipeline using TextureVideoFrame.
class NativeHandle MOZ_FINAL : public webrtc::NativeHandle
{
public:
  NativeHandle(layers::Image* aImage)
    : mImage(aImage)
    , mRefCnt(0)
  {}

  void* GetHandle() { return mImage.get(); }

  int AddRef()
  {
    MOZ_ASSERT(mRefCnt >= 0);
    int cnt = ++mRefCnt;
    return cnt;
  }

  int Release()
  {
    MOZ_ASSERT(mRefCnt > 0);
    void* that = this;
    void* handle = mImage.get();
    int cnt = --mRefCnt;
    if (cnt == 0) {
      delete this;
    }
    return cnt;
  }

private:
  RefPtr<layers::Image> mImage;
  Atomic<int> mRefCnt;
};

// Encoder.
WebrtcOMXH264VideoEncoder::WebrtcOMXH264VideoEncoder()
  : mCallback(nullptr)
  , mMutex("WebrtcOMXH264VideoEncoder")
  , mOMX(nullptr)
  , mOMXConfigured(false)
{
  memset(&mEncodedImage, 0, sizeof(mEncodedImage));
  CODEC_LOGD("this:%p constructed", this);
}

static
AMessage*
VideoCodecSettings2AMessage(const webrtc::VideoCodec* codecSettings)
{
  AMessage* format = new AMessage;

  format->setString("mime", MEDIA_MIMETYPE_VIDEO_AVC);
  format->setInt32("store-metadata-in-buffers", 0);
  format->setInt32("prepend-sps-pps-to-idr-frames", 0);
  format->setInt32("bitrate", codecSettings->minBitrate * 1000); // kbps->bps
  format->setInt32("width", codecSettings->width);
  format->setInt32("height", codecSettings->height);
  format->setInt32("stride", codecSettings->width);
  format->setInt32("slice-height", codecSettings->height);
  format->setFloat("frame-rate", (float)codecSettings->maxFramerate);
  // QCOM encoder only support this format. See
  // <B2G>/hardware/qcom/media/mm-video/vidc/venc/src/video_encoder_device.cpp:
  // venc_dev::venc_set_color_format()
  format->setInt32("color-format", OMX_COLOR_FormatYUV420SemiPlanar);
  // FIXME: get correct parameters from codecSettings?
  format->setInt32("i-frame-interval", 1); // one I-frame per sec
  format->setInt32("profile", OMX_VIDEO_AVCProfileBaseline);
  format->setInt32("level", OMX_VIDEO_AVCLevel3);
  //format->setInt32("bitrate-mode", OMX_Video_ControlRateConstant);

  return format;
}

int32_t
WebrtcOMXH264VideoEncoder::InitEncode(const webrtc::VideoCodec* codecSettings,
                                      int32_t numberOfCores,
                                      uint32_t maxPayloadSize)
{
  mMaxPayloadSize = maxPayloadSize;
  CODEC_LOGD("this:%p init", this);

  if (!mOMX) {
    mOMX = OMXCodecWrapper::CreateAVCEncoder();
  }
  mConfig.mWidth = codecSettings->width;
  mConfig.mHeight = codecSettings->height;
  mConfig.mFrameRate = codecSettings->maxFramerate;

  MutexAutoLock lock(mMutex);

  // TODO: eliminate extra pixel copy & color conversion
  size_t size = codecSettings->width * codecSettings->height * 3 / 2;
  if (mEncodedImage._size < size) {
    if (mEncodedImage._buffer) {
      delete [] mEncodedImage._buffer;
    }
    mEncodedImage._buffer = new uint8_t[size];
    mEncodedImage._size = size;
  }

  return WEBRTC_VIDEO_CODEC_OK;
}

int32_t
WebrtcOMXH264VideoEncoder::Encode(
    const webrtc::I420VideoFrame& inputImage,
    const webrtc::CodecSpecificInfo* codecSpecificInfo,
    const std::vector<webrtc::VideoFrameType>* frame_types)
{
  CODEC_LOGD("this:%p will encode", this);
  if (mOMX == nullptr) {
    return WEBRTC_VIDEO_CODEC_ERROR;
  }

  uint32_t time = PR_IntervalNow();

  if (!mOMXConfigured) {
    mOMX->Configure(mConfig.mWidth, mConfig.mHeight,
                    mConfig.mFrameRate, true /* NAL */);
    mOMXConfigured = true;
    CODEC_LOGD("this:%p encode start OMX took %u ms", this,
      PR_IntervalToMilliseconds(PR_IntervalNow()-time));
    time = PR_IntervalNow();
  }

  layers::PlanarYCbCrImage* img = new layers::PlanarYCbCrImage(nullptr);
  layers::PlanarYCbCrData yuvData;
  yuvData.mYChannel = const_cast<uint8_t*>(inputImage.buffer(webrtc::kYPlane));
  yuvData.mYSize = gfx::IntSize(inputImage.width(), inputImage.height());
  yuvData.mYStride = inputImage.stride(webrtc::kYPlane);
  MOZ_ASSERT(inputImage.stride(webrtc::kUPlane) == inputImage.stride(webrtc::kVPlane));
  yuvData.mCbCrStride = inputImage.stride(webrtc::kUPlane);
  yuvData.mCbChannel = const_cast<uint8_t*>(inputImage.buffer(webrtc::kUPlane));
  yuvData.mCrChannel = const_cast<uint8_t*>(inputImage.buffer(webrtc::kVPlane));
  yuvData.mCbCrSize = gfx::IntSize(yuvData.mYSize.width / 2, yuvData.mYSize.height / 2);
  yuvData.mPicSize = yuvData.mYSize;
  yuvData.mStereoMode = StereoMode::MONO;
  img->SetDataNoCopy(yuvData);

  nsresult rv = mOMX->Encode(img,
                             yuvData.mYSize.width,
                             yuvData.mYSize.height,
                             inputImage.render_time_ms() * 1000, // ms to us
                             0);
  NS_ENSURE_SUCCESS(rv, WEBRTC_VIDEO_CODEC_ERROR);

  CODEC_LOGD("this:%p encode took %u ms", this,
             PR_IntervalToMilliseconds(PR_IntervalNow()-time));
  time = PR_IntervalNow();

  nsTArray<uint8_t> output;
  int64_t timeUs;
  int flags;
  rv = mOMX->GetNextEncodedFrame(&output, &timeUs, &flags, 5000);
  NS_ENSURE_SUCCESS(rv, WEBRTC_VIDEO_CODEC_ERROR);
  if (output.Length() == 0) {
    CODEC_LOGD("this:%p encode no output available this time", this);
    // No encoded data yet. Try later.
    return WEBRTC_VIDEO_CODEC_OK;
  }

  EncodedFrame frame;
  frame.mWidth = inputImage.width();
  frame.mHeight = inputImage.height();
  frame.mTimestamp = inputImage.timestamp();

  CODEC_LOGD("this:%p encode image took %u ms", this,
    PR_IntervalToMilliseconds(PR_IntervalNow()-time));
  time = PR_IntervalNow();

  mEncodedImage._encodedWidth = frame.mWidth;
  mEncodedImage._encodedHeight = frame.mHeight;
  mEncodedImage._timeStamp = frame.mTimestamp;
  mEncodedImage.capture_time_ms_ = inputImage.render_time_ms();

  CODEC_LOGD("this:%p encode dequeue OMX output buffer len:%u time:%lld flags:0x%08x took %u ms",
             this, output.Length(), timeUs, flags, PR_IntervalToMilliseconds(PR_IntervalNow()-time));

  MutexAutoLock lock(mMutex);

  mEncodedImage._length = output.Length();
  memcpy(mEncodedImage._buffer, output.Elements(), mEncodedImage._length);
  mEncodedImage._completeFrame = true;
  mEncodedImage._frameType =
    (flags & (MediaCodec::BUFFER_FLAG_SYNCFRAME | MediaCodec::BUFFER_FLAG_CODECCONFIG))?
    webrtc::kKeyFrame:webrtc::kDeltaFrame;

  CODEC_LOGD("this:%p encode frame type:%d size:%u", this, mEncodedImage._frameType, mEncodedImage._length);
  mCallback->Encoded(mEncodedImage, nullptr, nullptr);

  return WEBRTC_VIDEO_CODEC_OK;
}

int32_t
WebrtcOMXH264VideoEncoder::RegisterEncodeCompleteCallback(
    webrtc::EncodedImageCallback* callback)
{
  mCallback = callback;

  return WEBRTC_VIDEO_CODEC_OK;
}

int32_t
WebrtcOMXH264VideoEncoder::Release()
{
  CODEC_LOGD("this:%p will be released", this);
  if (mOMX) {
    delete mOMX;
    mOMX = nullptr;
  }

  MutexAutoLock lock(mMutex);
  if (mEncodedImage._buffer) {
    delete [] mEncodedImage._buffer;
    mEncodedImage._buffer = nullptr;
    mEncodedImage._size = 0;
  }

  return WEBRTC_VIDEO_CODEC_OK;
}

WebrtcOMXH264VideoEncoder::~WebrtcOMXH264VideoEncoder()
{
  CODEC_LOGD("this:%p will be destructed", this);
  Release();
}

// TODO
int32_t
WebrtcOMXH264VideoEncoder::SetChannelParameters(uint32_t packetLoss, int rtt)
{
  return WEBRTC_VIDEO_CODEC_OK;
}

// Note: stagefright doesn't handle frame rate change.
int32_t
WebrtcOMXH264VideoEncoder::SetRates(uint32_t newBitRate, uint32_t frameRate)
{
  CODEC_LOGD("this:%p set bitrate:%u, frame rate:%u)", this, newBitRate, frameRate);
  if (!mOMX) {
    return WEBRTC_VIDEO_CODEC_UNINITIALIZED;
  }
  mOMX->SetBitrate(newBitRate);

  return WEBRTC_VIDEO_CODEC_OK;
}

// Decoder.
WebrtcOMXH264VideoDecoder::WebrtcOMXH264VideoDecoder()
  : mCallback(nullptr)
  , mOMX(nullptr)
{
  CODEC_LOGD("this:%p will be constructed", this);
}

int32_t
WebrtcOMXH264VideoDecoder::InitDecode(const webrtc::VideoCodec* codecSettings,
                                      int32_t numberOfCores)
{
  CODEC_LOGD("this:%p init decoder:%p", this, mOMX);
  if (!mOMX) {
    mOMX = new WebrtcOMX(MEDIA_MIMETYPE_VIDEO_AVC, false /* encoder */);
    mOMX->mConfig = VideoCodecSettings2AMessage(codecSettings);
    mOMX->mNativeWindow = new GonkNativeWindow();
    mOMX->mNativeWindow->setNewFrameCallback(mOMX);
    mOMX->mNativeWindowClient =
        new GonkNativeWindowClient(mOMX->mNativeWindow->getBufferQueue());
  }
  mOMX->mStarted = false;

  return WEBRTC_VIDEO_CODEC_OK;
}

static
status_t
feedOMXInput(WebrtcOMX* decoder, const sp<MediaCodec>& omx,
             const webrtc::EncodedImage& inputImage, int64_t* timeUs)
{
  static int64_t firstTime = -1ll;
  size_t index;
  uint32_t time = PR_IntervalNow();
  status_t err = omx->dequeueInputBuffer(&index,
    firstTime < 0 ? START_DEQUEUE_BUFFER_TIMEOUT_US : DEQUEUE_BUFFER_TIMEOUT_US);
  if (err != OK) {
    CODEC_LOGE("decode dequeue input buffer error:%d", err);
    return err;
  }
  CODEC_LOGD("decode dequeue input buffer took %u ms",
    PR_IntervalToMilliseconds(PR_IntervalNow()-time));
  time = PR_IntervalNow();

  uint32_t flags = 0;
  if (inputImage._frameType == webrtc::kKeyFrame) {
    flags |= (firstTime < 0)? MediaCodec::BUFFER_FLAG_CODECCONFIG:MediaCodec::BUFFER_FLAG_SYNCFRAME;
  }
  size_t size = inputImage._length;
  const sp<ABuffer>& omxIn = decoder->mInputBuffers.itemAt(index);
  MOZ_ASSERT(omxIn->capacity() >= size);
  omxIn->setRange(0, size);
  memcpy(omxIn->data(), inputImage._buffer, size);
  if (firstTime < 0) {
    firstTime = inputImage._timeStamp;
  }
  *timeUs = (inputImage._timeStamp - firstTime) * 10; // FIXME: use time of input image.
  err = omx->queueInputBuffer(index, 0, size, *timeUs, flags);

  CODEC_LOGD("decode queue input buffer len:%u flags:%u time:%lld took %u ms",
    size, flags, *timeUs, PR_IntervalToMilliseconds(PR_IntervalNow()-time));

  return err;
}

static
webrtc::I420VideoFrame*
GenerateVideoFrame(GonkNativeWindow* nativeWindow, EncodedFrame* frame,
                   const sp<MediaCodec>& omx, uint32_t bufferIndex,
                   const sp<ABuffer>& decoded, int64_t renderTimeMs)
{
  // TODO: Get decoded frame buffer through native window to obsolete
  //       changes to stagefright code.
  sp<RefBase> obj;
  bool hasGraphicBuffer = decoded->meta()->findObject("graphic-buffer", &obj);
  MOZ_ASSERT(hasGraphicBuffer);
  if (!hasGraphicBuffer) {
    // Nothing to render.
    return nullptr;
  }

  sp<GraphicBuffer> gb = static_cast<GraphicBuffer*>(obj.get());
  MOZ_ASSERT(gb.get());
  CODEC_LOGD("decode omx buffer:%p has graphic buffer:%p",
    decoded->data(), hasGraphicBuffer ? gb.get() : nullptr);
  if (!gb.get()) {
    return nullptr;
  }

  layers::SurfaceDescriptor *descriptor = nativeWindow->getSurfaceDescriptorFromBuffer(gb.get());

  // Change the descriptor's size to video's size. There are cases that
  // GraphicBuffer's size and actual video size is different.
  // See Bug 850566.
  layers::SurfaceDescriptorGralloc newDescriptor = descriptor->get_SurfaceDescriptorGralloc();
  newDescriptor.size() = gfx::IntSize(frame->mWidth, frame->mHeight);

  layers::SurfaceDescriptor descWrapper(newDescriptor);
  VideoGraphicBuffer* vgb = new VideoGraphicBuffer(omx, bufferIndex, descWrapper);

  layers::GrallocImage::GrallocData grallocData;
  grallocData.mPicSize = gfx::IntSize(frame->mWidth, frame->mHeight);
  grallocData.mGraphicBuffer = vgb;

  layers::GrallocImage* grallocImage = new layers::GrallocImage();
  grallocImage->SetData(grallocData);

  webrtc::TextureVideoFrame* videoFrame =
    new webrtc::TextureVideoFrame(new NativeHandle(grallocImage),
                                  frame->mWidth, frame->mHeight,
                                  frame->mTimestamp, renderTimeMs);
  CODEC_LOGD("omx graphic buffer:%p(%p) image:%p frame:%p handle:%p(%p)",
    vgb, gb.get(), grallocImage, videoFrame, videoFrame->native_handle(),
    static_cast<NativeHandle*>(videoFrame->native_handle())->GetHandle());

  return videoFrame;
}

static
status_t
getOMXOutput(WebrtcOMX* decoder, const sp<MediaCodec>& omx,
             const webrtc::EncodedImage& inputImage, const int64_t timeUs,
             int64_t renderTimeMs, webrtc::DecodedImageCallback* callback)
{
  sp<ABuffer> omxOut;

  EncodedFrame* frame = new EncodedFrame();
  frame->mWidth = decoder->mWidth;
  frame->mHeight = decoder->mHeight;
  frame->mTimestamp = inputImage._timeStamp;

  uint32_t time = PR_IntervalNow();

  size_t index;
  size_t outOffset;
  size_t outSize;
  int64_t outTime;
  uint32_t outFlags;
  status_t err = omx->dequeueOutputBuffer(&index, &outOffset, &outSize, &outTime, &outFlags);
  if (err == INFO_FORMAT_CHANGED) {
    // TODO: handle format change
    goto end;
  } else if (err == INFO_OUTPUT_BUFFERS_CHANGED) {
    CODEC_LOGD("decode dequeue OMX output buffer change:%d", err);
    err = omx->getOutputBuffers(&decoder->mOutputBuffers);
    MOZ_ASSERT(err == OK);
    err = INFO_OUTPUT_BUFFERS_CHANGED;
    goto end;
  } else if (err != OK) {
    CODEC_LOGE("decode dequeue OMX output buffer error:%d", err);
    goto end;
  }
  omxOut = decoder->mOutputBuffers.itemAt(index);

  CODEC_LOGD("decode dequeue output buffer#%u(%p) err:%d len:%u time:%lld flags:0x%08x took %u ms",
    index, omxOut.get(), err, outSize, outTime, outFlags, PR_IntervalToMilliseconds(PR_IntervalNow()-time));
  time = PR_IntervalNow();

  if (timeUs < outTime) {
    // Buffer out of date.
    omx->releaseOutputBuffer(index);
  } else {
    webrtc::I420VideoFrame* video_frame = GenerateVideoFrame(
      decoder->mNativeWindow.get(), frame, omx, index, omxOut, renderTimeMs);
    if (!video_frame) {
      omx->releaseOutputBuffer(index);
    }
    callback->Decoded(*video_frame);
    delete video_frame;
    // OMX buffer will be released after renderer is done with it.
  }

end:
  delete frame;

  return err;
}

int32_t
WebrtcOMXH264VideoDecoder::Decode(const webrtc::EncodedImage& inputImage,
                                  bool missingFrames,
                                  const webrtc::RTPFragmentationHeader* fragmentation,
                                  const webrtc::CodecSpecificInfo* codecSpecificInfo,
                                  int64_t renderTimeMs)
{
  CODEC_LOGD("this:%p will decode", this);

  if (inputImage._length== 0 || !inputImage._buffer) {
    return WEBRTC_VIDEO_CODEC_ERROR;
  }

  uint32_t time = PR_IntervalNow();
  if (!mOMX->mStarted) {
    // Get image width/height from CSD.
    sp<ABuffer> csd = new ABuffer(inputImage._buffer, inputImage._length);
    sp<MetaData> avc = MakeAVCCodecSpecificData(csd);
    int32_t width = 0;
    bool ok = avc->findInt32(kKeyWidth, &width);
    MOZ_ASSERT(ok && width > 0);
    int32_t height = 0;
    ok = avc->findInt32(kKeyHeight, &height);
    MOZ_ASSERT(ok && height > 0);
    CODEC_LOGD("this:%p decoder config width:%d height:%d", this, width, height);
    mOMX->mConfig->setInt32("width", width);
    mOMX->mConfig->setInt32("height", height);
    mOMX->mConfig->setInt32("stride", width);
    mOMX->mConfig->setInt32("slice-height", height);
    mOMX->mWidth = width;
    mOMX->mHeight = height;
    sp<Surface> nativeWindow = nullptr;
    if (mOMX->mNativeWindowClient.get()) {
      nativeWindow = new Surface(mOMX->mNativeWindowClient->getIGraphicBufferProducer());
    }
    mOMX->Configure(mOMX->mConfig, nativeWindow, nullptr, 0);
    mOMX->Start();
    CODEC_LOGD("this:%p start decoder took %u ms", this,
      PR_IntervalToMilliseconds(PR_IntervalNow()-time));
  }

  sp<MediaCodec> omx = mOMX->mCodec;
  bool feedFrame = true;

  while (feedFrame) {
    int64_t timeUs;
    status_t err = feedOMXInput(mOMX, omx, inputImage, &timeUs);
    feedFrame = (err == -EAGAIN);
    do {
      err = getOMXOutput(mOMX, omx, inputImage, timeUs, renderTimeMs, mCallback);
    } while (err == INFO_OUTPUT_BUFFERS_CHANGED);
  }

  CODEC_LOGD("this:%p decode done", this);

  return WEBRTC_VIDEO_CODEC_OK;
}

int32_t
WebrtcOMXH264VideoDecoder::RegisterDecodeCompleteCallback(
    webrtc::DecodedImageCallback* callback)
{
  mCallback = callback;

  return WEBRTC_VIDEO_CODEC_OK;
}

int32_t
WebrtcOMXH264VideoDecoder::Release()
{
  CODEC_LOGD("this:%p will be released", this);

  if (mOMX) {
    delete mOMX;
    mOMX = nullptr;
  }

  return WEBRTC_VIDEO_CODEC_OK;
}

WebrtcOMXH264VideoDecoder::~WebrtcOMXH264VideoDecoder()
{
  CODEC_LOGD("this:%p will be destructed", this);
  Release();
}

int32_t
WebrtcOMXH264VideoDecoder::Reset()
{
  return WEBRTC_VIDEO_CODEC_OK;
}

}
