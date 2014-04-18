/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this file,
 * You can obtain one at http://mozilla.org/MPL/2.0/. */

#include <iostream>
#include <string>
#include <sys/time.h>
#include <sys/resource.h>

#ifdef HAVE_GETOPT_H
#include <getopt.h>
#endif


#include "mozilla/Atomics.h"
#include "mozilla/Monitor.h"
#include "mozilla/Scoped.h"
#include "nsIEventTarget.h"
#include "nsIPrefService.h"
#include "nsServiceManagerUtils.h"
#include <MediaConduitInterface.h>
#include "FakeMediaStreamsImpl.h"

#include "mtransport_test_utils.h"
MtransportTestUtils *test_utils;

std::string g_input_file;
int g_width = 640;
int g_height = 480;
int g_frame_rate = 30;
int g_total_frames = -1;
bool g_send_and_receive = false;

uint64_t timeval2int(const struct timeval &tv) {
  uint64_t ret = tv.tv_sec;
  ret *= 1000000;
  ret += tv.tv_usec;

  return ret;
}

uint64_t time64() {
  struct timeval tv;

  gettimeofday(&tv, NULL);
  return timeval2int(tv);
}

void getrtimes(uint64_t *utime, uint64_t *stime) {
  struct rusage ru;

  int r = getrusage(RUSAGE_SELF, &ru);
  if (r < 0) {
    exit(1);
  }

  *utime = timeval2int(ru.ru_utime);
  *stime = timeval2int(ru.ru_stime);
}

namespace mozilla {

class Benchmark;

void usage() {
  std::cerr << "Usage: video_benchmark [-f input-file]" << std::endl;
  exit(1);
}


class YuvReader {
 public:
  YuvReader()
  : input_fp_(nullptr),
    init_(false),
    height_(0),
    width_(0),
    frame_size_(0),
    frame_(nullptr),
    loop_(false) {}
  ~YuvReader() {
    if (input_fp_)
      fclose(input_fp_);
  }

  bool Init(const std::string& input_file, bool loop) {
    std::cerr << "Initializing YuvReader with file " << input_file
	      << " loop=" << loop << std::endl;
    loop_ = loop;
    input_fp_ = fopen(input_file.c_str(), "r");
    if (!input_fp_)
      return false;

    char line[1024];
    if (!fgets(line, sizeof(line), input_fp_))
      return false;

    if (sscanf(line, "YUV4MPEG2 W%d H%d", &width_, &height_) != 2)
      return false;

    if (width_ <= 0 && width_ > 2000)
      return false;

    if (height_ <= 0 && height_ > 2000)
      return false;

    frame_size_ = (width_ * height_ * 3) / 2;
    frame_ = new unsigned char[frame_size_];
    init_ = true;

    return true;
  }

  bool ReadFrame() {
    int retry_ct = loop_ ? 2 : 1;

    while (retry_ct > 0) {
      --retry_ct;
      char frame_hdr[1024];
      if (fgets(frame_hdr, sizeof(frame_hdr), input_fp_)) {
        if (strncmp(frame_hdr, "FRAME", 5)) {
          std::cerr << "Bogus data" << std::endl;
          return false;
        }

        int r = fread(frame_, 1, frame_size_, input_fp_);
        if (r == frame_size_)
          return true;
      }

      // Loop
      std::cerr << "Rewinding" << std::endl;
      rewind(input_fp_);
      // Read the first line.
      if (!fgets(frame_hdr, sizeof(frame_hdr), input_fp_)){
        std::cerr << "Couldn't read first line" << std::endl;
        return false;
      }
    }

    return false;
  }

  unsigned char *frame() { return frame_.get(); }
  int frame_size() { return frame_size_; }
  int height() { return height_; }
  int width() { return width_; }

 private:
  FILE *input_fp_;
  bool init_;
  int height_;
  int width_;
  int frame_size_;
  ScopedDeleteArray<unsigned char> frame_;
  bool loop_;
};

class Benchmark;
class Transport : public mozilla::TransportInterface {
 public:
   Transport(Benchmark* benchmark);
  virtual nsresult SendRtpPacket(const void* data, int len);
  virtual nsresult SendRtcpPacket(const void* data, int len);
  void SetReceiver(mozilla::RefPtr<mozilla::VideoSessionConduit> receiver);
 private:
  mozilla::RefPtr<mozilla::VideoSessionConduit> receiver_;
  Benchmark* benchmark_;
};

class Renderer : public VideoRenderer {
 public:
  Renderer(Benchmark *parent) : parent_(parent) {}

  virtual void RenderVideoFrame(const unsigned char* buffer,
                                unsigned int buffer_size,
                                uint32_t time_stamp,
                                int64_t render_time);
  void FrameSizeChange(unsigned int, unsigned int, unsigned int) {}

 private:
  Benchmark *parent_;
};

class Benchmark {
 public:
  static Benchmark* Create(const std::string& input_file) {
    ScopedDeletePtr<Benchmark> b(new Benchmark(input_file));
    if (!b->Init())
      return nullptr;

    return b.forget();
  }

  ~Benchmark() {
  }

  void Describe() {
    std::cerr << "Running benchmark from file " << g_input_file << std::endl;
    std::cerr << "Frame count: " << total_frames_ << std::endl;
    std::cerr << "Frame rate: " << frame_rate_ << std::endl;
    std::cerr << "Frame size: " << width_ << "x" << height_ << std::endl;
    std::cout << "Frame\tProc.Time\tUser.Time\tSystem.Time\tBacklog" << std::endl;
  }

  void Run() {
    Describe();
    for (;;) {
      MonitorAutoLock lock(monitor_);
      if (!ProcessFrame())
        return;
      lock.Wait(1000000);
    }
  }

  bool ProcessFrame() {
    if (!reader_.ReadFrame()) {
      std::cerr << "No more data" << std::endl;
      return false;
    }

    int err = sender_->SendVideoFrame(reader_.frame(),
                                      reader_.frame_size(),
                                      width_,
                                      height_,
                                      mozilla::kVideoI420,
                                      0);
    if (err != mozilla::kMediaConduitNoError) {
      std::cerr << "Error sending video frame" << std::endl;
    }

    sent_frame();
    if ((total_frames_ != -1) && (frame_ct_ >= total_frames_)) {
      std::cout << "Read " << total_frames_ << "... finished" << std::endl;
      return false;
    }

    return true;
  }

  bool SetupReception() {
    if (!(receiver_ = mozilla::VideoSessionConduit::Create(nullptr)))
      return false;

    mozilla::VideoCodecConfig cinst1(120, "H261", 0);
    std::vector<mozilla::VideoCodecConfig*> configs;
    configs.push_back(&cinst1);
    int err = receiver_->ConfigureRecvMediaCodecs(configs);
    if (err != mozilla::kMediaConduitNoError)
      return false;

    renderer_ = new Renderer(this);
    err = receiver_->AttachRenderer(renderer_);
    if (err != mozilla::kMediaConduitNoError)
      return false;

    sender_transport_->SetReceiver(receiver_);

    return true;
  }

  void sent_frame() {
    ++frame_ct_;
    ++frames_outstanding_;
  }

  void received_frame() {
    --frames_outstanding_;
  }

  mozilla::Atomic<int32_t> frames_outstanding_;
  mozilla::Monitor monitor_;
  int frame_ct_;
 private:
  Benchmark(const std::string& input_file)
      : reader_(),
        input_file_(input_file),
        sender_(nullptr),
        sender_transport_(new Transport(this)),
        receiver_(nullptr),
        width_(0),
        height_(0),
        frame_rate_(g_frame_rate),
        frame_ct_(0),
        total_frames_(g_total_frames),
        loop_(total_frames_ > 0),
        cumulative_time_(0),
        interframe_time_(1000 / g_frame_rate),
        frames_outstanding_(0),
        utime_(0),
        stime_(0),
        monitor_("Benchmark") {}

  bool Init() {
    if (!(reader_.Init(input_file_, loop_)))
      return false;

    height_ = reader_.height();
    width_ = reader_.width();

    if (!(sender_ = mozilla::VideoSessionConduit::Create(nullptr)))
      return false;

    mozilla::VideoCodecConfig cinst1(120, "H261", 0);
    int err = sender_->ConfigureSendMediaCodec(&cinst1);
    if (err != mozilla::kMediaConduitNoError)
      return false;

    err = sender_->AttachTransport(sender_transport_);
    if (err != mozilla::kMediaConduitNoError)
      return false;

    return true;
  }

  YuvReader reader_;
  const std::string& input_file_;
  mozilla::RefPtr<mozilla::VideoSessionConduit> sender_;
  mozilla::RefPtr<Transport> sender_transport_;
  mozilla::RefPtr<mozilla::VideoSessionConduit> receiver_;
  mozilla::RefPtr<mozilla::VideoRenderer> renderer_;
  int width_;
  int height_;
  int frame_rate_;
  int total_frames_;
  bool loop_;
  uint64_t cumulative_time_;
  uint64_t interframe_time_;
  uint64_t utime_;
  uint64_t stime_;
};

Transport::Transport(Benchmark* benchmark): benchmark_(benchmark) {}

nsresult Transport::SendRtpPacket(const void* data, int len){
    const uint8_t* header = static_cast<const uint8_t*>(data);
    static int i = 0;
    if (header[1] & 0x80) {
      MonitorAutoLock lock(benchmark_->monitor_);
      lock.NotifyAll();
    }

    if (receiver_) {
      receiver_->ReceivedRTPPacket(data, len);
    }
    return NS_OK;
}

nsresult Transport::SendRtcpPacket(const void* data, int len){
    if (receiver_) {
      receiver_->ReceivedRTCPPacket(data, len);
    }
    return NS_OK;
}

void Transport::SetReceiver(mozilla::RefPtr<mozilla::VideoSessionConduit> receiver) {
    receiver_ = receiver;
}

bool ParseArguments(int argc, char **argv) {
  int c;
  int seconds = -1;

  while ((c = getopt(argc, argv, "i:f:r:s:R")) != -1) {
    switch(c) {
      case 'i':
        g_input_file = optarg;
        break;
      case 'f':
        g_total_frames = atoi(optarg);
        break;
      case 'r':
        g_frame_rate = atoi(optarg);
        break;
      case 's':
        seconds = atoi(optarg);
        break;
      case 'R':
        g_send_and_receive = true;
        break;
#if 0
      case 'h':
        g_height = atoi(optarg);
        break;
      case 'w':
        g_width = atoi(optarg);
        break;
#endif
      default:
        usage();
        return false;
    }
  }

  if (seconds != -1)
    g_total_frames = g_frame_rate * seconds;

  return true;
}

void Renderer::RenderVideoFrame(const unsigned char* buffer,
                                unsigned int buffer_size,
                                uint32_t time_stamp,
                                int64_t render_time) {
  parent_->received_frame();
}

}  // namespace mozilla

using namespace mozilla;

int main(int argc, char **argv)
{
  std::cerr << "Video benchmark test\n";
  ScopedDeletePtr<MtransportTestUtils> test_utils(new MtransportTestUtils());

  ParseArguments(argc, argv);

  if (g_input_file.empty()) {
    std::cerr << "Must specify input file" << std::endl;
    return 1;
  }

  const bool enableBenchmark = true;
  const int minBitrate = 20000;
  const int startBitrate = 300000;
  const int maxBitrate = 2000000;
  const int maxFramerate = 1000;

  nsresult rv;
  nsCOMPtr<nsIPrefService> prefs = do_GetService("@mozilla.org/preferences-service;1", &rv);
  if (NS_SUCCEEDED(rv)) {
    nsCOMPtr<nsIPrefBranch> branch = do_QueryInterface(prefs);

    if (branch) {
      branch->SetBoolPref("media.navigator.video.benchmark", enableBenchmark);
      branch->SetIntPref("media.navigator.video.minBitrate", minBitrate);
      branch->SetIntPref("media.navigator.video.startBitrate", startBitrate);
      branch->SetIntPref("media.navigator.video.maxBitrate", maxBitrate);
      branch->SetIntPref("media.navigator.video.maxFramerate", maxFramerate);
    }
  }
  /*
  Preferences::AddBoolVarCache(&enableBenchmark, "media.navigator.video.benchmark");
  Preferences::AddIntVarCache(&minBitrate, "media.navigator.video.minBitrate");
  Preferences::AddIntVarCache(&startBitrate, "media.navigator.video.startBitrate");
  Preferences::AddIntVarCache(&maxBitrate, "media.navigator.video.maxBitrate");
  Preferences::AddIntVarCache(&maxFramerate, "media.navigator.video.maxFramerate");
*/
  ScopedDeletePtr<Benchmark> benchmark(Benchmark::Create(g_input_file));
  if (!benchmark) {
    std::cerr << "Couldn't create benchmark" << std::endl;
    return 1;
  }
  if (g_send_and_receive)
    benchmark->SetupReception();
  uint64_t t0 = time64();
  uint64_t ut0, st0;
  getrtimes(&ut0, &st0);

  benchmark->Run();

  uint64_t t1 = time64();
  uint64_t elapsed_ms = (t1 - t0) / 1000;

  uint64_t ut1, st1;
  getrtimes(&ut1, &st1);
  std::cout << "SYSTEM: " << st1-st0 << std::endl;
  std::cout << "USER: " << ut1 - ut0 << std::endl;
  std::cout << "TIME: " << t1 - t0 << std::endl;
  std::cout << "total: " << elapsed_ms << "ms" << std::endl;
  std::cout << "fps: " << (float)(benchmark->frame_ct_ * 1000 )/elapsed_ms << std::endl;
  return 0;
}
