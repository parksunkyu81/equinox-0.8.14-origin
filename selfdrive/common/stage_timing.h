#pragma once

// Per-frame stage timing for the camera -> model path, always on.
//
// On the 2026-11-25 drive, camera EOF -> modelV2 took 62 ms (p50), of which the
// model itself was 32 ms. The other ~30 ms happens before modeld gets the frame
// (camerad's debayer / rgb->yuv on the GPU the model also uses, then VisionIPC)
// or around the model call, but rlogs carry no roadCameraState to split it.
// camerad and modeld each feed their stage durations in here, and every 10 s
// one JSON line goes to /data/log/<name>.jsonl with p50 / p90 / max per stage.
//
// Cost: a handful of clock_gettime calls per frame at 20 Hz, plus sorting ~200
// floats per stage every 10 s.

#include <time.h>

#include <algorithm>
#include <cstdio>
#include <ctime>
#include <string>
#include <vector>

namespace stage_timing {

inline double boot_ms() {
  timespec ts;
  clock_gettime(CLOCK_BOOTTIME, &ts);
  return ts.tv_sec * 1e3 + ts.tv_nsec * 1e-6;
}

inline double mono_ms() {
  timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return ts.tv_sec * 1e3 + ts.tv_nsec * 1e-6;
}

class Timer {
public:
  Timer(const char *log_path, std::vector<std::string> names)
      : path_(log_path), names_(std::move(names)), samples_(names_.size()) {
    for (auto &s : samples_) s.reserve(256);
  }

  void add(size_t stage, double ms) {
    if (stage < samples_.size()) samples_[stage].push_back((float)ms);
  }

  void count(int n = 1) { counter_ += n; }

  // Call once per frame, after the add()s for that frame.
  void frame_done() {
    frames_++;
    const double now = boot_ms();
    if (window0_ == 0.0) {
      window0_ = now;
      return;
    }
    const double dt = now - window0_;
    if (dt < WINDOW_MS) return;

    std::string line = "{\"unix_s\":" + std::to_string((long)time(nullptr)) +
                       ",\"window_s\":" + fmt(dt / 1000.0) +
                       ",\"fps\":" + fmt(frames_ * 1000.0 / dt) +
                       ",\"skipped\":" + std::to_string(counter_) +
                       // EOF stamps may be CLOCK_MONOTONIC while ours are BOOTTIME;
                       // they only differ after a suspend. Logged so it can be checked.
                       ",\"boot_minus_mono_ms\":" + fmt(now - mono_ms());
    const char *keys[3] = {"p50_ms", "p90_ms", "max_ms"};
    const double qs[3] = {0.5, 0.9, 1.0};
    for (int k = 0; k < 3; k++) {
      line += ",\"" + std::string(keys[k]) + "\":{";
      for (size_t i = 0; i < samples_.size(); i++) {
        auto &v = samples_[i];
        double val = -1.0;
        if (!v.empty()) {
          if (k == 0) std::sort(v.begin(), v.end());
          val = v[std::min(v.size() - 1, (size_t)(qs[k] * (v.size() - 1) + 0.5))];
        }
        line += (i ? ",\"" : "\"") + names_[i] + "\":" + fmt(val);
      }
      line += "}";
    }
    line += "}\n";

    if (FILE *probe = fopen(path_.c_str(), "r")) {
      fseek(probe, 0, SEEK_END);
      const long size = ftell(probe);
      fclose(probe);
      if (size > LOG_MAX_BYTES) rename(path_.c_str(), (path_ + ".1").c_str());
    }
    if (FILE *f = fopen(path_.c_str(), "a")) {
      fwrite(line.data(), 1, line.size(), f);
      fclose(f);
    }

    for (auto &s : samples_) s.clear();
    frames_ = 0;
    counter_ = 0;
    window0_ = now;
  }

private:
  static constexpr double WINDOW_MS = 10000.0;
  static constexpr long LOG_MAX_BYTES = 2 * 1024 * 1024;

  static std::string fmt(double v) {
    char b[32];
    snprintf(b, sizeof(b), "%.2f", v);
    return b;
  }

  std::string path_;
  std::vector<std::string> names_;
  std::vector<std::vector<float>> samples_;
  double window0_ = 0.0;
  int frames_ = 0;
  int counter_ = 0;
};

}  // namespace stage_timing
