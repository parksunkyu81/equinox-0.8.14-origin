#pragma once

// Where the onroad UI's CPU goes, always on.
//
// On 2026-11-24, parked onroad, _ui used 45% of a core (proc_cpu.jsonl), the
// third-largest process after controlsd and plannerd. The camera frame is an
// EGL image and costs the CPU next to nothing, so what is left is the HUD drawn
// over it and the 20 Hz state update -- but nothing said which part. This times
// each drawing step on the GUI thread's own CPU clock and writes one JSON line
// to /data/log/ui_paint.jsonl every 10 s while the onroad view is painting:
// mean and max CPU ms per paint for each step, the same for the state update,
// paints per second, and the GUI thread's and whole process's share of a core.
//
// Cost: two clock_gettime calls per step, ~15 per paint, at 20 paints a
// second -- a few microseconds a paint.

#include <time.h>
#include <unistd.h>

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <string>

namespace paint_profile {

enum Section {
  CAMERA, HUD_HEAD, LANES, LEADS, SPEED, SPEED_LIMIT, THERMAL, TURN_SIGNALS, TURN_INFO,
  INFO_LINE, BOTTOM_ICONS, PAINTER_END, UI_STATE, UI_SLOTS, COUNT
};
inline const char *const NAMES[COUNT] = {
  "camera", "hud_head", "lanes", "leads", "speed", "speed_limit", "thermal", "turn_signals",
  "turn_info", "info_line", "bottom_icons", "painter_end", "ui_state", "ui_slots",
};
constexpr int FIRST_STATE_SECTION = UI_STATE;

inline const char *LOG_PATH = "/data/log/ui_paint.jsonl";
constexpr long LOG_MAX_BYTES = 2 * 1024 * 1024;
constexpr double WINDOW_MS = 10000.0;

inline double acc[COUNT] = {};
inline double mx[COUNT] = {};
inline double mark_t = 0.0;
inline int paints = 0;
inline int state_updates = 0;
inline double window_wall0 = 0.0;
inline double window_thread0 = 0.0;
inline long window_ticks0 = -1;

inline double thread_cpu_ms() {
  timespec ts;
  clock_gettime(CLOCK_THREAD_CPUTIME_ID, &ts);
  return ts.tv_sec * 1e3 + ts.tv_nsec * 1e-6;
}

inline double wall_ms() {
  timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return ts.tv_sec * 1e3 + ts.tv_nsec * 1e-6;
}

inline void begin() { mark_t = thread_cpu_ms(); }

inline void mark(Section s) {
  const double now = thread_cpu_ms();
  const double d = now - mark_t;
  acc[s] += d;
  if (d > mx[s]) mx[s] = d;
  mark_t = now;
}

inline long process_ticks() {
  FILE *f = fopen("/proc/self/stat", "r");
  if (!f) return -1;
  char buf[1024];
  const size_t n = fread(buf, 1, sizeof(buf) - 1, f);
  fclose(f);
  buf[n] = 0;
  const char *p = strrchr(buf, ')');
  if (!p) return -1;
  long utime = 0, stime = 0;
  // fields after ')': state(3) ... utime(14) stime(15)
  int field = 2;
  for (const char *c = p + 1; *c && field < 15; c++) {
    if (*c == ' ') {
      field++;
      if (field == 14) utime = strtol(c + 1, nullptr, 10);
      if (field == 15) stime = strtol(c + 1, nullptr, 10);
    }
  }
  return utime + stime;
}

inline void reset_window(double now_wall) {
  for (int i = 0; i < COUNT; i++) acc[i] = mx[i] = 0.0;
  paints = state_updates = 0;
  window_wall0 = now_wall;
  window_thread0 = thread_cpu_ms();
  window_ticks0 = process_ticks();
}

// Called at the end of every onroad paint.
inline void paint_done() {
  paints++;
  const double now = wall_ms();
  if (window_wall0 == 0.0) {
    reset_window(now);
    return;
  }
  const double dt = now - window_wall0;
  if (dt < WINDOW_MS) return;

  const double gui_pct = 100.0 * (thread_cpu_ms() - window_thread0) / dt;
  const long ticks = process_ticks();
  const double proc_pct = (ticks >= 0 && window_ticks0 >= 0)
                            ? 100.0 * (ticks - window_ticks0) * (1000.0 / sysconf(_SC_CLK_TCK)) / dt
                            : -1.0;
  std::string line = "{\"unix_s\":" + std::to_string((long)time(nullptr)) +
                     ",\"window_s\":" + std::to_string(dt / 1000.0) +
                     ",\"paints_per_s\":" + std::to_string(paints * 1000.0 / dt) +
                     ",\"state_per_s\":" + std::to_string(state_updates * 1000.0 / dt) +
                     ",\"gui_thread_pct\":" + std::to_string(gui_pct) +
                     ",\"process_pct\":" + std::to_string(proc_pct);
  for (int pass = 0; pass < 2; pass++) {
    line += pass == 0 ? ",\"mean_ms\":{" : "},\"max_ms\":{";
    for (int i = 0; i < COUNT; i++) {
      const int n = i >= FIRST_STATE_SECTION ? state_updates : paints;
      const double v = pass == 0 ? (n > 0 ? acc[i] / n : 0.0) : mx[i];
      char item[64];
      snprintf(item, sizeof(item), "%s\"%s\":%.3f", i ? "," : "", NAMES[i], v);
      line += item;
    }
  }
  line += "}}\n";

  FILE *probe = fopen(LOG_PATH, "r");
  if (probe) {
    fseek(probe, 0, SEEK_END);
    const long size = ftell(probe);
    fclose(probe);
    if (size > LOG_MAX_BYTES) rename(LOG_PATH, (std::string(LOG_PATH) + ".1").c_str());
  }
  if (FILE *f = fopen(LOG_PATH, "a")) {
    fwrite(line.data(), 1, line.size(), f);
    fclose(f);
  }
  reset_window(now);
}

}  // namespace paint_profile
