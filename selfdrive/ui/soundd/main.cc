#include <sys/resource.h>

#include <QApplication>

#include "selfdrive/common/util.h"
#include "selfdrive/ui/qt/util.h"
#include "selfdrive/ui/soundd/sound.h"

void sigHandler(int s) {
  qApp->quit();
}

int main(int argc, char **argv) {
  qInstallMessageHandler(swagLogMessageHandler);

  // soundd is the only process on this device with no real-time priority at
  // all, and nice -20 buys nothing against SCHED_FIFO: every RT process
  // preempts it unconditionally, down to rtshield at FIFO 1 -- whose entire
  // job is to keep core 3 hostile to exactly this kind of task. With no
  // affinity set the scheduler is free to place soundd on that core, where it
  // then queues behind rtshield, controlsd (53) and boardd (54) and gets
  // almost nothing. A starved audio thread is a buffer underrun, which is
  // inaudible in a 0.7 s chime and unmistakable in a 1.5 s spoken line -- the
  // asymmetry the tearing has shown from the start.
  //
  // Priority 5 matches locationd and paramsd: above the FIFO-1 tier, far below
  // control at 51-54, so it cannot delay a control frame. Both calls sit ahead
  // of QApplication so the Qt audio threads inherit the policy and the mask.
  //
  // Cores 0-1, not 0-2: core 2 is modeld's, at FIFO 54, and this device runs
  // the big supercombo -- 48 MB against the stock 29 MB, with a second image
  // tensor. That is the heaviest periodic job on the machine sitting at the
  // second-highest priority on the machine, and it is what changed when the
  // voice alerts started tearing. Cores 0-1 hold only plannerd at FIFO 51,
  // which is 20 Hz and short.
  if (Hardware::EON()) {
    util::set_core_affinity({0, 1});
    util::set_realtime_priority(5);
  }
  setpriority(PRIO_PROCESS, 0, -20);

  QApplication a(argc, argv);
  std::signal(SIGINT, sigHandler);
  std::signal(SIGTERM, sigHandler);

  Sound sound;
  return a.exec();
}
