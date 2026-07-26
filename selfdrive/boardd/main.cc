#include <cassert>
#include <cerrno>
#include <cstring>

#include "selfdrive/boardd/boardd.h"
#include "selfdrive/common/swaglog.h"
#include "selfdrive/common/util.h"
#include "selfdrive/hardware/hw.h"

int main(int argc, char *argv[]) {
  LOGW("starting boardd");

  if (!Hardware::PC()) {
    int err;
    err = util::set_realtime_priority(54);
    assert(err == 0);
    const int preferred_core = Hardware::TICI() ? 4 : 3;
    err = util::set_core_affinity({preferred_core});
    if (err != 0) {
      const int affinity_errno = errno;
      LOGE("failed to set boardd affinity to CPU %d: errno=%d (%s); continuing with inherited allowed CPUs",
           preferred_core, affinity_errno, std::strerror(affinity_errno));
    }
  }

  std::vector<std::string> serials(argv + 1, argv + argc);
  boardd_main_thread(serials);
  return 0;
}
