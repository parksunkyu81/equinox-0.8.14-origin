#include <cstring>
#include <cstdio>

#include "selfdrive/modeld/runners/snpemodel.h"
#include "selfdrive/modeld/thneed/thneed.h"
#include "selfdrive/hardware/hw.h"

#define TEMPORAL_SIZE 512
#define DESIRE_LEN 8
#define TRAFFIC_CONVENTION_LEN 2

// TODO: This should probably use SNPE directly.
int main(int argc, char* argv[]) {
  #define OUTPUT_CAPACITY 0x10000
  #define V0813_NET_OUTPUT_SIZE 6472
  // 6012 parser output + 512 recurrent; see driving.h's BIG_MODEL asserts.
  #define BIG_NET_OUTPUT_SIZE 6524

  if (argc < 3) {
    fprintf(stderr, "usage: %s MODEL.dlc MODEL.thneed [--binary] [--extra]\n", argv[0]);
    return 2;
  }

  bool save_binaries = false;
  // The two-image model has to be recorded with both inputs bound, or the
  // captured kernel sequence is missing its wide branch.
  bool use_extra = false;
  for (int i = 3; i < argc; ++i) {
    if (strcmp(argv[i], "--binary") == 0) {
      save_binaries = true;
    } else if (strcmp(argv[i], "--extra") == 0) {
      use_extra = true;
    } else {
      fprintf(stderr, "unsupported option: %s\n", argv[i]);
      return 2;
    }
  }

  float *output = (float*)calloc(OUTPUT_CAPACITY, sizeof(float));
  // SNPEModel asserts that this exactly matches the DLC output tensor.
  SNPEModel mdl(argv[1], output, use_extra ? BIG_NET_OUTPUT_SIZE : V0813_NET_OUTPUT_SIZE,
                USE_GPU_RUNTIME, use_extra);

  float state[TEMPORAL_SIZE] = {0};
  float desire[DESIRE_LEN] = {0};
  float traffic_convention[TRAFFIC_CONVENTION_LEN] = {0};
  float *input = (float*)calloc(0x1000000, sizeof(float));
  float *extra = use_extra ? (float*)calloc(0x1000000, sizeof(float)) : NULL;

  mdl.addRecurrent(state, TEMPORAL_SIZE);
  mdl.addDesire(desire, DESIRE_LEN);
  mdl.addTrafficConvention(traffic_convention, TRAFFIC_CONVENTION_LEN);
  mdl.addImage(input, 0);
  if (use_extra) {
    mdl.addExtra(extra, 0);
  }

  // first run
  printf("************** execute 1 **************\n");
  memset(output, 0, OUTPUT_CAPACITY * sizeof(float));
  mdl.execute();

  // save model
  mdl.thneed->save(argv[2], save_binaries);

  // test model
  auto thneed = new Thneed(true);
  thneed->record = false;
  thneed->load(argv[2]);
  thneed->clexec();
  thneed->find_inputs_outputs();

  return 0;
}

