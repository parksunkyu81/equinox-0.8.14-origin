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

  if (argc < 3) {
    fprintf(stderr, "usage: %s MODEL.dlc MODEL.thneed [--binary]\n", argv[0]);
    return 2;
  }

  bool save_binaries = false;
  for (int i = 3; i < argc; ++i) {
    if (strcmp(argv[i], "--binary") == 0) {
      save_binaries = true;
    } else {
      fprintf(stderr, "unsupported option for v0.8.13 single-input model: %s\n", argv[i]);
      return 2;
    }
  }

  float *output = (float*)calloc(OUTPUT_CAPACITY, sizeof(float));
  // SNPEModel asserts that this exactly matches the DLC output tensor.
  SNPEModel mdl(argv[1], output, V0813_NET_OUTPUT_SIZE, USE_GPU_RUNTIME, false);

  float state[TEMPORAL_SIZE] = {0};
  float desire[DESIRE_LEN] = {0};
  float traffic_convention[TRAFFIC_CONVENTION_LEN] = {0};
  float *input = (float*)calloc(0x1000000, sizeof(float));

  mdl.addRecurrent(state, TEMPORAL_SIZE);
  mdl.addDesire(desire, DESIRE_LEN);
  mdl.addTrafficConvention(traffic_convention, TRAFFIC_CONVENTION_LEN);
  mdl.addImage(input, 0);

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

