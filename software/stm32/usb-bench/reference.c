#include "replay.h"
#include <stdio.h>

int main(void) {
    replay_start();
    for (unsigned i = 0; i < SAMPLE_COUNT; ++i) replay_sample(i);
    for (unsigned i = 0; i < 3; ++i) {
        char row[160];
        format_checkpoint(row, sizeof row, &checkpoints[i]);
        fputs(row, stdout);
    }
    unsigned mask = check_faults();
    printf("fault_mask,%u\n", mask);
    return !(mask == 255 && replay_correct());
}
