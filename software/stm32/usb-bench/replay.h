#pragma once
#include "flight_controller.h"
#include <stdbool.h>
#include <stdint.h>

enum { SAMPLE_COUNT = 160, PERIOD_US = 10000 };
typedef struct {
    const char *name;
    FlightControllerOutput output;
} Checkpoint;

extern Checkpoint checkpoints[3];
extern const char *const fault_names[8];
void replay_start(void);
void replay_sample(unsigned index);
bool replay_correct(void);
unsigned check_faults(void);
int format_checkpoint(char *buffer, unsigned capacity, const Checkpoint *point);
