#ifndef SEC_H
#define SEC_H

#include "commons.h"
#include "controller.h"
#include "connections.h"
#include "synchronization.h"
#include "flags.h"

namespace sec {

void run(double time = 0.0, std::vector<std::function<bool(void)>> endconditions = {});

void setSleeper(Sleeper* sleeper);

void sleep(double ms);

void runTrials(unsigned int N,
               std::vector<std::function<void(void)>> pre = {},
               std::vector<std::function<void(void)>> post = {},
               double time = 0.0,
               std::vector<std::function<bool(void)>> endconditions = {});

}

#endif // SEC_H
