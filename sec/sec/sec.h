#ifndef SEC_H
#define SEC_H

#include "controller.h"
#include "connections.h"
#include "synchronization.h"

namespace sec {

void run(double time = 0.0, std::vector<std::function<bool(void)>> endconditions = {});

void sleep(double ms);

}

#endif // SEC_H
