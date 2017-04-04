#include "sec.h"


namespace sec {

void run(double time, std::vector<std::function<bool(void)>> endconditions) {
    main_controller.run(time, endconditions);
}

void sleep(double ms) {
    synchronizer.sleep(ms);
}

}
