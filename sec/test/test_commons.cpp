#include <sec/commons.h>
#include <sec/synchronization.h>

#include <iostream>
#undef NDEBUG
#include <cassert>

int main(void) {

    auto uts1 = sec::getUniqueTimeStamp();
//    std::cout << uts1 << std::endl;
    sec::synchronizer.sleep(2000.0);
    auto uts2 = sec::getUniqueTimeStamp();
//    std::cout << uts2 << std::endl;

    assert(uts1 == uts2);

    return 0;
}
