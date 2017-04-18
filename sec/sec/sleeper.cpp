#include "sleeper.h"

#include <thread>
#include <chrono>

namespace sec {

void BasicSleeper::sleep(double d) {

    std::this_thread::sleep_for(std::chrono::duration<double, std::milli>(d));

}

bool BasicSleeper::isSynchronous() const {
    return false;
}


void NoSleeper::sleep(double) {

}

bool NoSleeper::isSynchronous() const {
    return false;
}

void Barrier::sleep(double) {

}

bool Barrier::isSynchronous() const {
    return true;
}

}
