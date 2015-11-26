#include "sleeper.h"

#include <thread>
#include <chrono>

void BasicSleeper::sleep(double d) {

    std::this_thread::sleep_for(std::chrono::duration<double, std::milli>(d));

}


void NoSleeper::sleep(double) {

}
