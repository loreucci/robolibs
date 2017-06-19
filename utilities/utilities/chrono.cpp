#include "chrono.h"

namespace Utils {

Chrono::Chrono() {
    started = false;
}

void Chrono::start() {
    starttime = std::chrono::system_clock::now();
    started = true;
}

double Chrono::getTime() {
    if (!started)
        return -1.0;
    std::chrono::duration<double> diff = std::chrono::system_clock::now()-starttime;
    return diff.count();
}

bool Chrono::isStarted() {
    return started;
}

}
