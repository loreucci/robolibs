#include "commons.h"

#include <chrono>

namespace sec {


bool initialized = false;
std::string timestamp;

std::string getUniqueTimeStamp() {

    // read the current time just once
    if (!initialized) {
        auto now = std::chrono::system_clock::now();
        auto now_c = std::chrono::system_clock::to_time_t(now);
        struct tm* timeinfo = localtime(&now_c);
        // std::string time = std::string(std::put_time(&now_c, "%c %Z")); not yet supported
        char buffer[80];
        std::strftime(buffer,80,"%F-%H%M%S",timeinfo);
        timestamp = buffer;
        initialized = true;
    }

    return timestamp;

}


}