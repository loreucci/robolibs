/*
 * Copyright (C) 2014-2019 Lorenzo Vannucci
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
 */

#include "commons.h"

#include <chrono>
#include <stdexcept>


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


double defaultfreq = 0.0;

void setDefaultFrequency(double freq) {

    if (freq <= 0.0) {
        throw std::invalid_argument("[Controller] Default frequency of the controller must be positive.");
    }

    defaultfreq = freq;

}

double getDefaultFrequency() {

    return defaultfreq;

}


}
