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

#ifndef CHRONO_H
#define CHRONO_H

#include <chrono>

namespace Utils {

//!  Chrono class.
/*!
  This class implements a simple chronometer as a wrapper around std::chrono.
*/
class Chrono {

public:

    //! Chrono constructor.
    Chrono();

    //! Start chronometer.
    /*!
    A call to this function will set the start time of the time measure with the current time.
    Subsequent calls will update the start time.
    */
    void start();

    //! Get elapsed time.
    /*!
      This call is only valid after a call to start().
      Elapsed time is in seconds.
      \return the elapsed time.
    */
    double getTime();

    //! Check whether the chronometer has started measuring.
    /*!
      \return true if a call to start() has been executed, false otherwise.
    */
    bool isStarted();


private:
    std::chrono::system_clock::time_point starttime;
    bool started;

};


}

#endif // CHRONO_H
