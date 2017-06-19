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
