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
    void start();

    //! Get elapsed time.
    /*!
      \return the elapsed time.
    */
    double getTime();


private:
    std::chrono::system_clock::time_point starttime;

};


}

#endif // CHRONO_H
