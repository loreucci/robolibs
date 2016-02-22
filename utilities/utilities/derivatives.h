//!  \file derivatives.h
/*!
  This file contains methods for smooth differentiation.
*/

#ifndef DERIVATIVES_H
#define DERIVATIVES_H


#include <vector>
#include <deque>


//!  Derivative class.
/*!
  This class implements a smooth noise-robust differentiator, as described in
  http://www.holoborodko.com/pavel/numerical-methods/numerical-derivative/smooth-low-noise-differentiators/
*/
class Derivative {

public:
    //! Derivative constructor.
    /*!
      Creates a new filter with a fixed temporal window and sampling frequency.
      \param N the size of the temporal window (must be and odd number).
      \param freq the sampling frequency.
    */
    Derivative(unsigned int N, double freq);

    //! Performs the derivation.
    /*!
      \param x input value.
      \return the derivative of the input (delayed by (N-1)/2, due to causality).
    */
    std::vector<double> derive(std::vector<double> x);

protected:
    double freq;
    unsigned int N, M, m, datasize;
    double coeff;
    std::vector<double> c;
    std::deque<std::vector<double>> past, fut;
    std::vector<double> pres;

};


#endif // DERIVATIVES_H
