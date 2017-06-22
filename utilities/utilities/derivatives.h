//!  \file derivatives.h
/*!
  This file contains methods for discrete differentiation.
*/

#ifndef DERIVATIVES_H
#define DERIVATIVES_H


#include <deque>

#include "vector.h"


//!  Derivative class.
/*!
  This class is an interface for numeric, discrete derivation methods.
*/
class Derivative {

public:
    //! Derivative constructor.
    /*!
      \param freq the sampling frequency.
    */
    Derivative(double freq);

    //! Performs the derivation.
    /*!
      \param x input value sample.
      \return the discrete derivative of the input at this step.
    */
    virtual Utils::Vector derive(const Utils::Vector& x) = 0;

protected:
    double freq;

};


//!  SimpleDerivative class.
/*!
  This class implements a simple one-step discrete derivative: (f(b)-f(a))/(b-a).
*/
class SimpleDerivative : public Derivative {

public:
    //! Inherited constructor.
    using Derivative::Derivative;

    virtual Utils::Vector derive(const Utils::Vector& x) override;

protected:
    Utils::Vector prev;

};


//!  SmoothDerivative class.
/*!
  This class implements a smooth noise-robust differentiator, as described in
  http://www.holoborodko.com/pavel/numerical-methods/numerical-derivative/smooth-low-noise-differentiators/
*/
class SmoothDerivative : Derivative {

public:
    //! SmoothDerivative constructor.
    /*!
      Creates a new filter with a fixed temporal window and sampling frequency.
      \param N the size of the temporal window (must be and odd number).
      \param freq the sampling frequency.
    */
    SmoothDerivative(unsigned int N, double freq);

    //! Performs the derivation.
    /*!
      \param x input value sample.
      \return the derivative of the input (delayed by (N-1)/2 steps, due to causality).
    */
    virtual Utils::Vector derive(const Utils::Vector& x) override;

protected:
    unsigned int N, M, m, datasize;
    double coeff;
    std::vector<double> c;
    std::deque<std::vector<double>> past, fut;
    std::vector<double> pres;

};


#endif // DERIVATIVES_H
