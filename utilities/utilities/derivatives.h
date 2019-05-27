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

/*!
 * \file derivatives.h
 * \brief Classes and functions related to discrete derivation.
 */

#ifndef DERIVATIVES_H
#define DERIVATIVES_H

#include <deque>

#include "vector.h"


/*!
 * \brief An interface for numeric, discrete derivation methods.
 *
 * A Derivative is an object that can compute the derivative of a discrete signal.
 * The signal can be a vector of different signals and the derivation will be performed element-wise.
 */
class Derivative {

public:

    /*!
     * \brief Creates an Derivative object with a fixed sampling frequency.
     *
     * \param freq sampling frequency of the derivation
     */
    Derivative(double freq);

    /*!
     * \brief Performs the discrete derivation step.
     *
     * Computes the derivation, given a new sample of the signal.
     *
     * \param x input value sample
     * \return the discrete derivative of the input at this step
     */
    virtual Utils::Vector derive(const Utils::Vector& x) = 0;

    /*!
     * \brief Performs the discrete derivation step.
     *
     * Computes the derivation, given a new sample of the signal.
     *
     * \param x input value sample
     * \return the discrete derivative of the input at this step
     */
    virtual double derive(double x) final;

protected:

    /*!
     * \brief Sampling frequency.
     */
    double freq;

};


/*!
 * \brief Simple one-step discrete derivative.
 *
 * This class implements a simple one-step discrete derivative: \f$ \frac{f(x_n)-f(x_{n-1})}{\Delta t} \f$.
 */
class SimpleDerivative : public Derivative {

public:

    /*!
     * \brief Inherited constructor.
     */
    using Derivative::Derivative;

    virtual Utils::Vector derive(const Utils::Vector& x) override;

private:
    Utils::Vector prev;

};


/*!
 * \brief Smooth derivation.
 *
 * This class implements a smooth noise-robust differentiator, as described in
 * http://www.holoborodko.com/pavel/numerical-methods/numerical-derivative/smooth-low-noise-differentiators/
 *
 * In this case, the derivative obtained is delayed by \f$ \frac{N-1}{2} \f$ steps, due to casuality.
 */
class SmoothDerivative : Derivative {

public:

    /*!
     * \brief Creates a new smooth derivative.
     *
     * Creates a new filter with a fixed temporal window and sampling frequency.
     *
     * \param N the size of the temporal window (must be and odd number)
     * \param freq the sampling frequency
     */
    SmoothDerivative(unsigned int N, double freq);

    virtual Utils::Vector derive(const Utils::Vector& x) override;

private:
    unsigned int N, M, m, datasize;
    double coeff;
    std::vector<double> c;
    std::deque<std::vector<double>> past, fut;
    std::vector<double> pres;

};


#endif // DERIVATIVES_H
