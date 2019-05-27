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
 * \file integrals.h
 * \brief Classes and functions related to discrete integration.
 */

#ifndef INTEGRALS_H
#define INTEGRALS_H

#include "vector.h"


/*!
 * \brief An interface for numeric, discrete integration methods.
 *
 * An Integral is an object that can compute the integral of a discrete signal.
 */
class Integral {

public:

    /*!
     * \brief Creates an Integral object with a fixed sampling frequency.
     *
     * \param freq the sampling frequency
     */
    Integral(double freq);

    /*!
     * \brief Performs the discrete integration step.
     *
     * Computes the integration, given a new sample of the signal.
     *
     * \param x input value at this step
     * \return the definite integral up to this step
     */
    virtual Utils::Vector integrate(const Utils::Vector& x) = 0;

    /*!
     * \brief Performs the discrete integration step.
     *
     * Computes the integration, given a new sample of the signal.
     *
     * \param x input value at this step
     * \return the definite integral up to this step
     */
    virtual double integrate(double x) final;

protected:

    /*!
     * \brief Finite integral up to this sample point.
     */
    Utils::Vector integr;

    /*!
     * \brief Sampling Frequency
     */
    double freq;

};


/*!
 * \brief Discrete Integral with rectangle method.
 *
 * This class uses the endpoint rule to perform the integration. That is, \f$ I_n = I_{n-1} + f(x_n) \cdot \Delta t \f$ .
 */
class IntegralRectangle : public Integral {

public:

    /*!
     * \brief Inherited constructor.
     */
    using Integral::Integral;

    virtual Utils::Vector integrate(const Utils::Vector& x) override;

};


/*!
 * \brief Discrete Integral with trapezoidal method of integration.
 *
 * This class uses the trapezoidal rule to perform the integration. That is, \f$ I_n = I_{n-1} + \frac{f(x_{n-1}) + f(x_{n})}{2} \cdot \Delta t \f$ .
 */
class IntegralTrapezoidal : public Integral {

public:

    /*!
     * \brief Creates an IntegralTrapezoidal object with a fixed sampling frequency.
     *
     * \param freq the sampling frequency
     */
    IntegralTrapezoidal(double freq);

    virtual Utils::Vector integrate(const Utils::Vector& x) override;

private:
    Utils::Vector prev;
    bool first;

};

#endif // INTEGRALS_H
