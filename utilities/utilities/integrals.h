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

//!  \file integrals.h
/*!
  This file contains methods for discrete integration.
*/

#ifndef INTEGRALS_H
#define INTEGRALS_H


#include "vector.h"


//!  Integral class.
/*!
  This class is an interface for numeric, discrete integration methods.
*/
class Integral {

public:
    //! Integral constructor.
    /*!
      \param freq the sampling frequency.
    */
    Integral(double freq);

    //! Performs the integration.
    /*!
      \param x input value at this step.
      \return the definite integral up to this step.
    */
    virtual Utils::Vector integrate(const Utils::Vector& x) = 0;

protected:
    Utils::Vector integr;
    double freq;

};


//!  Integral class.
/*!
  This class implements the rectangle method of integration.
*/
class IntegralRectangle : public Integral {

public:
    //! Inherited constructor.
    using Integral::Integral;

    virtual Utils::Vector integrate(const Utils::Vector& x) override;

};


//!  Integral class.
/*!
  This class implements the trapezoidal method of integration.
*/
class IntegralTrapezoidal : public Integral {

public:
    //! IntegralTrapezoidal constructor.
    /*!
      \param freq the sampling frequency.
    */
    IntegralTrapezoidal(double freq);

    virtual Utils::Vector integrate(const Utils::Vector& x) override;

protected:
    Utils::Vector prev;
    bool first;

};

#endif // INTEGRALS_H
