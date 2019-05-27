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
 * \file sleeper.h
 * \brief Interfaces and classes for the waiting mechanism used by the Synchronizer.
 */

#ifndef SLEEPER_H
#define SLEEPER_H

namespace sec {

/*!
 * \brief Abstract interface for the waiting mechanism used by the Synchronizer.
 *
 * A Sleeper is an object responsible for waiting for a certain amount of time.
 * This is used by the Synchronizer to wake up the different threads of the Controller at specific frequencies.
 * Different implementations can specify whether the time waited is wall clock time, simulated time or if no waiting should be done.
 *
 * Depending on the implmentation, the Sleeper will set the synchronization mechanism used by the runtime: real-time or full-synch.
 */
class Sleeper {

public:
    /*!
     * \brief Waits for a certain amount of time.
     *
     * Depending on the implementation this could be wall clock time, simulated time, etc..
     *
     * \param ms waiting time (ms)
     */
    virtual void sleep(double ms) = 0;

    /*!
     * \brief Checks whether the runtime is operating in real-time or full-synch mode.
     *
     * \return true if full-synch mode
     */
    virtual bool isSynchronous() const = 0;

};


/*!
 * \brief Sleeper that waits for wall clock time to pass.
 *
 * This Sleeper sets the synchronization mode to real-time and waits for absolute time, as given by the CPU.
 * This is the default Sleeper, and is the one that should be used with physical systems.
 */
class BasicSleeper : public Sleeper {

public:

    virtual void sleep(double ms) override;

    virtual bool isSynchronous() const override;

};


/*!
 * \brief Sleeper that does not wait for time to pass.
 *
 * This Sleeper sets the synchronization mode to real-time, but does not actually wait for time to pass.
 * This can be useful if there is no need to synchronize with physical sistems or simulators and to run the Controller at the maximum possible speed.
 */
class NoSleeper : public Sleeper {

public:

    virtual void sleep(double) override;

    virtual bool isSynchronous() const override;

};


/*!
 * \brief Sleeper that waits for all threads to finish.
 *
 * This Sleeper sets the synchronization mode to full-synch, without actually waiting any amount of time.
 * This has the effect of waiting for all Controller threads to finish their step before moving to the next.
 */
class Barrier : public Sleeper {

public:

    virtual void sleep(double) override;

    virtual bool isSynchronous() const override;

};

}

#endif // SLEEPER_H
