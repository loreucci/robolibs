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
 * \file signals.h
 * \brief Classes and functions related to discrete signals.
 */

#ifndef SIGNALS_H
#define SIGNALS_H

#include <functional>
#include <string>


/*!
 * \brief This namespace contains commonly used discrete signals.
 */
namespace Signals {

/*!
 * \brief An interface for discrete signals.
 *
 * A Signal represents a dicrete signal. Its interface is build so that every time the function operator is called,
 * the sampling time is advanced to the next discrete step in time.
 *
 * This class describes the signal interface as well as managing some internal state such as the current step.
 *
 * Each signal should be implemented as a function that provides the Signal constructor,
 * the appropriate signal function (that has the current step and sampling frequency as parameters)
 * and a description. These functions should be called with a fixed sampling frequency.
 * Examples of such functions can be found in the Signals namespace.
 *
 * Signals can be combined by using arithmetic operators between them and with doubles.
 */
class Signal {

public:

    /*!
     * \brief Creates a new Signal with a fixed sampling frequency.
     *
     * \param fun the signal function that will be called at each step, y(t).
     * \param description a description of the signal (for logging).
     * \param samplingfreq the sampling frequency.
     */
    explicit Signal(std::function<double(double)> fun = [](double){return 0.0;},
                    const std::string& description = "[default signal (0)]",
                    double samplingfreq = 0.0);

    /*!
     * \brief Gets the value of the signal at the current time step.
     *
     * This function also advances the internal time step.
     *
     * \return the output value of the signal
     */
    double operator()();

    /*!
     * \brief Gets the value of the signal at the current time step.
     *
     * This function also advances the internal time step.
     *
     * \return the output value of the signal
     */
    double output();

    /*!
     * \brief Resets the signal to a specific point in time.
     *
     * \param time time to which the signal will be reset
     */
    void reset(double time = 0.0);

    /*!
     * \brief Conversion to string.
     *
     * Returns a synthetic representation of the signal. This should include all the relevant parameters.
     * Usually is something of the form "[sin: a=ampl, f=freq, ph=phase]".
     *
     * \return the string representation
     */
    std::string to_string() const;

    /*!
     * \brief Gets the signal function.
     *
     * \return the signal function
     */
    std::function<double(double)> getFunction() const;

    /*!
     * \brief Gets the sampling frequency.
     *
     * \return the current sampling frequency
     */
    double getSamplingFreq() const;

    /*!
     * \brief Changes the sampling frequency.
     *
     * \param samplingfreq new sampling frequency
     */
    void setSamplingFreq(double samplingfreq);

private:
    unsigned int t;
    std::function<double(double)> fun;
    std::string description;
    double samplingfreq;

};


/*!
 * \brief Creates a new signal whose value is constant over time.
 *
 * The sampling frequency is not used in this signal type.
 *
 * \param c value of the signal
 * \param samplingfreq sampling frequency
 * \return the constant Signal
 */
Signal constant(double c, double samplingfreq = 0.0);


/*!
 * \brief Creates a new sinusoidal signal.
 *
 * \param ampl amplitude
 * \param freq frequency
 * \param phase phase
 * \param samplingfreq sampling frequency
 * \return the sinusoidal Signal.
 */
Signal sin(double ampl, double freq, double phase = 0.0, double samplingfreq = 100.0);


/*!
 * \brief Creates a new triangular signal.
 *
 * \param ampl amplitude
 * \param freq frequency
 * \param phase phase
 * \param samplingfreq sampling frequency
 * \return the triangular Signal
 */
Signal triangle(double ampl, double freq, double phase, double samplingfreq = 100.0);


/*!
 * \brief Creates a new  signal.
 *
 * The four phases of the wave are equally long.
 *
 * \param ampl amplitude
 * \param freq frequency
 * \param phase phase
 * \param samplingfreq sampling frequency
 * \return the trapezoidal Signal
 */
Signal trapezoid(double ampl, double freq, double phase, double samplingfreq = 100.0);


/*!
 * \brief Creates a new ramp signal.
 *
 * The signal increases indefinitely.
 *
 * \param slope slope of the ramp
 * \param initialvalue initial value of the signal
 * \param starttime time at which the slope should start
 * \param samplingfreq sampling frequency
 * \return the ramp Signal
 */
Signal ramp(double slope, double initialvalue, double starttime = 0.0, double samplingfreq = 100.0);


/*!
 * \brief Ramp and hold signal.
 *
 * This is a ramp signal that stops increasing after a certain amount of time.
 *
 * \param slope slope of the ramp
 * \param initialvalue initial value of the signal
 * \param stoptime time at which the splope should stop increasing
 * \param starttime time at which the slope should start
 * \param samplingfreq sampling frequency
 * \return the ramp and hold Signal
 */
Signal rampandhold(double slope, double initialvalue, double stoptime, double starttime = 0.0, double samplingfreq = 100.0);


/*!
 * \brief Creates a new chirp signal.
 *
 * \param ampl amplitude
 * \param f0 initial frequency
 * \param k ate of frequency change (chirpyness)
 * \param phase phase
 * \param samplingfreq sampling frequency
 * \return the chirp Signal
 */
Signal chirp(double ampl, double f0, double k, double phase = 0.0, double samplingfreq = 100.0);


/*!
 * \brief Creates a Gaussian noise signal.
 *
 * \param mean mean value of the noise
 * \param stddev standard deviation of the noise
 * \param samplingfreq sampling frequency
 * \return the noise Signal
 */
Signal noise(double mean, double stddev, double samplingfreq = 100.0);


/*!
 * \brief Creates a new switch between two signals that activates after a certain time.
 *
 * The first signal will be the ouput before the switching time, the second one after.
 * The second signal can be shifted in time of switchtime so that s2'(switchtime) = s2(0).
 *
 * \param s1 first Signal
 * \param s2 second Signal
 * \param switchtime time of the switch
 * \param shift true if s2 needs to be shifted
 * \param samplingfreq sampling frequency
 * \return the switch Signal
 */
Signal Switch(Signal s1, Signal s2, double switchtime, bool shift = false, double samplingfreq = 100.0);


/*!
 * \brief Creates a signal that is a generic binary operation between two signals (eg. +, -).
 *
 * Used as a base function for all binary operations implemented.
 *
 * \param s1 first Signal operand
 * \param s2 second Signal operand
 * \param op binary operator
 * \param samplingfreq sampling frequency
 * \return resulting Signal
 */
Signal BinaryOperation(Signal s1, Signal s2, std::function<double(double, double)> op, double samplingfreq = 100.0);


/*!
 * \brief Creates a signal that is the sum of two signals.
 *
 * \param s1 first operand
 * \param s2 second operand
 * \return Signal representing the sum
 */
Signal operator+(Signal s1, Signal s2);


/*!
 * \brief Creates a signal that is the subtraction of two signals.
 *
 * \param s1 first operand
 * \param s2 second operand
 * \return Signal representing the subtraction
 */
Signal operator-(Signal s1, Signal s2);


/*!
 * \brief Creates a signal that is the product of the two signals
 *
 * \param s1 first operand
 * \param s2 second operand
 * \return Signal representing the product
 */
Signal operator*(Signal s1, Signal s2);


/*!
 * \brief Creates a signal that is the quotient of the two signals.
 *
 * \param s1 first operand
 * \param s2 second operand
 * \return Signal representing the quotient
 */
Signal operator/(Signal s1, Signal s2);


/*!
 * \brief Creates a signal that is the sum of a given Signal and a constant one.
 *
 * \param c constant value
 * \param s Signal
 * \return Signal representing the sum
 */
Signal operator+(double c, Signal s);
/*!
 * @copydoc operator+(double, Signal)
 */
Signal operator+(Signal s, double c);


/*!
 * \brief Creates a signal that is the subtraction of a given Signal from a constant one.
 *
 * \param c constant value
 * \param s Signal
 * \return Signal representing the subtraction
*/
Signal operator-(double c, Signal s);
/*!
 * @copydoc operator-(double, Signal)
 */
Signal operator-(Signal s, double c);


/*!
 * \brief Creates a signal that is the product of a constant signals and a given one.
 *
 * \param c constant value
 * \param s Signal
 * \return Signal representing the product
*/
Signal operator*(double c, Signal s);
/*!
 * @copydoc operator*(double, Signal)
 */
Signal operator*(Signal s, double c);


/*!
 * \brief Creates a signal that is the quotient of a constant Signal and a given one.
 *
 * \param c constant value
 * \param s Signal
 * \return Signal representing the quotient
*/
Signal operator/(double c, Signal s);
/*!
 * @copydoc operator/(double, Signal)
 */
Signal operator/(Signal s, double c);

}


#endif // SIGNALS_H
