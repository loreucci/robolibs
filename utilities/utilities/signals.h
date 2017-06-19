#ifndef SIGNALS_H
#define SIGNALS_H

#include <functional>


//!  Signals namespace.
/*!
  This namespace contains commonly used discrete signals.
*/
namespace Signals {

//!  Signal class.
/*!
  This class describes the signal interface as well as managing some internal state such
  as the current step.

  Each signal should be implemented as a function that provides the Signal constructor
  the appropriate signal function (that has the current step and sampling frequency as parameters)
  and a description. These functions should be called with a fixed sampling frequency.
*/
class Signal {

public:
    //! Signal constructor.
    /*!
      Creates a new Signal with a fixed sampling frequency.
      \param fun the signal function that will be called at each step.
      \param description a description of the signal (for logging).
      \param samplingfreq the sampling frequency.
    */
    explicit Signal(std::function<double(unsigned int, double)> fun = [](unsigned int, double){return 0.0;},
                    const std::string& description = "[default signal (0)]",
                    double samplingfreq = 0.0);

    //! Function operator.
    /*!
      Calls output().
      \return the output value of the signal.
    */
    double operator()();

    //! Function operator.
    /*!
      \return the output value of the signal.
    */
    double output();

    //! Resets the signal to a specific point in time.
    /*!
      \param time time to which the signal will be reset.
    */
    void reset(double time = 0.0);

    //! Conversion to string.
    /*!
      Synthetic representation of the signal.
      \return the string representation.
    */
    std::string to_string() const;

    //! Signal function getter.
    /*!
      \return the signal function.
    */
    std::function<double(unsigned int, double)> getFunction() const;

    //! Sampling frequency getter.
    /*!
      \return the current sampling frequency.
    */
    double getSamplingFreq() const;

    //! Sampling frequency setter.
    /*!
      \param samplingfreq new sampling frequency.
    */
    void setSamplingFreq(double samplingfreq);

protected:
    unsigned int t;
    std::function<double(unsigned int, double)> fun;
    std::string description;
    double samplingfreq;

};

//! Constant signal.
/*!
  Creates a new constant signal.
  The sampling frequency is not used in this signal type.
  \param c value of the signal.
  \param samplingfreq the sampling frequency.
  \return the constant Signal.
*/
Signal constant(double c, double samplingfreq = 0.0);

//! Sinusoidal signal.
/*!
  Creates a new sinusoidal signal.
  \param ampl amplitude.
  \param freq frequency.
  \param phase phase.
  \param samplingfreq the sampling frequency.
  \return the sinusoidal Signal.
*/
Signal sin(double ampl, double freq, double phase = 0.0, double samplingfreq = 100.0);

//! Ramp signal.
/*!
  Creates a new ramp signal.
  \param slope the slope of the ramp.
  \param initialvalue initial value of the signal.
  \param starttime time at which the splope should start.
  \param samplingfreq the sampling frequency.
  \return the ramp Signal.
*/
Signal ramp(double slope, double initialvalue, double starttime = 0.0, double samplingfreq = 100.0);

//! Ramp and hold signal.
/*!
  Creates a new ramp and hold signal, i.e. a ramp signal that stops increasing after a certain time.
  \param slope the slope of the ramp.
  \param initialvalue initial value of the signal.
  \param stop time at which the splope should stop increasing.
  \param starttime time at which the splope should start.
  \param samplingfreq the sampling frequency.
  \return the ramp and hold Signal.
*/
Signal rampandhold(double slope, double initialvalue, double stoptime, double starttime = 0.0, double samplingfreq = 100.0);

//! Chirp signal.
/*!
  Creates a new chirp signal.
  \param ampl amplitude.
  \param f0 initial frequency.
  \param k rate of frequency change (chirpyness).
  \param phase phase.
  \param samplingfreq the sampling frequency.
  \return the chirp Signal.
*/
Signal chirp(double ampl, double f0, double k, double phase = 0.0, double samplingfreq = 100.0);

//! Noise signal.
/*!
  Creates a Gaussian noise signal.
  \param mean mean value of the noise.
  \param stddev standard deviation of the noise.
  \param samplingfreq the sampling frequency.
  \return the noise Signal.
*/
Signal noise(double mean, double stddev, double samplingfreq = 100.0);

//! Switch signal.
/*!
  Creates a new switch between two signals that activates after a certain time.
  The first signal will be the ouput before the switching time, the second one after.
  The second signal can be shifted in time of switchtime so that s2'(switchtime) = s2(0).
  \param s1 first singnal.
  \param s2 second singal.
  \param switchtime time of the switch.
  \param shift shift flag.
  \param samplingfreq the sampling frequency.
  \return the switch Signal.
*/
Signal Switch(Signal s1, Signal s2, double switchtime, bool shift = false, double samplingfreq = 100.0);

//! BinaryOperation signal.
/*!
  Creates a signal that is a generic binary operation between two signals (eg. +, -).
  Used as a base function for all binary operations implemented.
  \param s1 first operand.
  \param s2 second operand.
  \param fun binary operator.
  \param samplingfreq the sampling frequency.
  \return the resulting Signal.
*/
Signal BinaryOperation(Signal s1, Signal s2, std::function<double(double, double)> op, double samplingfreq = 100.0);


//! Plus operator between Signals.
/*!
  Creates a signal that is the sum of two signals.
  \param s1 first operand.
  \param s2 second operand.
  \return Signal representing the sum.
*/
Signal operator+(Signal s1, Signal s2);

//! Minus operator between Signals.
/*!
  Creates a signal that is the subtraction of two signals.
  \param s1 first operand.
  \param s2 second operand.
  \return Signal representing the subtraction.
*/
Signal operator-(Signal s1, Signal s2);

//! Multiplies operator between Signals.
/*!
  Creates a signal that is the product of the two signals.
  \param s1 first operand.
  \param s2 second operand.
  \return Signal representing the product.
*/
Signal operator*(Signal s1, Signal s2);

//! Divides operator between Signals.
/*!
  Creates a signal that is the quotient of the two signals.
  \param s1 first operand.
  \param s2 second operand.
  \return Signal representing the quotient.
*/
Signal operator/(Signal s1, Signal s2);


//! Plus operator between Signals and constants.
/*!
  Creates a signal that is the sum of a given Signal and a constant one.
  \param c constant value.
  \param s Signal.
  \return Signal representing the sum.
*/
Signal operator+(double c, Signal s);
Signal operator+(Signal s, double c);

//! Minus operator between Signals and constants.
/*!
  Creates a signal that is the subtraction of a given Signal from a constant one.
  \param c constant value.
  \param s Signal.
  \return Signal representing the subtraction.
*/
Signal operator-(double c, Signal s);
Signal operator-(Signal s, double c);

//! Multiply operator between Signals and constants.
/*!
  Creates a signal that is the product of a constant signals and a given one.
  \param c constant value.
  \param s Signal.
  \return Signal representing the product.
*/
Signal operator*(double c, Signal s);
Signal operator*(Signal s, double c);

//! Divides operator between Signals and constants.
/*!
  Creates a signal that is the quotient of a constant Signal and a given one.
  \param c constant value.
  \param s Signal.
  \return Signal representing the quotient.
*/
Signal operator/(double c, Signal s);
Signal operator/(Signal s, double c);

}


#endif // SIGNALS_H
