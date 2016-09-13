//!  \file signals.h
/*!
  This file contains the implementation of common signals.
*/

#ifndef SIGNALS_H
#define SIGNALS_H


#include <string>
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

  Each signal should be implemented as a function that gives the next output at each call,
  thus incrementing the internal step. These functions should be called with a fixed
  sampling frequency.
*/
class Signal {

public:
    //! Signal constructor.
    /*!
      Creates a new Signal with a fixed sampling frequency.
      \param samplingfreq the sampling frequency.
    */
    explicit Signal(double samplingfreq = 0.0);

    //! Function operator.
    /*!
      Calls output().
      \return the output value of the signal.
    */
    virtual double operator()() final;

    //! Function operator.
    /*!
      \return the output value of the signal.
    */
    virtual double output() = 0;

    //! Resets the signal to a specific point in time.
    /*!
      \param time time to which the signal will be reset.
    */
    virtual void reset(double time = 0.0);

    //! Conversion to string.
    /*!
      Synthetic representation of the signal.
      \return the string representation.
    */
    virtual std::string to_string() const = 0;

    //! Sampling frequency getter.
    /*!
      \return the current sampling frequency.
    */
    double getSamplingFreq() const;

    //! Sampling frequency setter.
    /*!
      \param samplingfreq new sampling frequency.
    */
    virtual void setSamplingFreq(double samplingfreq);

protected:
    double samplingfreq;
    unsigned int t;

};


//!  constant class.
/*!
  This class represents a constant signal.
*/
class constant : public Signal {

public:
    //! Constant signal constructor.
    /*!
      Creates a new constant signal.
      The sampling frequency is not used in this signal type.
      \param c value of the signal.
      \param samplingfreq the sampling frequency.
    */
    constant(double c, double samplingfreq = 0.0);

    virtual double output() override;

    virtual std::string to_string() const override;

protected:
    double c;

};


//!  sin class.
/*!
  This class represents a sinusoidal signal.
*/
class sin : public Signal {

public:
    //! Sinusoidal signal constructor.
    /*!
      Creates a new sinusoidal signal.
      \param ampl amplitude.
      \param freq frequency.
      \param phase phase.
      \param samplingfreq the sampling frequency.
    */
    sin(double ampl, double freq, double phase = 0.0, double samplingfreq = 100.0);

    virtual double output() override;

    virtual std::string to_string() const override;

protected:
    double ampl, freq, phase;

};


//! ramp class
/*!
  This class represents a ramp signal.
*/
class ramp : public Signal {

public:
    //! Ramp signal constructor.
    /*!
      Creates a new ramp signal.
      \param slope the slope of the ramp.
      \param initialvalue initial value of the signal.
      \param starttime time at which the splope should start.
      \param samplingfreq the sampling frequency.
    */
    ramp(double slope, double initialvalue, double starttime = 0.0, double samplingfreq = 100.0);

    virtual double output() override;

    virtual std::string to_string() const override;

protected:
    double slope, initialvalue, starttime;

};

//! rampandhold class
/*!
  This class represents a ramp signal that stops increasing after a certain time.
*/
class rampandhold : public Signal {

public:
    //! Ramp and hold signal constructor.
    /*!
      Creates a new ramp and hold signal.
      \param slope the slope of the ramp.
      \param initialvalue initial value of the signal.
      \param stop time at which the splope should stop increasing.
      \param starttime time at which the splope should start.
      \param samplingfreq the sampling frequency.
    */
    rampandhold(double slope, double initialvalue, double stoptime, double starttime = 0.0, double samplingfreq = 100.0);

    virtual double output() override;

    virtual std::string to_string() const override;

protected:
    double slope, initialvalue, stoptime, starttime, lastvalue;

};


//! Signal switch class.
/*!
  This class implements a switching mechanism between signals that activates after a certain time.
  The first signal will be the ouput before the switching time, the second one after.
*/
class Switch : public Signal {

public:
    //! Switch signal constructor.
    /*!
      Creates a new switch between two signals.
      \param s1 first singnal.
      \param s2 second singal.
      \param switchtime time of the switch.
      \param samplingfreq the sampling frequency.
    */
    Switch(Signal& s1, Signal& s2, double switchtime, double samplingfreq = 100.0);

    virtual double output() override;

    virtual std::string to_string() const override;

    virtual void reset(double time = 0.0) override;

    virtual void setSamplingFreq(double samplingfreq) override;

protected:
    Signal& s1;
    Signal& s2;
    double switchtime;
};


//!  BinaryOperation class.
/*!
  This class is used to represent a binary operation between two signals (eg. +, -).
  The operation behaves like a Signal.
*/
class BinaryOperation : public Signal {

public:
    //! BinaryOperation constructor.
    /*!
      Creates a new binary operation between two signals.
      \param s1 first operand.
      \param s2 second operand.
      \param fun binary operator.
    */
    BinaryOperation(Signal& s1, Signal& s2, std::function<double(double, double)> fun);
//    BinaryOperation(Signal& s1, Signal&& s2, std::function<double(double, double)> fun);
//    BinaryOperation(Signal&& s1, Signal& s2, std::function<double(double, double)> fun);
//    BinaryOperation(Signal&& s1, Signal&& s2, std::function<double(double, double)> fun);

    virtual double output() override;

    virtual std::string to_string() const override;

    virtual void reset(double time = 0.0) override;

    virtual void setSamplingFreq(double samplingfreq) override;

protected:
    Signal& s1;
    Signal& s2;
    std::function<double(double, double)> fun;

};


//! Plus operator between Signals.
/*!
  Creates a BinaryOperation instance representing the sum of two signals.
  \param s1 first operand.
  \param s2 second operand.
  \return BinaryOperation representing the sum.
*/
BinaryOperation operator+(Signal& s1, Signal& s2);

//! Minus operator between Signals.
/*!
  Creates a BinaryOperation instance representing the subtraction of two signals.
  \param s1 first operand.
  \param s2 second operand.
  \return BinaryOperation representing the subtraction.
*/
BinaryOperation operator-(Signal& s1, Signal& s2);

//! Multiplies operator between Signals.
/*!
  Creates a BinaryOperation instance representing the product of the two signals.
  \param s1 first operand.
  \param s2 second operand.
  \return BinaryOperation representing the product.
*/
BinaryOperation operator*(Signal& s1, Signal& s2);

//! Divides operator between Signals.
/*!
  Creates a BinaryOperation instance representing the quotient of the two signals.
  \param s1 first operand.
  \param s2 second operand.
  \return BinaryOperation representing the quotient.
*/
BinaryOperation operator/(Signal& s1, Signal& s2);

    //BinaryOperation operator+(Signal& s1, double s2);
    //BinaryOperation operator+(double s1, Signal& s2);


}


#endif // SIGNALS_H
