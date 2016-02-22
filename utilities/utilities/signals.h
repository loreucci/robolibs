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
  This class represent a constant signal.
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
  This class represent a sinusoidal signal.
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
