#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <stdexcept>

template <typename A, typename B>
class Controller {

public:

    Controller(double freq = 0.0) { // freq == 0.0 means inherited

        setFrequency(freq);

    }

    virtual void control(const A& ref) = 0;

    virtual B getOutput() const = 0;

    virtual B controlAndGetOutput(const A& ref) {
        control(ref);
        return getOutput();
    }

    virtual void setFrequency(double freq) {

        if (freq < 0) {
            std::invalid_argument("Controller: frequence must be non-negative.");
        }

        this->freq = freq;
    }

    virtual double getFrequency() const final {
        return freq;
    }

    virtual std::string parameters() const = 0;

protected:
    double freq;

};

#endif // CONTROLLER_H
