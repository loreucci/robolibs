#ifndef COMMONCONTROLLERS_H
#define COMMONCONTROLLERS_H

#include <deque>

#include "basiccontroller.h"

template <typename A>
class Delay : public BasicController<A, A> {

public:
    Delay(double secs, A initial_value = A(), double freq = 0.0)
        :BasicController<A, A>(freq), secs(secs), initial_value(initial_value) {}

    virtual std::string parameters() const override {
        return "Delay of " + std::to_string(secs) + " seconds @" + std::to_string(this->freq) + "Hz";
    }

protected:
    double secs;
    A initial_value;

    std::deque<A> q;
    unsigned int count = 0;

    virtual A ctrl(const A& ref) override {

        if (secs == 0)
            return ref;

        if (count < secs*this->freq) {
            q.push_back(ref);
            count++;
            return initial_value;
        }
        A ret = q.front();
        q.pop_front();
        q.push_back(ref);
        return ret;
    }

};

#endif // COMMONCONTROLLERS_H
