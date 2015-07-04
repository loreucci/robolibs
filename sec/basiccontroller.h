#ifndef BASICCONTROLLER_H
#define BASICCONTROLLER_H

#include "controller.h"
#include "threadslink.h"

template <typename A, typename B>
class BasicController : public Controller<A, B> {

public:
    BasicController(double freq = 0.0)
        :Controller<A, B>(freq){}

    virtual void control(const A& ref) final override {
        B b = ctrl(ref);
        link.addData(b);
    }

    virtual B getOutput() const final override {
        return link.getData();
    }


protected:
    ThreadsLink<B> link;

    virtual B ctrl(const A& ref) = 0;

};

#endif // BASICCONTROLLER_H
