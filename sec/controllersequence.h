#ifndef CONTROLLERSEQUENCE_H
#define CONTROLLERSEQUENCE_H

//#include <stdexcept>
#include <thread>

#include "controller.h"
#include "threadslink.h"
#include "synchronization.h"

template <typename A, typename C, typename B>
class ControllerSequence : public Controller<A, B> {

public:

    ControllerSequence(Controller<A, C>& c1, Controller<C, B>& c2)
        :Controller<A, B>(), c1(c1), c2(c2) {

        if (c1.getFrequency() == 0.0 && c2.getFrequency() != 0.0) {
            single_thread = true;
            this->freq = c2.getFrequency();
            c1.setFrequency(c2.getFrequency());
        } else if (c1.getFrequency() != 0.0 && c2.getFrequency() == 0.0) {
            single_thread = true;
            this->freq = c1.getFrequency();
            c2.setFrequency(c1.getFrequency());
        } else if (c1.getFrequency() == c2.getFrequency()) {
            single_thread = true;
            this->freq = c1.getFrequency();
        } else {
            single_thread = false;

            if (c1.getFrequency() < c2.getFrequency()) {

                exec_first = false;

                auto thread_fun = [=]() {
                    Semaphore s = synchronizer.registerSignal(this->c1.getFrequency());
                    while (true) {
                        s.wait();
                        if (s.shouldquit())
                            break;
                        this->c1.control(this->input_value.getData());
                    }
                };

                std::thread t(thread_fun);
                t.detach();

                this->freq = c2.getFrequency();

            } else {

                exec_first = true;

                auto thread_fun = [=]() {
                    Semaphore s = synchronizer.registerSignal(this->c2.getFrequency());
                    while (true) {
                        s.wait();
                        if (s.shouldquit())
                            break;
                        this->c2.control(this->c1.getOutput());
                    }
                };

                std::thread t(thread_fun);
                t.detach();

                this->freq = c1.getFrequency();

            }

        }

    }

    virtual void control(const A& ref) override{
        if (single_thread)
            c2.control(c1.controlAndGetOutput(ref));
        else {

            if (exec_first) {
                c1.control(ref);
            } else {
                input_value.addData(ref);
                c2.control(c1.getOutput());
            }

        }
    }

    virtual B getOutput() const override{
        return c2.getOutput();
    }

    virtual void setFrequency(double freq) override {
        this->freq = freq;
        c1.setFrequency(freq);
        c2.setFrequency(freq);
    }

    virtual std::string parameters() const override {
        std::string ret = "Sequence of controllers: \n";
        auto par1 = Utils::replace(c1.parameters(), "\n", "\n\t");
        auto par2 = Utils::replace(c2.parameters(), "\n", "\n\t");
        ret += "\t" + par1 + "\n\t" + par2;
        return ret;
    }

protected:
    Controller<A, C>& c1;
    Controller<C, B>& c2;
    bool single_thread, exec_first;
    ThreadsLink<A> input_value;

};


// helper
template <typename A, typename C, typename B>
ControllerSequence<A, C, B> operator+(Controller<A, C>& c1, Controller<C, B>& c2) {
    return ControllerSequence<A, C, B>(c1, c2);
}

template <typename A, typename C, typename B>
ControllerSequence<A, C, B> operator+(Controller<A, C>&& c1, Controller<C, B>&& c2) {
    return ControllerSequence<A, C, B>(c1, c2);
}

template <typename A, typename C, typename B>
ControllerSequence<A, C, B> operator+(Controller<A, C>&& c1, Controller<C, B>& c2) {
    return ControllerSequence<A, C, B>(c1, c2);
}

template <typename A, typename C, typename B>
ControllerSequence<A, C, B> operator+(Controller<A, C>& c1, Controller<C, B>&& c2) {
    return ControllerSequence<A, C, B>(c1, c2);
}

#endif // CONTROLLERSEQUENCE_H
