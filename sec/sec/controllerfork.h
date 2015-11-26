#ifndef CONTROLLERFORK_H
#define CONTROLLERFORK_H

#include <utility>

#include "controller.h"
#include "threadslink.h"

template <typename A, typename A1, typename B1, typename A2, typename B2, typename B>
class ControllerFork : public Controller<A, B> {

public:
    ControllerFork(Controller<A1, B1>& c1, Controller<A2, B2>& c2)
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
                        this->c1.control(this->input1.getData());
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
                        this->c2.control(this->input2.getData());
                    }
                };

                std::thread t(thread_fun);
                t.detach();

                this->freq = c1.getFrequency();

            }

        }

    }


    virtual void control(const A& ref) override {

        split_data(ref);

        if (single_thread) {
            c1.control(input1.getData());
            c2.control(input2.getData());
        } else {
            if (exec_first) {
                c1.control(input1.getData());
            } else {
                c2.control(input2.getData());
            }
        }

        join_data();

    }

    virtual B getOutput() const override {
        return output;
    }

    virtual void setFrequency(double freq) override {
        this->freq = freq;
        c1.setFrequency(freq);
        c2.setFrequency(freq);
    }

    virtual std::string parameters() const override {
        std::string ret = "Fork of controllers: \n";
        auto par1 = Utils::replace(c1.parameters(), "\n", "\n\t");
        auto par2 = Utils::replace(c2.parameters(), "\n", "\n\t");
        ret += "\t" + par1 + "\n\t" + par2;
        return ret;
    }


    virtual std::pair<A1, A2> split(const A& data) = 0;

    virtual B join(const B1& b1, const B2& b2) = 0;

protected:
    Controller<A1, B1>& c1;
    Controller<A2, B2>& c2;
    bool single_thread, exec_first;
    ThreadsLink<A1> input1;
    ThreadsLink<A2> input2;
    B output;


    virtual void split_data(const A& data) final {
        std::pair<A1, A2> pair = split(data);
        input1.addData(pair.first);
        input2.addData(pair.second);
    }

    virtual void join_data() final {
        output = join(c1.getOutput(), c2.getOutput());
    }



};

#endif // CONTROLLERFORK_H
