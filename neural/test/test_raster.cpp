#include "neural/rasterclient.h"
#include "neural/spikes.h"

#include <sec/controller.h>
#include <sec/node.h>
#include <sec/nodelink.h>
#include <sec/synchronization.h>
#include <sec/source.h>

#include <utilities/utilities.h>

auto gen = Utils::uniformGenerator(1, 0.0, 0.005);
const unsigned int popsize = 25;


class TestSpikeSource : public sec::Node {

public:
    using Node::Node;

    virtual void refreshInputs() override {}

    virtual bool connected() const override {
        return true;
    }

    virtual void execute() override {

        neural::SpikeData ret1, ret2;

        for (unsigned int i = 0; i < popsize; i++) {
            ret1.push_back({i, sec::synchronizer.getTime() + gen()[0]});
            ret2.push_back({i+popsize, sec::synchronizer.getTime() + gen()[0]});
        }

        output1 = ret1;
        output2 = ret2;

    }

    virtual std::string parameters() const override {
        return "";
    }

    neural::SpikeNodeOut output1, output2;

};


class SpikeGenerator : public sec::Source {

public:
    SpikeGenerator(unsigned int n, double rate, double freq = 0.0)
        :sec::Source(freq), n(n) {

        interval = 1.0/rate;

        stime = 0.0;

    }

    virtual void execute() override {

        neural::SpikeData ret;

        double curtime = sec::synchronizer.getTime();


        while (stime + interval < curtime + 1.0/freq) {
            stime += interval;
            for (unsigned int i = 0; i < n; i++) {
                ret.push_back({i, stime});
            }
        }

        output = ret;

    }

    virtual std::string parameters() const override {
        return "";
    }

    neural::SpikeNodeOut output;

protected:
    unsigned int n, count;
    double interval, stime;

};


int main() {

    neural::RasterClient rclient(50.0);

//    TestSpikeSource tss(200.0);

//    sec::connect(tss, &TestSpikeSource::output1, rclient, "pop1");
//    sec::connect(tss, &TestSpikeSource::output2, rclient, "pop2");

    SpikeGenerator sg(10, 400.0, 100.0);
    sec::connect(sg, &SpikeGenerator::output, rclient, "pop");

    sec::main_controller.run();

    return 0;

}
