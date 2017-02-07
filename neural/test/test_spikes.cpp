#include "neural/rasterclient.h"
#include "neural/spikes.h"

#include <sec/controller.h>
#include <sec/node.h>
#include <sec/nodelink.h>
#include <sec/synchronization.h>

#include <utilities/utilities.h>

auto gen = Utils::uniformGenerator(1, 0.0, 0.01);
const unsigned int popsize = 50;


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

class TestSpikeSink : public sec::Node {

public:
    using Node::Node;

    virtual void refreshInputs() override {
        input1.refreshData();
        input2.refreshData();
    }

    virtual bool connected() const override {
        return input1.isConnected() && input2.isConnected();
    }

    virtual void execute() override {

    }

    virtual std::string parameters() const override {
        return "";
    }

    neural::SpikeNodeIn input1, input2;

};



int main() {

    TestSpikeSource tss(100.0);
    TestSpikeSink sink(100.0);

    sec::connect(tss, &TestSpikeSource::output1, sink, &TestSpikeSink::input1);
    sec::connect(tss, &TestSpikeSource::output2, sink, &TestSpikeSink::input2);

    sec::main_controller.run(5.0);

    return 0;

}
