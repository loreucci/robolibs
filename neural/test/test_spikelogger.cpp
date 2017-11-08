#include <neural/spikelogger.h>

#include <utilities/utilities.h>
#include <sec/sec.h>


auto gen = Utils::uniformGenerator(1, 0.0, 0.01);
const unsigned int popsize = 10;

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


int main(void) {

    sec::setResultsMode(sec::SINGLE_FILES_MODE);

    TestSpikeSource ss(100.0);

    neural::SpikeLogger logger;

    sec::connect(ss.output1, logger);
    sec::connect(ss.output2, logger);

    sec::main_controller.run(2.0);

    return 0;

}
