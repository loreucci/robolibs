#include <sec/plottingclient.h>
#include <sec/controller.h>
#include <sec/simplesources.h>
#include <sec/datalogger.h>
#include <sec/connections.h>

#include <utilities/vector.h>
#include <utilities/signals.h>

#include <cmath>


class TestVecNode : public sec::Node {

public:

    TestVecNode(double freq = 0.0)
        :sec::Node(freq) {}

    virtual void refreshInputs() override {}

    virtual bool connected() const override {
        return true;
    }

    virtual void execute() override {
        Utils::Vector ret(3);
        ret[0] = -1;
        ret[1] = 2;
        ret[2] = 5;
        output = ret;
    }

    virtual std::string parameters() const override {
        return "TestVecNode";
    }

    sec::NodeOut<Utils::Vector> output;


};



int main(void) {

    Signals::Signal testsig([](double t) { return t == 0.0 ? 1.0 : 10*std::sin(3.14*2*t)/t;}, "[sin(x)/x]", 50.0);

    sec::SignalSource ss(testsig, 50.0);


    TestVecNode tvn(30.0);

    auto ssource = Signals::sin(1.0, 1.0, 0.0, 50.0);
    sec::SignalSource source(ssource, 50.0);
    sec::PlottingClient plots(50.0);

    auto ssource2 = Signals::sin(5.0, 2.0, 0.0, 50.0) + 5.0;
    sec::SignalSource source2(ssource2, 30.0);

    sec::connect(ss.output, plots, "test1");

    sec::connect(tvn.output, {0, 2}, plots, {"a", "c"}, {[](double x){return x-2;}, [](double x){return x-1;}});

    sec::connect(source2.output, plots, "test2", [](double x){return x+5.0;});
    sec::connect(source.output, plots, "test2");

    sec::main_controller.run();

}
