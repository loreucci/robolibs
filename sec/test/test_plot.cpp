#include <sec/plottingclient.h>
#include <sec/controller.h>
#include <sec/simplesources.h>
#include <sec/datalogger.h>
#include <sec/connections.h>

#include <utilities/vector.h>


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

    TestVecNode tvn(30.0);

//    sec::SinusoidalSource source(10.0, 1.0, 0.0, 0.0, 30.0);
    sec::PlottingClient plots(30.0);

//    sec::SinusoidalSource source2(5.0, 2.0, 0.0, 5.0, 30.0);

//    sec::Logger logger;
//    logger.toggleLogging();

    sec::connect(tvn, &TestVecNode::output, {0, 2}, plots, {"a", "c"}, {[](double x){return x+2;}, [](double x){return x-1;}});

//    sec::connect(source2, &sec::SinusoidalSource::output, plots, "test2", [](double x){return x+5.0;});
//    sec::connect(source, &sec::SinusoidalSource::output, plots, "test");

//    sec::connect(source, &sec::SinusoidalSource::output, logger, "test");
//    sec::connect(source2, &sec::SinusoidalSource::output, logger, "test2");

//    logger.logNodes(sec::main_controller.getAllNodes());

    sec::main_controller.run(3.0);

}
