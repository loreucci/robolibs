#include <sec/sec.h>

#include <iostream>


class TestNode : public sec::Node {

public:
    TestNode(int i, double freq = 100.0)
        :sec::Node(freq), i(i) {}

    virtual void refreshInputs() override {}

    virtual bool connected() const override {
        return true;
    }

    virtual void execute() override {
        std::cout << i << std::endl;
    }

    virtual std::string parameters() const override {
        return "Test node " + std::to_string(i) + "\n and some other stuff that should be cut off";
    }

private:
    int i;

};


int main(void) {

    sec::setVerbose();

//    std::cout << sec::isVerbose() << std::endl;

    TestNode n0(0);
    TestNode n1(1);
    TestNode n2(2);
    TestNode n3(3);
    TestNode n4(4);

    n4.setFrequency(20.0);
    n3.setFrequency(50.0);
    n2.setFrequency(20.0);
    n1.setFrequency(50.0);
    n0.setFrequency(20.0);

    sec::run(0.1);

    return 0;

}
