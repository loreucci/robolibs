#include "sec/simplenodes.h"
#include "sec/connections.h"
#include "sec/nodelink.h"

#include <vector>

using Dummy1 = sec::OneToOneNode<double, double>;
using Dummy2 = sec::OneToOneNode<std::vector<double>, std::vector<double>>;


int main(void) {

    std::function<double(std::vector<double>)> funct = [](std::vector<double> x){return 10.0*x[0];};

    Dummy1 d1(100.0);
    Dummy2 d2(20.0);

    sec::connect(d1.output, d2.input);
//    sec::connect(d2.output, d1.input, funct);
    sec::connect(d2.output, d1.input, [](std::vector<double> x){return 10.0*x[0];});

    Dummy2* d3 = new Dummy2(100.0);
    Dummy2* d4 = new Dummy2(20.0);
    Dummy2* d5 = new Dummy2(30.0);
    Dummy2 d6(40.0);
    d5->runOnSingleThread();
    d6.runOnSingleThread();

    return 0;

}
