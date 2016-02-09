#include <sec/simplenodes.h>
#include <sec/connections.h>


using Dummy1 = sec::OneToOneNode<double, double>;
using Dummy2 = sec::OneToOneNode<std::vector<double>, std::vector<double>>;


int main(void) {

    Dummy1 d1;
    Dummy2 d2;

    sec::connect(d1, &Dummy1::output, d2, &Dummy2::input);
    sec::connect(d2, &Dummy2::output, d1, &Dummy1::input);

    return 0;

}
