#include <iterator>
#include <iostream>

#include "sec/simplesources.h"
#include "sec/connections.h"

int main(void) {

    sec::FileSource fs("filesource.txt", 2);

//    while (fs.valid()) {
    for (unsigned int i = 0; i < 20; i++) {
        fs.execute();
        auto d = fs.output.getData().first;
        std::copy(d.begin(), d.end(), std::ostream_iterator<double>(std::cout, " "));
        std::cout << std::endl;
    }

    return 0;

}
