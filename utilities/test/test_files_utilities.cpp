#include "utilities/utilities.h"
#include "utilities/vector.h"

#include <iostream>


int main(void) {

    auto d1 = Utils::readDatasetFromFile("testload.txt");
    auto d2 = Utils::readDatasetFromFile("testload.csv", ',', 1);

    std::cout << d1.size() << std::endl;
    for (auto& d : d1) {
        std::cout << d << std::endl;
    }
    std::cout << std::endl;
    std::cout << d2.size() << std::endl;
    for (auto& d : d2) {
        std::cout << d << std::endl;
    }
    std::cout << std::endl;

    return 0;

}
