#include "utilities.h"

#include <random>
#include <iostream>
#include <iterator>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <ctime>


namespace Utils {

// standard generation function
std::function<std::vector<double>(void)> uniformGenerator(unsigned int size, double a, double b) {
    std::mt19937 engine = std::mt19937(static_cast<unsigned long>(time(nullptr)));;
    std::uniform_real_distribution<double> dist(a, b);

    return [engine, dist, size] () mutable {
        std::vector<double> ret(size);
        for (unsigned int i = 0; i < size; i++) {
            ret[i] = dist(engine);
        }
        return ret;
    };

}


void printVector(const std::vector<double> v, const std::string& pre, bool endline) {

    std::cout << pre;
    std::copy(v.begin(), v.end(), std::ostream_iterator<double>(std::cout, " "));
    if (endline)
        std::cout << std::endl;

}


std::vector<std::vector<double> > readDatasetFromFile(const std::string& filename) {

    std::ifstream file(filename, std::ios_base::in);

    std::vector<std::vector<double>> set;

    while (file.good() && !file.eof()) {
        std::string line;
        std::vector<double> p;
        std::getline(file, line);
        if ( file.eof() ) {
            break;
        }

        std::stringstream ss(line);

        double d;
        while (!ss.eof()) {
            ss >> d;
            if (ss.eof())
                break;
            p.push_back(d);
        }

        set.push_back(p);

    }

    file.close();

    return set;

}

void saveToFile(const std::string& filename, const std::vector<std::vector<double>>& data) {

    std::ofstream out(filename);

    for (auto& v : data) {
        std::copy(v.begin(), v.end(), std::ostream_iterator<double>(out, " "));
        out << std::endl;
    }

    out.close();

}

std::string replace(const std::string& str, const std::string& target, const std::string& repl) {

    if (target.length() == 0)
        return str;

    if (str.length() == 0)
        return str;

    std::string ret = str;
    size_t idx = 0;

    for (;;) {
        idx = ret.find(target, idx);
        if (idx == std::string::npos)
            break;

        ret.replace(idx, target.length(), repl);
        idx += repl.length();
    }

    return ret;

}

}
