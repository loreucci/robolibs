#include "utilities.h"

#include <random>

#include <iostream>
#include <iterator>
#include <fstream>
#include <sstream>

namespace Utils {

// standard generation function
std::function<std::vector<double>(void)> standardGenerator(unsigned int size, double a, double b) {
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

// operations on vectors
double norm(const std::vector<double>& v) {
    double n = std::inner_product(v.begin(), v.end(), v.begin(), 0.0);
    return std::sqrt(n);
}

std::vector<double> normalized(const std::vector<double>& v) {
    std::vector<double> v1 = v;
    double n = norm(v1);
    for (double& d : v1)
        d /= n;
    return v1;
}

std::vector<double> difference(const std::vector<double>& v1, const std::vector<double>& v2) {
    std::vector<double> ret(v1.size());
    for (unsigned int i = 0; i < ret.size(); i++) {
        ret[i] = v1[i] - v2[i];
    }
    return ret;
}

std::vector<double> normalizedDifference(const std::vector<double>& v1, const std::vector<double>& v2) {
    return normalized(difference(v1, v2));
}

double distance(const std::vector<double>& v1, const std::vector<double>& v2) {
    double ret = 0.0;
    for (unsigned int i = 0; i < v1.size(); i++) {
        ret += (v1[i]-v2[i])*(v1[i]-v2[i]);
    }
    return std::sqrt(ret);
}





std::function<std::vector<double>(void)> standardGeneratorPitch(double a, double b) {
    std::mt19937 engine = std::mt19937(static_cast<unsigned long>(time(nullptr)));;
    std::uniform_real_distribution<double> dist(a, b);

    return [engine, dist] () mutable {
        std::vector<double> ret(6, 0);
        ret[4] = dist(engine);
        ret[5] = dist(engine);
        return ret;
    };
}

void printVector(const std::string& pre, const std::vector<double> v, bool endline) {

    std::cout << pre;
    std::copy(v.begin(), v.end(), std::ostream_iterator<double>(std::cout, " "));
    if (endline)
        std::cout << std::endl;

}

double squaredNorm(const std::vector<double>& v) {
    double n = norm(v);
    return n*n;
}

double squaredDistance(const std::vector<double>& v1, const std::vector<double>& v2) {
    return squaredNorm(difference(v1, v2));
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
