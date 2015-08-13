#ifndef UTILS_H
#define UTILS_H

#include <functional>
#include <string>
#include <vector>

#include "message.h"

namespace Utils {

// standard generation function
std::function<std::vector<double>(void)> standardGenerator(unsigned int size, double a = 0.0, double b = 1.0);
std::function<std::vector<double>(void)> standardGeneratorPitch(double a = 0.0, double b = 1.0);


// operations on vectors

// ||v||
double norm(const std::vector<double>& v);

// ||v||^2
double squaredNorm(const std::vector<double>& v);

// v/||v||
std::vector<double> normalized(const std::vector<double>& v);

// v1-v2
std::vector<double> difference(const std::vector<double>& v1, const std::vector<double>& v2);

// v1-v2/||v1-v2||
std::vector<double> normalizedDifference(const std::vector<double>& v1, const std::vector<double>& v2);

// ||v1-v2||
double distance(const std::vector<double>& v1, const std::vector<double>& v2);

// ||v1-v2||^2
double squaredDistance(const std::vector<double>& v1, const std::vector<double>& v2);

const double PI = 3.1415926535;

// vector printing
void printVector(const std::string& pre, const std::vector<double> v, bool endline = true);


// read a dataset from file
std::vector<std::vector<double>> readDatasetFromFile(const std::string& filename);

// print data to file
void saveToFile(const std::string& filename, const std::vector<std::vector<double>>& data);


// string makers
template <typename T>
std::string make_string(T& t, const std::string& sep) {
    return std::to_string(t) + sep;
}

template <typename T>
std::string make_string(std::vector<T>& t, const std::string& sep) {
    std::string ret;
    for (T& v : t) {
        ret += Utils::make_string(v, sep);
    }
    return ret;
}

template <typename T, unsigned int S, unsigned int ID>
std::string make_string(Message<T, S, ID>& m, const std::string& sep) {
    std::string ret;
    for (unsigned int i = 0; i < m.size; i++) {
        ret += Utils::make_string(m[i], sep);
    }
    return ret;
}


// replace substring with another in string
std::string replace(const std::string& str, const std::string& target, const std::string& repl);


// apply masks to messages
template <typename T, unsigned int S, unsigned int ID1, unsigned int ID2>
Message<T, S, ID1> apply_mask(const Message<T, S, ID1>& msg, const Message<T, S, ID2>& mask) {
    Message<T, S, ID1> ret;
    for (unsigned int i = 0; i < S; i++) {
        ret[i] = msg[i] * mask[i];
    }
    return ret;
}

}

#endif // UTILS_H
