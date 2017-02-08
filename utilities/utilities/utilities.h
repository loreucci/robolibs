//!  \file utilities.h
/*!
  This file contains the Utils namespace.
*/

#ifndef UTILS_H
#define UTILS_H

#include <functional>
#include <string>
#include <vector>


//!  Mark a paramaters as unused.
#define UTILS_UNUSED(x) [&x]{}()


//!  Utils namespace.
/*!
  This namespace contains utility functions such as:
  - numeric constants;
  - random number generation;
  - vector operations;
  - file i/o;
  - string operations.
*/
namespace Utils {


//! Greek pi.
const double PI = 3.1415926535;

//! Degree to radians conversion.
/*!
  \param deg the angle in degrees.
  \return the angle in radians.
*/
double degtorad(double deg);

//! Radians to degree conversion.
/*!
  \param rad the angle in radians.
  \return the angle in degrees.
*/
double radtodeg(double rad);


//! Uniform random number generator.
/*!
  Returns an uniform random number generator function for vectors.
  The elements of the vector are uniformily chosen in an interval.
  \param size the size of the vectors that have to be generated.
  \param a minimum value of the interval.
  \param b maximum value of the interval.
  \return the generator function.
*/
std::function<std::vector<double>(void)> uniformGenerator(unsigned int size, double a = 0.0, double b = 1.0);


//! Prints a vector on screen.
/*!
  Prints a vector as a sequence of space separated values.
  \param v vector.
  \param pre string to be printented before the vector.
  \param endline flag to indicate whether an end of line must be printed.
*/
void printVector(const std::vector<double> v, const std::string& pre = "", bool endline = true);


//! Loads a dataset from a file.
/*!
  Reads a dataset from a space separated file and store it in a vector of vectors.
  \param filename the file path.
  \return the dataset.
*/
std::vector<std::vector<double>> readDatasetFromFile(const std::string& filename);

//! Saves a dataset in a file.
/*!
  Writes a dataset stored as a vector of vectors in a space separated file.
  \param filename the file path.
  \param data the dataset
*/
void saveToFile(const std::string& filename, const std::vector<std::vector<double>>& data);


//! Creates a string out of data.
/*!
  Creates a string out of a generic data.
  Specific version for std::vector or other types are selected by SFINAE.
  \param t the data.
  \param sep unused.
  \return the string.
*/
template <typename T>
std::string make_string(const T& t, const std::string& sep = "") {
    UTILS_UNUSED(sep);
    return std::to_string(t);
}

//! Creates a string out of a vector.
/*!
  Creates a string out of a vector.
  \param v the vector.
  \param sep separator to be appended after each element, except for last.
  \return the string.
*/
template <typename T>
std::string make_string(const std::vector<T>& v, const std::string& sep = "") {
    if (v.empty())
        return "";
    std::string ret;
    for (unsigned int i = 0; i < v.size()-1; i++) {
        ret += Utils::make_string(v[i], sep) + sep;
    }
    ret += Utils::make_string(v[v.size()-1]);
    return ret;
}

//! Substring replacement.
/*!
  Replaces a substring with another, inside a string.
  \param str input string.
  \param target substring to be replaced.
  \param repl replacing substring.
  \return modified string.
*/
std::string replace(const std::string& str, const std::string& target, const std::string& repl);


//! Create a vector from a specified range.
/*!
  \param start lower limit of the range.
  \param end upper limit of the range.
  \param step the step between elements.
  \return the vector containing the range.
*/
template <typename T>
std::vector<T> range(T start, T end, T step = 1) {

    if (step == 0)
        return std::vector<T>();

    if (end > start && step < 0) {
        return std::vector<T>();
    }

    if (end < start && step > 0) {
        return std::vector<T>();
    }

    std::vector<T> ret;
    for (T v = start; v < end; v += step) {
        ret.push_back(v);
    }
    return ret;
}

}

#endif // UTILS_H
