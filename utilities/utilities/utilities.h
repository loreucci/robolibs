//!  \file utilities.h
/*!
  This file contains the Utils namespace.
*/

#ifndef UTILS_H
#define UTILS_H

#include <functional>
#include <string>
#include <vector>

#include "message.h"


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


// operations on vectors

//! Vector norm.
/*!
  Euclidean norm of a vector.
  \param v input vector.
  \return norm.
*/
double norm(const std::vector<double>& v);

//! Squared vector norm.
/*!
  Squared euclidean norm of a vector.
  \param v input vector.
  \return squared norm.
*/
double squaredNorm(const std::vector<double>& v);

//! Vector normalization.
/*!
  Computes the normalized vector.
  \param v input vector.
  \return normalized vector.
*/
std::vector<double> normalized(const std::vector<double>& v);

//! Vector difference.
/*!
  Difference between two vectors.
  The two vectors must be of the same size.
  \param v1 first operand.
  \param v2 second operand.
  \return element-wise difference.
*/
std::vector<double> difference(const std::vector<double>& v1, const std::vector<double>& v2);

//! Vector normalized difference.
/*!
  Normalized difference between two vectors.
  The two vectors must be of the same size.
  \param v1 first operand.
  \param v2 second operand.
  \return normalized element-wise difference.
*/
std::vector<double> normalizedDifference(const std::vector<double>& v1, const std::vector<double>& v2);

//! Euclidean distance.
/*!
  Euclidean distance between two vectors.
  The two vectors must be of the same size.
  \param v1 first vector.
  \param v2 second vector.
  \return distance.
*/
double distance(const std::vector<double>& v1, const std::vector<double>& v2);

//! Squared distance.
/*!
  Squared euclidean distance between two vectors.
  The two vectors must be of the same size.
  \param v1 first vector.
  \param v2 second vector.
  \return squared distance.
*/
double squaredDistance(const std::vector<double>& v1, const std::vector<double>& v2);

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
  Specific version for std::vector and Message are selected by SFINAE.
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
        ret += Utils::make_string(v[i], sep);
    }
    ret += Utils::make_string(v[v.size()-1]);
    return ret;
}

//! Creates a string out of a Message.
/*!
  Creates a string out of a Message.
  \param m the Message.
  \param sep separator to be appended after each element, except for last.
  \return the string.
*/
template <typename T, unsigned int S, unsigned int ID>
std::string make_string(const Message<T, S, ID>& m, const std::string& sep = "") {
    if (m.size == 0)
        return "";
    std::string ret;
    for (unsigned int i = 0; i < m.size-1; i++) {
        ret += Utils::make_string(m[i], sep);
    }
    ret += Utils::make_string(m[m.size-1]);
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

//! Message mask application.
/*!
  Applies a mask to a Message. The mask is another Message of the same type and size.
  \param msg input message.
  \param mask mask.
  \return element-wise application of the mask to the Message.
*/
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