/*
 * Copyright (C) 2014-2019 Lorenzo Vannucci
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
 */

/*!
 * \file utilities.h
 * \brief A collection of different utility functions.
 */

#ifndef UTILS_H
#define UTILS_H

#include <functional>
#include <string>
#include <vector>

#include "vector.h"


/*!
 * \brief Marks a paramaters as unused.
 */
#define UTILS_UNUSED(x) [&x]{}()


/*!
 * \brief Namespace containing utility functions.
 *
 * This namespace contains utility functions such as:
 * - numeric constants;
 * - random number generation;
 * - vector operations;
 * - file i/o;
 * - string operations.
 */
namespace Utils {


/*!
 * \brief Greek pi.
 */
const double PI = 3.1415926535;


/*!
 * \brief Degree to radians conversion.
 *
 * \param deg angle in degrees
 * \return angle in radians
 */
double degtorad(double deg);

/*!
 * \brief Radians to degree conversion.
 * \param rad angle in radians
 * \return angle in degrees
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
/*!
 * \brief Uniform random number generator.
 *
 * Returns an uniform random number generator function for Vectors.
 * The elements of the Vector are uniformily chosen in an interval.
 *
 * \param size size of the Vectors that have to be generated
 * \param a minimum value of the interval
 * \param b maximum value of the interval
 * \return the generator function
 */
std::function<Vector(void)> uniformGenerator(unsigned int size, double a = 0.0, double b = 1.0);


/*!
 * \brief Prints a Vector on screen.
 *
 * \param v Vector
 * \param pre string to be printed before the Vector
 * \param endline flag to indicate whether an end of line must be printed
 */
void printVector(const Vector v, const std::string& pre = "", bool endline = true);


/*!
 * \brief Reads a dataset from a delimiter-separated file and store it in a vector of Vectors.
 *
 * \param filename path to file
 * \param del delimiter character
 * \param skiplines number of lines to skip from the beginning of file
 * \return the dataset
 */
std::vector<Vector> readDatasetFromFile(const std::string& filename, char del = ' ', unsigned int skiplines = 0);

/*!
 * \brief Writes a dataset stored as a vector of Vectors in a space separated file.
 * \param filename file path
 * \param data dataset
 */
void saveToFile(const std::string& filename, const std::vector<Vector>& data);


/*!
 * \brief Creates a string out of generic data.
 *
 * Specializations for std::vector or other types are provided.
 *
 * \param t data
 * \param sep unused
 * \return the string obtained from the conversion
 */
template <typename T>
std::string make_string(const T& t, const std::string& sep = "") {
    UTILS_UNUSED(sep);
    return std::to_string(t);
}

/*!
 * \brief Creates a string out of a vector, recursively.
 *
 * \param v vector
 * \param sep separator to be appended after each element, except for last
 * \return the string obtained from the conversion
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

/*!
 * \brief Creates a string out of data (base case for std::string).
 *
 * \param t a string
 * \param sep unused
 * \return the string obtained from the conversion
 */
template <>
std::string make_string(const std::string& t, const std::string& sep);


/*!
 * \brief Replaces a substring with another, inside a string.
 *
 * \param str input string
 * \param target substring to be replaced
 * \param repl replacing substring
 * \return modified string
 */
std::string replace(const std::string& str, const std::string& target, const std::string& repl);


/*!
 * \brief Creates a generic vector from a specified range.
 *
 * T must support the increment operator.
 *
 * \param start lower limit of the range
 * \param end upper limit of the range
 * \param step the step between elements
 * \return the vector containing the range
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
