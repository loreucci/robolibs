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
 * \file vector.h
 * \brief A collection of functions that work with Vectors.
 */

#ifndef VECTOR_H
#define VECTOR_H

#include <vector>
#include <ostream>


namespace Utils {


/*!
 * \brief Vector class.
 *
 * This class is actually a std::vector<double> upon which many operators have been built.
 */
using Vector = std::vector<double>;


/*!
 * \brief Dot product between Vectors.
 *
 * The two operands must have the same size.
 *
 * \param v1 first Vector operand
 * \param v2 second Vector operand
 * \return dot product of v1 and v2
 */
double dot(const Vector& v1, const Vector& v2);


/*!
 * \brief Euclidean norm of a Vector.
 *
 * \param v input Vector
 * \return norm
 */
double norm(const Vector& v);


/*!
 * \brief Squared euclidean norm of a Vector.
 *
 * \param v input Vector
 * \return squared norm
 */
double squaredNorm(const Vector& v);


/*!
 * \brief Computes the normalized Vector, that is the Vector divided by its norm.
 *
 * \param v input Vector
 * \return normalized Vector
 */
Vector normalized(const Vector& v);


/*!
 * \brief Normalized difference between two vectors.
 *
 * The two vectors must be of the same size.
 *
 * \param v1 first Vector operand
 * \param v2 second Vector operand
 * \return normalized element-wise difference.
 */
Vector normalizedDifference(const Vector& v1, const Vector& v2);


/*!
 * \brief Euclidean distance between two Vectors.
 *
 * The two vectors must be of the same size.
 *
 * \param v1 first Vector
 * \param v2 second Vector
 * \return distance
 */
double distance(const Vector& v1, const Vector& v2);


/*!
 * \brief Squared euclidean distance between two Vector.
 *
 * The two Vectors must be of the same size.
 *
 * \param v1 first Vector
 * \param v2 second Vector
 * \return squared distance
 */
double squaredDistance(const Vector& v1, const Vector& v2);


/*!
 * \brief Creates a new Vector as a subvector of another Vector.
 *
 * The new Vector will be equal to Vector[start:end-1].
 *
 * \param v source Vector
 * \param start first index
 * \param end index after the last one
 * \return subvector
 */
Vector subvector(const Vector& v, unsigned int start, unsigned int end);


/*!
 * \brief Creates a new Vector that is the join of two Vectors.
 *
 * The second one will be appended at the end of the first.
 *
 * \param v1 first Vector
 * \param v2 second Vector
 * \return joined vectors
 */
Vector joinVectors(const Vector& v1, const Vector& v2);


/*!
 * \brief Scales elements of the Vector.
 *
 * Scale the Vector elements from [min(v); max(v)] to [outmin; outmax].
 *
 * \param v source Vector
 * \param outmin minimum scaled value
 * \param outmax maximum scaled valu.
 * \return range normalized vector
 */
Vector rangeNormalization(const Vector& v, double outmin = 0, double outmax = 1);


/*!
 * \brief Scales elements of the Vector.
 *
 * Scale the Vector element i from [inmin(i); inmax(i)] to [outmin; outmax].
 *
 * \param v source Vector
 * \param inmin minimum value for each element
 * \param inmax maximum value for each element
 * \param outmin minimum scaled value
 * \param outmax maximum scaled value
 * \return range normalized vector
 */
Vector rangeNormalization(const Vector& v, const Vector& inmin, const Vector& inmax, double outmin, double outmax);


/*!
 * \brief Scales elements of the Vector.
 *
 * Scale the Vector element i from [inmin, inmax] to [outmin(i); outmax(i)].
 *
 * \param v source Vector
 * \param inmin minimum value
 * \param inmax maximum value
 * \param outmin minimum scaled value for each element
 * \param outmax maximum scaled value for each element
 * \return range normalized vector
 */
Vector rangeNormalization(const Vector& v, double inmin, double inmax, const Vector& outmin, const Vector& outmax);

}


/*!
 * \brief Element-wise plus operator between Vectors.
 *
 * The two operands must have the same size.
 *
 * \param v1 first Vector operand
 * \param v2 second Vector operand
 * \return element-wise sum of v1 and v2
 */
Utils::Vector operator+(const Utils::Vector& v1, const Utils::Vector& v2);


/*!
 * \brief Element-wise subtraction operator between Vectors.
 *
 * The two operands must have the same size.
 *
 * \param v1 first Vector operand
 * \param v2 second Vector operand
 * \return element-wise subtraction of v2 from v1
 */
Utils::Vector operator-(const Utils::Vector& v1, const Utils::Vector& v2);


/*!
 * \brief Unary minus operator on Vectors.
 *
 * \param v the Vector operand
 * \return element-wise unary minus of v
 */
Utils::Vector operator-(const Utils::Vector& v);


/*!
 * \brief Scalar product between a double and a Vector.
 *
 * \param k scalar value
 * \param v vector
 * \return element-wise product of k and v
 */
Utils::Vector operator*(double k, const Utils::Vector& v);
/*!
 * @copydoc operator*(double k, const Utils::Vector& v)
 */
Utils::Vector operator*(const Utils::Vector& v, double k);


/*!
 * \brief Scalar division between a Vector and a double.
 *
 * \param v vector
 * \param k scalar value
 * \return element-wise product of v and 1/k
 */
Utils::Vector operator/(const Utils::Vector& v, double k);


/*!
 * \brief Insertion operators on Vectors.
 *
 * The elements will be separated by spaces.
 *
 * \param o output stream
 * \param v Vector to be inserted into the stream
 * \return output stream
 */
std::ostream& operator<<(std::ostream& o, const Utils::Vector& v);


#endif // VECTOR_H
