//!  \file vector.h
/*!
  This file contains functions for vector arithmetic.
*/


#ifndef VECTOR_H
#define VECTOR_H

#include <vector>
#include <ostream>

namespace Utils {


//! Vector class
using Vector = std::vector<double>;

//! Dot product.
/*!
  The two operands must have the same size.
  \param v1 first operand.
  \param v2 second operand.
  \return Dot product of v1 and v2.
*/
double dot(const Vector& v1, const Vector& v2);

//! Vector norm.
/*!
  Euclidean norm of a Vector.
  \param v input Vector.
  \return norm.
*/
double norm(const Vector& v);

//! Squared vector norm.
/*!
  Squared euclidean norm of a Vector.
  \param v input Vector.
  \return squared norm.
*/
double squaredNorm(const Vector& v);

//! Vector normalization.
/*!
  Computes the normalized Vector.
  \param v input Vector.
  \return normalized Vector.
*/
Vector normalized(const Vector& v);

//! Vector normalized difference.
/*!
  Normalized difference between two vectors.
  The two vectors must be of the same size.
  \param v1 first operand.
  \param v2 second operand.
  \return normalized element-wise difference.
*/
Vector normalizedDifference(const Vector& v1, const Vector& v2);

//! Euclidean distance.
/*!
  Euclidean distance between two Vectors.
  The two vectors must be of the same size.
  \param v1 first Vector.
  \param v2 second Vector.
  \return distance.
*/
double distance(const Vector& v1, const Vector& v2);

//! Squared distance.
/*!
  Squared euclidean distance between two Vector.
  The two Vectors must be of the same size.
  \param v1 first Vector.
  \param v2 second Vector.
  \return squared distance.
*/
double squaredDistance(const Vector& v1, const Vector& v2);


//! Subvector extraction.
/*!
  Create a new Vector as a subvector of another Vector.
  The new Vector will be equal to Vector[start:end-1].
  \param v source Vector.
  \param start first index.
  \param end index after the last one.
  \return subvector.
*/
Vector subvector(const Vector& v, unsigned int start, unsigned int end);


//! Vector range normalization
/*!
  Scale the Vector elements from [min(v); max(v)] to [outmin; outmax].
  \param v source Vector.
  \param outmin minimum scaled value.
  \param outmax maximum scaled value.
  \return range normalized vector.
*/
Vector rangeNormalization(const Vector& v, double outmin = 0, double outmax = 1);

//! Vector range normalization
/*!
  Scale the Vector element i from [inmin(i); inmax(i)] to [outmin; outmax].
  \param v source Vector.
  \param inmin minimum value for each element.
  \param inmax maximum value for each element.
  \param outmin minimum scaled value.
  \param outmax maximum scaled value.
  \return range normalized vector.
*/
Vector rangeNormalization(const Vector& v, const Vector& inmin, const Vector& inmax, double outmin, double outmax);

//! Vector range normalization
/*!
  Scale the Vector element i from [inmin, inmax] to [outmin(i); outmax(i)].
  \param v source Vector.
  \param inmin minimum value.
  \param inmax maximum value.
  \param outmin minimum scaled value for each element.
  \param outmax maximum scaled value for each element.
  \return range normalized vector.
*/
Vector rangeNormalization(const Vector& v, double inmin, double inmax, const Vector& outmin, const Vector& outmax);

}


//! Plus operator between Vectors.
/*!
  The two operands must have the same size.
  \param v1 first operand.
  \param v2 second operand.
  \return element-wise sum of v1 and v2.
*/
Utils::Vector operator+(const Utils::Vector& v1, const Utils::Vector& v2);

//! Subtraction operator between Vectors.
/*!
  The two operands must have the same size.
  \param v1 first operand.
  \param v2 second operand.
  \return element-wise subtraction of v2 from v1.
*/
Utils::Vector operator-(const Utils::Vector& v1, const Utils::Vector& v2);

//! Unary minus operator on Vectors.
/*!
  \param v the operand.
  \return element-wise unary minus of v.
*/
Utils::Vector operator-(const Utils::Vector& v);


//! Scalar product between a double and a Vector.
/*!
  \param k scalar value.
  \param v vector.
  \return element-wise product of k and v.
*/
Utils::Vector operator*(double k, const Utils::Vector& v);

//! Scalar product between a double and a Vector.
/*!
  \param v vector.
  \param k scalar value.
  \return element-wise product of k and v.
*/
Utils::Vector operator*(const Utils::Vector& v, double k);

//! Scalar division between a Vector and a double.
/*!
  \param v vector.
  \param k scalar value.
  \return element-wise product of v and 1/k.
*/
Utils::Vector operator/(const Utils::Vector& v, double k);

//! Insertion operator on Vectors.
/*!
  The elements will be separated by spaces.
  \param o output stream.
  \param v Vector to be inserted into the stream.
  \return output stream.
*/
std::ostream& operator<<(std::ostream& o, const Utils::Vector& v);


#endif // VECTOR_H
