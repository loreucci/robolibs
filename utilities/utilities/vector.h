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
//class Vector : public std::vector<double> {

//public:

//    //! Message constructor.
//    /*!
//      Inherited by std::vector.
//    */
//    using std::vector<double>::vector;

//    //! Vector norm.
//    /*!
//      Euclidean norm of a Vector.
//      \return norm.
//    */
//    double norm() const;

//    //! Squared Vector norm.
//    /*!
//      Squared euclidean norm of a Vector.
//      \return squared norm.
//    */
//    double squaredNorm() const;

//    //! Vector normalization.
//    /*!
//      Computes the normalized Vector, dividing it by its norm.
//      \return normalized Vector.
//    */
//    Vector normalized() const;

//};


//! Plus operator between Vectors.
/*!
  The two operands must have the same size.
  \param v1 first operand.
  \param v2 second operand.
  \return element-wise sum of v1 and v2.
*/
Vector operator+(const Vector& v1, const Vector& v2);

//! Subtraction operator between Vectors.
/*!
  The two operands must have the same size.
  \param v1 first operand.
  \param v2 second operand.
  \return element-wise subtraction of v2 from v1.
*/
Vector operator-(const Vector& v1, const Vector& v2);

//! Unary minus operator on Vectors.
/*!
  \param v the operand.
  \return element-wise unary minus of v.
*/
Vector operator-(const Vector& v);


//! Scalar product between a double and a Vector.
/*!
  \param k scalar value.
  \param v vector.
  \return element-wise product of k and v.
*/
Vector operator*(double k, const Vector& v);

//! Scalar product between a double and a Vector.
/*!
  \param v vector.
  \param k scalar value.
  \return element-wise product of k and v.
*/
Vector operator*(const Vector& v, double k);

//! Scalar division between a Vector and a double.
/*!
  \param v vector.
  \param k scalar value.
  \return element-wise product of v and 1/k.
*/
Vector operator/(const Vector& v, double k);

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

//! Insertion operator on Vectors.
/*!
  \param o output stream.
  \param v Vector to be inserted into the stream.
  \return output stream.
*/
std::ostream& operator<<(std::ostream& o, const Vector& v) {

    for (unsigned int i = 0; i < v.size(); i++)
        o << v[i] << " ";

    return o;

}

}

#endif // VECTOR_H
