//!  \file message.h
/*!
  This file contains the Message class and the associated
  arithmetic operators.
*/

#ifndef MESSAGE_H
#define MESSAGE_H

#include <array>
#include <vector>
#include <initializer_list>

//!  Message class.
/*!
  This class represent a container for a message that can be exchanged.
  Each Message type has:
  - a type T of the data inside the container;
  - a statically determined size S;
  - an unique ID.
*/
template <typename T, unsigned int S, unsigned int ID>
class Message {

public:

    //! Message constructor.
    /*!
      Creates an empty Message (i.e. filled with T()).
    */
    Message() {
        std::fill(_data.begin(), _data.end(), T());
    }

    //! Message Constructor.
    /*!
      Creates a Message from a std::vector by copying its elements.
      The std::vector size and the Message size should match.
    */
    Message(const std::vector<T>& v) {
        if (v.size() != size)
            throw std::runtime_error("Message constructor: size mismatch.");
        for (unsigned int i = 0; i < size; i++)
            _data[i] = v[i];
    }

    //! Message Constructor.
    /*!
      Creates a Message from a std::initializer_list.
      The std::initializer_list and Message size should match.
    */
    Message(std::initializer_list<T> l) {
        if (l.size() != size)
            throw std::runtime_error("Message constructor: size mismatch");
        auto v = l.begin();
        unsigned int i = 0;
        for (; i < size; v++, i++) {
            _data[i] = *v;
        }
    }

    //! Subscript operator.
    /*!
      \param i index.
      \return reference to the i-th element.
    */
    T& operator[](unsigned int i) {
        return _data[i];
    }

    //! Constant subscript operator.
    /*!
      \param i index.
      \return constant reference to the i-th element.
    */
    const T& operator[](unsigned int i) const {
        return _data[i];
    }

    //! Casts the Message into a std::vector containing the same elements.
    /*!
      \return std::vector containing the same elements.
    */
    operator std::vector<T>() const {
        return to_vector();
    }

    //! Returns a std::vector containing the same elements.
    /*!
      \return std::vector containing the same elements.
    */
    std::vector<T> to_vector() const {
        std::vector<T> v;
        for (auto& it : _data) {
            v.push_back(it);
        }
        return v;
    }

    //! Returns a C-style array containing the same elements.
    /*!
      Memory is allocated for the array.
      \return pointer to the array.
    */
    T* to_ptr() const {
        T* ptr = new T[size];
        for (unsigned int i = 0; i < size; i++)
            ptr[i] = _data[i];
        return ptr;
    }

    //! Underlying data of the message.
    /*!
      \return pointer to the underlying data.
    */
    const T* data() const {
        return _data.data();
    }

    //! Size of the Message.
    static const unsigned int size = S;

    //! ID of the message.
    static const unsigned int id = ID;

protected:
    std::array<T, S> _data;

};


//! Plus operator between Messages.
/*!
  The two operands must have the same element type, the same size and the same ID.
  \param m1 first operand.
  \param m2 second operand.
  \return element-wise sum of m1 and m2.
*/
template <typename T, unsigned int S, unsigned int ID>
Message<T, S, ID> operator+(const Message<T, S, ID>& m1, const Message<T, S, ID>& m2) {

    Message<T, S, ID> m;
    for (unsigned int i = 0; i < S; i++) {
        m[i] = m1[i] + m2[i];
    }
    return m;

}

//! Subtraction operator between Messages.
/*!
  The two operands must have the same element type, the same size and the same ID.
  \param m1 first operand.
  \param m2 second operand.
  \return element-wise subtraction of m2 from m1.
*/
template <typename T, unsigned int S, unsigned int ID>
Message<T, S, ID> operator-(const Message<T, S, ID>& m1, const Message<T, S, ID>& m2) {

    Message<T, S, ID> m;
    for (unsigned int i = 0; i < S; i++) {
        m[i] = m1[i] - m2[i];
    }
    return m;

}

//! Unary minus operator on Messages.
/*!
  \param m the operand.
  \return element-wise unary minus of m.
*/
template <typename T, unsigned int S, unsigned int ID>
Message<T, S, ID> operator-(const Message<T, S, ID>& m) {

    Message<T, S, ID> ret;
    for (unsigned int i = 0; i < S; i++) {
        ret[i] = -m[i];
    }
    return ret;

}

//! Insertion operator.
/*!
  \param o output stream.
  \param m message to be inserted into the stream.
  \return output stream.
*/
template <typename T, unsigned int S, unsigned int ID>
std::ostream& operator<<(std::ostream& o, const Message<T, S, ID>& m) {

    for (unsigned int i = 0; i < m.size; i++)
        o << m[i] << " ";

    return o;

}

#endif // MESSAGE_H
