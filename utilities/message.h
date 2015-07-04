#ifndef MESSAGE_H
#define MESSAGE_H

#include <array>
#include <vector>
#include <initializer_list>

/////////////////////////////////
// needs C++14 extended constexpr

/*#define COUNTER_READ_CRUMB( TAG, RANK, ACC ) counter_crumb( TAG(), constant_index< RANK >(), constant_index< ACC >() )
#define COUNTER_READ( TAG ) COUNTER_READ_CRUMB( TAG, 1, COUNTER_READ_CRUMB( TAG, 2, COUNTER_READ_CRUMB( TAG, 4, COUNTER_READ_CRUMB( TAG, 8, \
  COUNTER_READ_CRUMB( TAG, 16, COUNTER_READ_CRUMB( TAG, 32, COUNTER_READ_CRUMB( TAG, 64, COUNTER_READ_CRUMB( TAG, 128, 0 ) ) ) ) ) ) ) )

#define COUNTER_INC( TAG ) \
constant_index< COUNTER_READ( TAG ) + 1 > \
constexpr counter_crumb( TAG, constant_index< ( COUNTER_READ( TAG ) + 1 ) & ~ COUNTER_READ( TAG ) >, \
  constant_index< ( COUNTER_READ( TAG ) + 1 ) & COUNTER_READ( TAG ) > ) { return {}; }

#define COUNTER_LINK_NAMESPACE( NS ) using NS::counter_crumb;

#define REGISTERED_CLASS( TAG, KEY, NAME ) KEY NAME &register_type( TAG, decltype( COUNTER_READ( TAG ) ) ); \
COUNTER_INC( TAG ) KEY NAME

#include <utility>

template< std::size_t n >
struct constant_index : std::integral_constant< std::size_t, n > {};

template< typename id, std::size_t rank, std::size_t acc >
constexpr constant_index< acc > counter_crumb( id, constant_index< rank >, constant_index< acc > ) { return {}; } // found by ADL via constant_index

struct my_cnt {};

struct IDGEN {

    static constexpr unsigned int gen() {
        COUNTER_INC(my_cnt);
        return COUNTER_READ(my_cnt);
    }
};*/


template <typename T, unsigned int S, unsigned int ID/* = IDGEN::gen()*/>
class Message {

public:

    Message() {
        std::fill(_data.begin(), _data.end(), 0.0);
    }
    Message(const std::vector<T>& v) {
        if (v.size() != size)
            throw std::runtime_error("Message constructor: size mismatch.");
        for (unsigned int i = 0; i < size; i++)
            _data[i] = v[i];
    }
    Message(std::initializer_list<T> l) {
        if (l.size() != size)
            throw std::runtime_error("Message constructor: size mismatch");
        auto v = l.begin();
        unsigned int i = 0;
        for (; i < size; v++, i++) {
            _data[i] = *v;
        }
    }

    T& operator[](unsigned int i) {
        return _data[i];
    }

    const T& operator[](unsigned int i) const {
        return _data[i];
    }

    static const unsigned int size = S;

    operator std::vector<T>() const {
        return to_vector();
    }

    std::vector<T> to_vector() const {
        std::vector<T> v;
        for (auto& it : _data) {
            v.push_back(it);
        }
        return v;
    }

    T* to_ptr() const {
        T* ptr = new T[size];
        for (unsigned int i = 0; i < size; i++)
            ptr[i] = _data[i];
        return ptr;
    }

    const T* data() const {
        return _data.data();
    }

protected:
    static const unsigned int id = ID;
    std::array<T, S> _data;

};




/////////////////////
///
/// common operations

template <typename T, unsigned int S, unsigned int ID>
Message<T, S, ID> operator+(const Message<T, S, ID>& m1, const Message<T, S, ID>& m2) {

    Message<T, S, ID> m;
    for (unsigned int i = 0; i < S; i++) {
        m[i] = m1[i] + m2[i];
    }
    return m;

}

template <typename T, unsigned int S, unsigned int ID>
Message<T, S, ID> operator-(const Message<T, S, ID>& m1, const Message<T, S, ID>& m2) {

    Message<T, S, ID> m;
    for (unsigned int i = 0; i < S; i++) {
        m[i] = m1[i] - m2[i];
    }
    return m;

}

template <typename T, unsigned int S, unsigned int ID>
std::ostream& operator<<(std::ostream& o, const Message<T, S, ID>& m) {

    for (unsigned int i = 0; i < m.size; i++)
        o << m[i] << " ";

    return o;

}

#endif // MESSAGE_H
