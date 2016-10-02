#include "vector.h"

#include <stdexcept>
#include <numeric>
#include <cmath>


namespace Utils {


Vector operator+(const Vector& v1, const Vector& v2) {

    if (v1.size() != v2.size())
        throw std::invalid_argument("Vector sizes mismatch.");

    unsigned int sz = v1.size();
    Vector ret(sz);
    for (unsigned int i = 0; i < sz; i++)
        ret[i] = v1[i] + v2[i];

    return ret;

}

Vector operator-(const Vector& v1, const Vector& v2) {

    if (v1.size() != v2.size())
        throw std::invalid_argument("Vector sizes mismatch.");

    unsigned int sz = v1.size();
    Vector ret(sz);
    for (unsigned int i = 0; i < sz; i++)
        ret[i] = v1[i] - v2[i];

    return ret;

}

Vector operator-(const Vector& v) {
    unsigned int sz = v.size();
    Vector ret(sz);
    for (unsigned int i = 0; i < sz; i++)
        ret[i] = -v[i];
    return ret;
}

Vector operator*(double k, const Vector& v) {
    unsigned int sz = v.size();
    Vector ret(sz);
    for (unsigned int i = 0; i < sz; i++)
        ret[i] = k*v[i];
    return ret;
}

Vector operator*(const Vector& v, double k) {
    return k*v;
}

Vector operator/(const Vector& v, double k) {
    unsigned int sz = v.size();
    Vector ret(sz);
    for (unsigned int i = 0; i < sz; i++)
        ret[i] = v[i]/k;
    return ret;
}

double dot(const Vector& v1, const Vector& v2) {

    if (v1.size() != v2.size())
        throw std::invalid_argument("Vector sizes mismatch.");

    return std::inner_product(v1.begin(), v2.end(), v1.begin(), 0.0);;

}

double norm(const Vector& v) {
    return std::sqrt(dot(v, v));;
}

double squaredNorm(const Vector& v) {
    return dot(v, v);
}

Vector normalized(const Vector& v) {
    return v / norm(v);
}

Vector normalizedDifference(const Vector& v1, const Vector& v2) {
    return normalized(v1-v2);
}

double distance(const Vector& v1, const Vector& v2) {
    return norm(v1-v2);
}

double squaredDistance(const Vector& v1, const Vector& v2) {
    double d = norm(v1-v2);
    return d*d;
}


}
