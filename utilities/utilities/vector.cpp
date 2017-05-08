#include "vector.h"

#include <stdexcept>
#include <numeric>
#include <cmath>
#include <algorithm>


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

    return std::inner_product(v1.begin(), v1.end(), v2.begin(), 0.0);;

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

Vector subvector(const Vector& v, unsigned int start, unsigned int end) {
    Vector ret;

    // max end should be v.size
    if (end > v.size())
        end = v.size();

    // empty case
    if (start >= end) {
        return ret;
    }

    for (unsigned int i = start; i < end; i++)
        ret.push_back(v[i]);

    return ret;
}

std::ostream&operator<<(std::ostream& o, const Vector& v) {

    for (unsigned int i = 0; i < v.size(); i++)
        o << v[i] << " ";

    return o;

}

Vector rangeNormalization(const Vector& v, double outmin, double outmax) {

    double inmax = *std::max_element(v.begin(), v.end());
    double inmin = *std::min_element(v.begin(), v.end());

    Vector ret = v;
    for (double& d : ret) {
        d = (d-inmin)/(inmax-inmin)*(outmax-outmin)+outmin;
    }

    return ret;

}

Vector rangeNormalization(const Vector& v, const Vector& inmin, const Vector& inmax, double outmin, double outmax) {

    if ((v.size() != inmin.size()) || (v.size() != inmax.size()))
        throw std::invalid_argument("Vector sizes mismatch.");

    Vector ret = v;
    for (unsigned int i = 0; i < v.size(); i++) {
        ret[i] = (ret[i]-inmin[i])/(inmax[i]-inmin[i])*(outmax-outmin)+outmin;
    }

    return ret;

}

Vector rangeNormalization(const Vector& v, double inmin, double inmax, const Vector& outmin, const Vector& outmax) {

    if ((v.size() != outmin.size()) || (v.size() != outmax.size()))
        throw std::invalid_argument("Vector sizes mismatch.");

    Vector ret = v;
    for (unsigned int i = 0; i < v.size(); i++) {
        ret[i] = (ret[i]-inmin)/(inmax-inmin)*(outmax[i]-outmin[i])+outmin[i];
    }

    return ret;

}


}
