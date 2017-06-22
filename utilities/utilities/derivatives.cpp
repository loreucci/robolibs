#include "derivatives.h"

#include <stdexcept>
#include <cmath>

using namespace Utils;

Derivative::Derivative(double freq)
    :freq(freq) {}


Utils::Vector SimpleDerivative::derive(const Utils::Vector& x) {

    unsigned int sz = x.size();

    if (prev.size() != sz)
        prev.resize(sz);

    auto der = (x-prev)*freq;

    prev = x;
    return der;

}


int binomialcoeff(int n, int k) {

    if (k < 0)
        return 0;

    if (n < k)
        throw std::invalid_argument("binomialcoeff: n must be >= k.");

    double ret = 1;

    for (int i = 1; i < k + 1; i++) {
        ret *= (double)(n+1-i)/i;
    }

    return ret;

}


SmoothDerivative::SmoothDerivative(unsigned int N, double freq)
    :Derivative(freq) {

    if (N % 2 != 1 || N < 5)
        throw std::invalid_argument("SmoothDerivative: N must be odd and >= 5.");

    M = (N-1) / 2;
    m = (N-3) / 2;

    // compute coeffs
    coeff = 1.0/std::pow(2, 2*m+1);
    c.resize(M);
    for (unsigned int k = 1; k < c.size()+1; k++) {
        c[k-1] = binomialcoeff(2*m, m-k+1) - binomialcoeff(2*m, m-k-1);
    }

    // memory
    past.resize(M);
    fut.resize(M);

    datasize = 0;

}

Utils::Vector SmoothDerivative::derive(const Utils::Vector& x) {

    // initialize buffers to the proper data size
    if (x.size() != datasize) {
        datasize = x.size();
        for (auto& v : past)
            v.resize(datasize);
        pres.resize(datasize);
        for (auto& v : fut)
            v.resize(datasize);
    }

    // advance time
    fut.push_back(x);
    past.push_front(pres);
    pres = fut.front();
    fut.pop_front();
    past.pop_back();

    // derivative
    std::vector<double> ret(datasize);
    for (unsigned int i = 0; i < datasize; i++) {
        for (unsigned int k = 0; k < M; k++) {
            ret[i] += c[k]*(fut[k][i]-past[k][i]);
        }
        ret[i] *= freq*coeff;
    }

    return ret;

}
