#include "integrals.h"


using namespace Utils;


Integral::Integral(double freq)
    :freq(freq) {}


Vector IntegralRectangle::integrate(const Vector& x) {

    unsigned int sz = x.size();

    if (integr.size() != sz)
        integr.resize(sz);

    integr = integr + 1.0/freq*x;

    return integr;

}


IntegralTrapezoidal::IntegralTrapezoidal(double freq)
    :Integral(freq) {
    first = true;
}

Utils::Vector IntegralTrapezoidal::integrate(const Utils::Vector& x) {

    unsigned int sz = x.size();

    if (integr.size() != sz)
        integr.resize(sz);
    if (prev.size() != sz)
        prev.resize(sz);

    if (first)
        first = false;
    else
        integr = integr + 1.0/freq*(x+prev)/2.0;

    prev = x;

    return integr;

}
