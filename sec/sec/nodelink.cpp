#include "nodelink.h"

namespace sec {

std::vector<double> convert(const double& a) {
    return {a};
}

double convert(const std::vector<double>& a) {
    return a[0];
}

}
