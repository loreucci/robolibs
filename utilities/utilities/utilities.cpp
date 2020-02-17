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

#include "utilities.h"

#include <random>
#include <iostream>
#include <iterator>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <ctime>


namespace Utils {

// standard generation function
std::function<Vector(void)> uniformGenerator(unsigned int size, double a, double b) {
    std::mt19937 engine = std::mt19937(static_cast<unsigned long>(time(nullptr)));;
    std::uniform_real_distribution<double> dist(a, b);

    return [engine, dist, size] () mutable {
        Vector ret(size);
        for (unsigned int i = 0; i < size; i++) {
            ret[i] = dist(engine);
        }
        return ret;
    };

}


void printVector(const Vector v, const std::string& pre, bool endline) {

    std::cout << pre;
    std::copy(v.begin(), v.end(), std::ostream_iterator<double>(std::cout, " "));
    if (endline)
        std::cout << std::endl;

}


std::vector<Vector> readDatasetFromFile(const std::string& filename, char del, unsigned int skiplines) {

    std::ifstream file(filename, std::ios_base::in);

    std::vector<std::vector<double>> set;

    unsigned int count = 0;
    while (file.good() && !file.eof()) {
        std::string line;
        std::vector<double> p;
        std::getline(file, line);
        if ( file.eof() && line.empty()) {
            break;
        }
        count++;
        if (count <= skiplines)
            continue;

        std::stringstream lstream(line);
        std::string token;
        while (std::getline(lstream, token, del)) {
            std::stringstream ss(token);
            double d;
            ss >> d;
            p.push_back(d);
        }

        set.push_back(p);

        if (file.eof()) {
            break;
        }

    }

    file.close();

    return set;

}

void saveToFile(const std::string& filename, const std::vector<Vector>& data) {

    std::ofstream out(filename);

    for (auto& v : data) {
        std::copy(v.begin(), v.end(), std::ostream_iterator<double>(out, " "));
        out << std::endl;
    }

    out.close();

}

template <>
std::string make_string(const std::string& t, const std::string& sep) {
    UTILS_UNUSED(sep);
    return t;
}

std::string replace(const std::string& str, const std::string& target, const std::string& repl) {

    if (target.length() == 0)
        return str;

    if (str.length() == 0)
        return str;

    std::string ret = str;
    size_t idx = 0;

    for (;;) {
        idx = ret.find(target, idx);
        if (idx == std::string::npos)
            break;

        ret.replace(idx, target.length(), repl);
        idx += repl.length();
    }

    return ret;

}

double degtorad(double deg) {
    return (deg / 360.0) * (2 * PI);
}

double radtodeg(double rad) {
    return (rad / (2*PI)) * 360.0;
}

}
