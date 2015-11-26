#include "commonsources.h"

#include <cmath>
#include <limits>

#include <utilities/utilities.h>

///////////////////
/// \brief SinusoidalSource::SinusoidalSource
/// \param ampl
/// \param freq
/// \param phase
/// \param samplingfreq
///
SinusoidalSource::SinusoidalSource(double ampl, double freq, double phase, double samplingfreq)
    :Source<double>(samplingfreq), ampl(ampl), freq(freq), phase(phase) {
    step = 0;
}

std::string SinusoidalSource::parameters() const {

    std::string ret = "Sinusoidal source:\n";
    ret += "\tampl = " + std::to_string(ampl) + "\n";
    ret += "\tfreq = " + std::to_string(freq) + "\n";
    ret += "\tphase = " + std::to_string(phase);
    return ret;

}

double SinusoidalSource::getInput() {
    double ret = ampl * std::sin(2*Utils::PI*freq*step/samplingfreq + phase);
    step++;
    return ret;
}




////////////////////////////
/// \brief FileSource::FileSource
/// \param filename
/// \param col
/// \param tot_col
/// \param skip
/// \param loop
///
FileSource::FileSource(const std::string& filename, unsigned int col, unsigned int tot_col, bool loop, unsigned int skip, double samplingfreq)
    :Source<double>(samplingfreq), filename(filename), col(col), tot_col(tot_col), loop(loop), skip(skip) {

    file.open(filename);
    if (!file.is_open()) {
        throw std::runtime_error("FileSource: unable to open file " + filename + ".");
    }
    file.peek();
    if (!file.good()) {
        throw std::runtime_error("FileSource: file " + filename + " appears to be empty or unreadable.");
    }

    if (col == 0 || tot_col == 0 || col > tot_col) {
        throw std::runtime_error("FileSource: wrong number of columns.");
    }

    end = false;
    beginning = true;

}

FileSource::~FileSource() {
    file.close();
}

bool FileSource::ended() const {
    return end;
}

std::string FileSource::parameters() const {

    return "File source from " + filename + ", using column " + std::to_string(col) + " of " + std::to_string(tot_col);

}

double FileSource::getInput() {

    if (end)
        return this->getCurrent();

    // skip initial lines
    if (beginning) {
        for (unsigned int k = 0; k < skip; k++) {
            std::string s;
            std::getline(file, s);
        }
        beginning = false;
    }

    double r, ret;

    // find value we are interested in
    unsigned int i = 0;
    for (; i < col; i++) {
        file >> ret;
        if (!file.good())
            throw std::runtime_error("FileSource::getInput: something went wrong reading the file");
    }

    // skip rest of the columns
    for (; i < tot_col; i++) {
        file >> r;
        if (!file.good())
            throw std::runtime_error("FileSource::getInput: something went wrong reading the file");
    }

    // skip whitespace
    char c = ' ';
    while (std::isspace(c)) {
        file >> c;
        if (file.eof())
            break;
    }

    // check whether the file is finished or not
    if (!file.eof()) {
        file.putback(c);
    } else {
        if (loop) {
            file.clear();
            file.seekg(0);
            beginning = true;
        } else {
            end = true;
        }
    }

    return ret;

}


///////////////////////
/// \brief TriangularSource::SinusoidalSource
/// \param ampl
/// \param freq
/// \param phase
/// \param samplingfreq
///
TriangularSource::TriangularSource(double ampl, double freq, double phase, double samplingfreq)
    :Source<double>(samplingfreq), ampl(ampl), freq(freq), phase(phase) {

    step = 0;

}

std::string TriangularSource::parameters() const {

    std::string ret = "Triangular source:\n";
    ret += "\tampl = " + std::to_string(ampl) + "\n";
    ret += "\tfreq = " + std::to_string(freq) + "\n";
    ret += "\tphase = " + std::to_string(phase);
    return ret;

}

double TriangularSource::getInput() {
    double ret = ampl*(2*std::abs(2*(step/samplingfreq*freq + phase - std::floor(step/samplingfreq*freq+phase+0.5)))-1);
    step++;
    return ret;
}
