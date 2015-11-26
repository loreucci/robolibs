#ifndef COMMONSOURCES_H
#define COMMONSOURCES_H

#include <fstream>

#include <utilities/utilities.h>

#include "source.h"

template <typename T>
class ConstantSource : public Source<T> {

public:
    ConstantSource(T val, double samplingfreq = 0.0)
        :Source<T>(samplingfreq), val(val){

    }

    virtual std::string parameters() const override {
        return "Constant source = " + Utils::make_string(val, " ");
    }

protected:
    T val;

    virtual T getInput() override {
        return val;
    }

};


class SinusoidalSource : public Source<double> {

public:
    SinusoidalSource(double ampl, double freq, double phase = 0.0, double samplingfreq = 0.0);

    virtual std::string parameters() const override;

protected:
    double ampl, freq, phase;
    unsigned int step;

    double getInput() override;

};


class TriangularSource : public Source<double> {

public:
    TriangularSource(double ampl, double freq, double phase = 0.0, double samplingfreq = 0.0);

    virtual std::string parameters() const override;

protected:
    double ampl, freq, phase;
    unsigned int step;

    double getInput() override;

};


class FileSource : public Source<double> {

public:
    FileSource(const std::string& filename, unsigned int col, unsigned int tot_col, bool loop, unsigned int skip = 0, double samplingfreq = 0.0);
    virtual ~FileSource();

    virtual bool ended() const override;

    virtual std::string parameters() const override;

protected:
    std::ifstream file;
    const std::string filename;
    unsigned int col, tot_col;
    bool loop, end, beginning;
    unsigned int skip;

    double getInput() override;

};


//////////////////////////
/// \brief The HighDimSource class
///
template <typename T, unsigned int S, unsigned int ID>
class HighDimSource : public Source<Message<T, S, ID>> {

public:

    friend class HighDimSource<T, S+1, ID>;

    template <typename ...Types>
    HighDimSource(Source<T>& s, Types&...args)
        :s(s), sub(HighDimSource<T, S-1, ID>(args...)){

    }

    virtual void setSamplingFreq(double samplingfreq) override {
        this->samplingfreq = samplingfreq;
        s.setSamplingFreq(samplingfreq);
        sub.setSamplingFreq(samplingfreq);
    }

    virtual bool ended() const override {
        return s.ended() || sub.ended();
    }

    virtual std::string parameters() const {
        std::string ret = "HighDimSource of dimension " + std::to_string(S) + "\n\t";
        return ret + Utils::replace(this->get_substring(), "\n", "\n\t");
    }

protected:
    Source<T>& s;
    HighDimSource<T, S-1, ID> sub;

    std::string get_substring() const {
        return s.parameters() + "\n" + sub.get_substring();
    }

    T at(unsigned int i) {
        if (i >= S) {
            throw std::invalid_argument("Invalid index.");
        }
        if (i == S-1) {
            return s();
        }
        return sub.at(i);
    }

    virtual Message<T, S, ID> getInput() {
        Message<double, S, ID> ret;
        for (unsigned int i = 0; i < S; i++) {
            ret[i] = at(S-i-1);
        }
        return ret;
    }

};

///////////////////////
/// specialization to end template recursion
///
template <typename T, unsigned int ID>
class HighDimSource<T, 1, ID> : public Source<Message<T, 1, ID>> {

public:

    friend class HighDimSource<T, 2, ID>;

    HighDimSource(Source<T>& s)
        :s(s) {
    }

    virtual void setSamplingFreq(double samplingfreq) override {
        this->samplingfreq = samplingfreq;
        s.setSamplingFreq(samplingfreq);
    }

    virtual bool ended() const override {
        return s.ended();
    }

    virtual std::string parameters() const override {
        return "HighDimSource of dimension 1:\n\t" + get_substring();
    }

    Message<T, 1, ID> out() {
        return {s()};
    }

protected:
    Source<T>& s;

    std::string get_substring() const {
        return s.parameters();
    }

    T at(unsigned int i) {
        if (i != 0) {
            throw std::invalid_argument("Invalid index.");
        }
        return s();
    }

    virtual Message<T, 1, ID> getInput() {
        return {s()};
    }

};

#endif // COMMONSOURCES_H
