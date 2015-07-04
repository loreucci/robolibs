#ifndef SOURCE_H
#define SOURCE_H

#include <stdexcept>
#include <functional>

#include <utilities/utilities.h>

// basic source
template <typename T>
class Source {

public:
    Source(double samplingfreq = 0.0)
        :samplingfreq(samplingfreq) {
    }

    virtual T next() final {
        return nxt = getInput();
    }

    virtual T getCurrent() const final {
        return nxt;
    }

    virtual T operator()() final {
        return next();
    }

    double getSamplingFreq() const {
        return samplingfreq;
    }

    virtual void setSamplingFreq(double samplingfreq) {
        this->samplingfreq = samplingfreq;
    }

    virtual bool ended() const {
        return false;
    }

    virtual std::string parameters() const {
        return "Constant source returning T().";
    }

protected:
    double samplingfreq;
    T nxt;

    virtual T getInput() {
        return T();
    }

};


// TODO:
// vedere se si può copiare le source dentro Sum e Mul, così da poter fare
// auto s4 = s1+s2;
// auto s5 = s1+s3;


// sum sources
template <typename T>
class SumSource : public Source<T> {

public:
    SumSource(Source<T>& s1, Source<T>& s2, std::function<T(const T&, const T&)> plus = std::plus<T>())
        :s1(s1), s2(s2), plus(plus) {

        if (s1.getSamplingFreq() == 0) {
            s1.setSamplingFreq(s2.getSamplingFreq());
            this->samplingfreq = s2.getSamplingFreq();
        } else if (s2.getSamplingFreq() == 0) {
            s2.setSamplingFreq(s1.getSamplingFreq());
            this->samplingfreq = s1.getSamplingFreq();
        } else if (s1.getSamplingFreq() != s2.getSamplingFreq()) {
            throw std::runtime_error("Summing sources with different sampling times.");
        } else {
            this->samplingfreq = s1.getSamplingFreq();
        }

    }

    virtual void setSamplingFreq(double samplingfreq) override {
        this->samplingfreq = samplingfreq;
        s1.setSamplingFreq(samplingfreq);
        s2.setSamplingFreq(samplingfreq);
    }

    virtual std::string parameters() const override{
        std::string ret = "Sum of sources: \n";
        auto par1 = Utils::replace(s1.parameters(), "\n", "\n\t");
        auto par2 = Utils::replace(s2.parameters(), "\n", "\n\t");
        ret += "\t" + par1 + "\n\t" + par2;
        return ret;
    }

protected:
    Source<T>& s1;
    Source<T>& s2;
    std::function<T(const T&, const T&)> plus;

    virtual T getInput() override {
        return plus(s1.next(), s2.next());
    }

};

// helpers
template <typename T>
SumSource<T> operator+(Source<T>& s1, Source<T>& s2) {
    return SumSource<T>(s1, s2);
}

template <typename T>
SumSource<T> operator+(Source<T>&& s1, Source<T>&& s2) {
    return SumSource<T>(s1, s2);
}

template <typename T>
SumSource<T> operator+(Source<T>& s1, Source<T>&& s2) {
    return SumSource<T>(s1, s2);
}

template <typename T>
SumSource<T> operator+(Source<T>&& s1, Source<T>& s2) {
    return SumSource<T>(s1, s2);
}



// multiply sources
template <typename A, typename B>
B mult(const A& a, const B& b) {
    return a*b;
}


template <typename A, typename B>
class MulSource : public Source<B> {

public:
    MulSource(Source<A>& s1, Source<B>& s2, std::function<B(const A&, const B&)> mul = mult<A, B>)
        :s1(s1), s2(s2), mul(mul) {

        if (s1.getSamplingFreq() == 0) {
            s1.setSamplingFreq(s2.getSamplingFreq());
            this->samplingfreq = s2.getSamplingFreq();
        } else if (s2.getSamplingFreq() == 0) {
            s2.setSamplingFreq(s1.getSamplingFreq());
            this->samplingfreq = s1.getSamplingFreq();
        } else if (s1.getSamplingFreq() != s2.getSamplingFreq()) {
            throw std::runtime_error("Multipling sources with different sampling times.");
        } else {
            this->samplingfreq = s1.getSamplingFreq();
        }

    }

    virtual void setSamplingFreq(double samplingfreq) override {
        this->samplingfreq = samplingfreq;
        s1.setSamplingFreq(samplingfreq);
        s2.setSamplingFreq(samplingfreq);
    }

    virtual std::string parameters() const override{
        std::string ret = "Multiplication of sources: \n";
        auto par1 = Utils::replace(s1.parameters(), "\n", "\n\t");
        auto par2 = Utils::replace(s2.parameters(), "\n", "\n\t");
        ret += "\t" + par1 + "\n\t" + par2;
        return ret;
    }

protected:
    Source<A>& s1;
    Source<B>& s2;
    std::function<B(const A&, const B&)> mul;

    virtual B getInput() override {
        return mul(s1.next(), s2.next());
    }

};

// helpers
template <typename A, typename B>
MulSource<A, B> operator*(Source<A>& s1, Source<B>& s2) {
    return MulSource<A, B>(s1, s2);
}

template <typename A, typename B>
MulSource<A, B> operator*(Source<A>&& s1, Source<B>& s2) {
    return MulSource<A, B>(s1, s2);
}

template <typename A, typename B>
MulSource<A, B> operator*(Source<A>& s1, Source<B>&& s2) {
    return MulSource<A, B>(s1, s2);
}

template <typename A, typename B>
MulSource<A, B> operator*(Source<A>&& s1, Source<B>&& s2) {
    return MulSource<A, B>(s1, s2);
}




////////////////////////
/// \brief The SwitchSource class
///
template <typename T>
class SwitchSource : public Source<T> {

public:
    SwitchSource(Source<T>& s1, Source<T>& s2, double secs)
        :s1(s1), s2(s2), secs(secs) {

        if (s1.getSamplingFreq() == 0 && s2.getSamplingFreq() != 0) {
            s1.setSamplingFreq(s2.getSamplingFreq());
            this->samplingfreq = s2.getSamplingFreq();
        } else if (s2.getSamplingFreq() == 0 && s1.getSamplingFreq() != 0) {
            s2.setSamplingFreq(s1.getSamplingFreq());
            this->samplingfreq = s1.getSamplingFreq();
        } else if (s1.getSamplingFreq() != s2.getSamplingFreq()) {
            throw std::runtime_error("Switching between sources with different sampling times.");
        } else {
            this->samplingfreq = s1.getSamplingFreq();
        }

        steps = 0;

    }

    virtual void setSamplingFreq(double samplingfreq) override {
        this->samplingfreq = samplingfreq;
        s1.setSamplingFreq(samplingfreq);
        s2.setSamplingFreq(samplingfreq);
    }

    virtual bool ended() const override {
        return s1.ended() || s2.ended();
    }

    virtual std::string parameters() const override {
        std::string ret = "Switching sources after " + std::to_string(secs) + " seconds:\n";
        auto par1 = Utils::replace(s1.parameters(), "\n", "\n\t");
        auto par2 = Utils::replace(s2.parameters(), "\n", "\n\t");
        ret += "\t" + par1 + "\n\t" + par2;
        return ret;
    }

protected:
    Source<T>& s1;
    Source<T>& s2;
    double secs;
    unsigned int steps;

    virtual T getInput() override {

        if (this->samplingfreq == 0)
            throw std::runtime_error("Sampling frequency == 0.");

        s1();
        s2();

        double time = steps/this->samplingfreq;
        steps++;

        if (time < secs) {
            return s1.getCurrent();
        }
        return s2.getCurrent();

    }

};

#endif // SOURCE_H
