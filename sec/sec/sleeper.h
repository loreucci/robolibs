#ifndef SLEEPER_H
#define SLEEPER_H

namespace sec {

class Sleeper {

public:
    virtual void sleep(double d) = 0;

    virtual bool isSynchronous() const = 0;

};

class BasicSleeper : public Sleeper {

public:

    virtual void sleep(double d) override;

    virtual bool isSynchronous() const override;

};

class NoSleeper : public Sleeper {

public:

    virtual void sleep(double) override;

    virtual bool isSynchronous() const override;

};

class Barrier : public Sleeper {

public:

    virtual void sleep(double) override;

    virtual bool isSynchronous() const override;

};

}

#endif // SLEEPER_H
