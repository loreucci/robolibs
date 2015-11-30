#ifndef SLEEPER_H
#define SLEEPER_H

namespace sec {

class Sleeper {

public:
    virtual void sleep(double d) = 0;

};

class BasicSleeper : public Sleeper {

public:

    virtual void sleep(double d) override;

};

class NoSleeper : public Sleeper {

public:

    virtual void sleep(double) override;

};

}

#endif // SLEEPER_H
