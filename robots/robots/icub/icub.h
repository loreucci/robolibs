#ifndef ICUB_H
#define ICUB_H

#include <utilities/vector.h>

#include "icubpart.h"

namespace iCub {

extern const Utils::Vector headPosMin;
extern const Utils::Vector headPosMax;
extern const Utils::Vector headVelMin;
extern const Utils::Vector headVelMax;
extern const Utils::Vector headInitial;

extern const Utils::Vector rightarmPosMin;
extern const Utils::Vector rightarmPosMax;
extern const Utils::Vector rightarmVelMin;
extern const Utils::Vector rightarmVelMax;
extern const Utils::Vector rightarmInitial;

extern const Utils::Vector leftarmPosMin;
extern const Utils::Vector leftarmPosMax;
extern const Utils::Vector leftarmVelMin;
extern const Utils::Vector leftarmVelMax;
extern const Utils::Vector leftarmInitial;

}

class _iCubHead : public _Head {

public:

    _iCubHead();
    virtual void activate(const std::string& robotname, const std::string& localname) override;

    virtual std::string name() const override;

    virtual Utils::Vector getMinPos() const override;
    virtual Utils::Vector getMaxPos() const override;
    virtual Utils::Vector getMinVel() const override;
    virtual Utils::Vector getMaxVel() const override;

};

class _iCubRightArm : public _RightArm {

public:
    _iCubRightArm();
    virtual void activate(const std::string& robotname, const std::string& localname) override;

    virtual std::string name() const override;

    virtual Utils::Vector getMinPos() const override;
    virtual Utils::Vector getMaxPos() const override;
    virtual Utils::Vector getMinVel() const override;
    virtual Utils::Vector getMaxVel() const override;

    // methods overrided to accept commands with only 4 references
    virtual void movePos(const Utils::Vector& refs, bool wait = false) override;
    virtual void moveVel(const Utils::Vector& refs, bool wait = false) override;

};

//class _iCubRightArm : public _LeftArm {

//public:
//    virtual unsigned int dof() const override;

//    virtual void refresh() override;

//    virtual void movePos(const Utils::Vector& refs, bool wait = false);
//    virtual void moveVel(const Utils::Vector& refs, bool wait = false);

//private:
//    static Utils::Vector handposition;

//};


class _iCubLeftArm : public _LeftArm {

public:
    _iCubLeftArm();
    virtual void activate(const std::string& robotname, const std::string& localname) override;

    virtual std::string name() const override;

    virtual Utils::Vector getMinPos() const override;
    virtual Utils::Vector getMaxPos() const override;
    virtual Utils::Vector getMinVel() const override;
    virtual Utils::Vector getMaxVel() const override;

};


///////////////
// part traits
class iCubHead : public HasHead {

public:
    iCubHead();

};

class iCubLeftArm : public HasLeftArm {

public:
    iCubLeftArm();

};

#endif // ICUB_H
