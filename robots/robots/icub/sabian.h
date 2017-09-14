#ifndef SABIAN_H
#define SABIAN_H

#include <utilities/vector.h>

#include "../camera.h"
#include "icubpart.h"
#include "commons.h"


namespace Sabian {

extern const CameraParameters cameraParameters320;
extern const CameraParameters cameraParameters640;

extern const Utils::Vector headPosMin;
extern const Utils::Vector headPosMax;
extern const Utils::Vector headVelMin;
extern const Utils::Vector headVelMax;

extern const Utils::Vector headMask;

}


class _SabianHead : public _iCubHead {

public:
    virtual void activate(const std::string& robotname, const std::string& localname) override;
    virtual void deactivate() override;

    virtual void refresh() override;

    virtual void movePos(const Utils::Vector& refs, bool wait = false) override;
    virtual void moveVel(const Utils::Vector& refs, bool wait = false) override;
    virtual void movePosJoint(unsigned int joint, double ref, bool wait = false) override;
    virtual void moveVelJoint(unsigned int joint, double ref, bool wait = false) override;

    virtual std::string name() override;

    virtual Utils::Vector getMinPos() const override;
    virtual Utils::Vector getMaxPos() const override;
    virtual Utils::Vector getMinVel() const override;
    virtual Utils::Vector getMaxVel() const override;

protected:
    VelocityObserver* obs;

    Utils::Vector applyMask(const Utils::Vector& v);

};


class _SabianInertial : public _Inertial {

public:
    virtual std::string name() override;

};

///////////////
// part traits
class SabianHead : public HasHead {

public:
    SabianHead();

};

class SabianInertial : public HasInertial {

public:
    SabianInertial();
};

#endif // SABIAN_H
