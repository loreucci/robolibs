#ifndef ICUBSIM_H
#define ICUBSIM_H

#include <utilities/vector.h>

#include "../camera.h"
#include "icubpart.h"


namespace iCubSim {

extern const CameraParameters cameraParameters;

extern const Utils::Vector headPosMin;
extern const Utils::Vector headPosMax;
extern const Utils::Vector headVelMin;
extern const Utils::Vector headVelMax;
extern const Utils::Vector headInitial;

extern const Utils::Vector torsoPosMin;
extern const Utils::Vector torsoPosMax;
extern const Utils::Vector torsoVelMin;
extern const Utils::Vector torsoVelMax;
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


class _iCubSimHead : public _Head {

public:
    _iCubSimHead();

    virtual void activate(const std::string& robotname, const std::string& localname) override;

    virtual std::string name() override;

    virtual Utils::Vector getMinPos() const override;
    virtual Utils::Vector getMaxPos() const override;
    virtual Utils::Vector getMinVel() const override;
    virtual Utils::Vector getMaxVel() const override;

};


class _iCubSimTorso : public _Torso {

public:
    _iCubSimTorso();

    virtual void activate(const std::string& robotname, const std::string& localname) override;

    virtual std::string name() override;

    virtual Utils::Vector getMinPos() const override;
    virtual Utils::Vector getMaxPos() const override;
    virtual Utils::Vector getMinVel() const override;
    virtual Utils::Vector getMaxVel() const override;

};


class _iCubSimRightArm : public _RightArm {

public:
    _iCubSimRightArm();
    virtual void activate(const std::string& robotname, const std::string& localname) override;

    virtual std::string name() override;

    virtual Utils::Vector getMinPos() const override;
    virtual Utils::Vector getMaxPos() const override;
    virtual Utils::Vector getMinVel() const override;
    virtual Utils::Vector getMaxVel() const override;

};

class _iCubSimLeftArm : public _LeftArm {

public:
    _iCubSimLeftArm();
    virtual void activate(const std::string& robotname, const std::string& localname) override;

    virtual std::string name() override;

    virtual Utils::Vector getMinPos() const override;
    virtual Utils::Vector getMaxPos() const override;
    virtual Utils::Vector getMinVel() const override;
    virtual Utils::Vector getMaxVel() const override;

};

class _iCubSimInertial : public _Inertial {

public:
    virtual std::string name() override;

};

///////////////
// part traits

class iCubSimHead : public HasHead {

public:
    iCubSimHead();

};

class iCubSimTorso : public HasTorso {

public:
    iCubSimTorso();

};

class iCubSimRightArm : public HasRightArm {

public:
    iCubSimRightArm();

};

class iCubSimLeftArm : public HasLeftArm {

public:
    iCubSimLeftArm();

};

class iCubSimInertial : public HasInertial {

public:
    iCubSimInertial();
};

#endif // ICUBSIM_H
