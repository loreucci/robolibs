#include "icub.h"

#include "commons.h"

namespace iCub {

const Utils::Vector headPosMin =  { -40,  -70,  -55,  -35,  -50,    0};
const Utils::Vector headPosMax =  {  30,   60,   55,   15,   52,   90};
const Utils::Vector headVelMin =  {-100, -100, -100, -100, -100, -100};
const Utils::Vector headVelMax =  {+100, +100, +100, +100, +100, +100};
const Utils::Vector headInitial = {   0,    0,    0,    0,    0,    0};

const Utils::Vector rightarmPosMin =  { -95,     0,  -37, 15.5,  -90,  -90,  -20,    0,    0,    0,    0,    0,    0,    0,    0,    0};
const Utils::Vector rightarmPosMax =  {  10, 160.8,   80,  106,   90,    0,   40,   60,   90,   90,  180,   90,  180,   90,  180,  270};
const Utils::Vector rightarmVelMin =  {-100,  -100, -100, -100, -100, -100, -100, -100, -100, -100, -100, -100, -100, -100, -100, -100};
const Utils::Vector rightarmVelMax =  {+100,  +100, +100, +100, +100, +100, +100, +100, +100, +100, +100, +100, +100, +100, +100, +100};
const Utils::Vector rightarmInitial = { -25,    20,    0,   50,    0,    0,    0,   60,   20,   20,   20,   10,   10,   10,   10,   10};

const Utils::Vector leftarmPosMin =  { -94.5,    0,  -36,   19,  -90,  -90,  -20,    0,    0,    0,    0,    0,    0,    0,    0,    0};
const Utils::Vector leftarmPosMax =  {   9.5,  161,   80,  106,   90,    0,   40,   60,  180,   90,  180,   90,  180,   90,  180,  270};
const Utils::Vector leftarmVelMin =  {  -100, -100, -100, -100, -100, -100, -100, -100, -100, -100, -100, -100, -100, -100, -100, -100};
const Utils::Vector leftarmVelMax =  {  +100, +100, +100, +100, +100, +100, +100, +100, +100, +100, +100, +100, +100, +100, +100, +100};
const Utils::Vector leftarmInitial = {   -25,   20,    0,   50,    0,    0,    0,   30,   20,   20,   20,   10,   10,   10,   10,    5};

}


_iCubHead::_iCubHead() {
    initpos = iCub::headInitial;
}

void _iCubHead::activate(const std::string& robotname, const std::string& localname) {

    _Head::activate(robotname, localname);

}

std::string _iCubHead::name() const {
    return "iCubHead";
}

Utils::Vector _iCubHead::getMinPos() const {
    return iCub::headPosMin;
}

Utils::Vector _iCubHead::getMaxPos() const {
    return iCub::headPosMax;
}

Utils::Vector _iCubHead::getMinVel() const {
    return iCub::headVelMin;
}

Utils::Vector _iCubHead::getMaxVel() const {
    return iCub::headVelMax;
}


_iCubRightArm::_iCubRightArm() {
    initpos = iCub::rightarmInitial;
}

void _iCubRightArm::activate(const std::string& robotname, const std::string& localname) {

    _RightArm::activate(robotname, localname);

}

std::string _iCubRightArm::name() const {
    return "iCubRightArm";
}

Utils::Vector _iCubRightArm::getMinPos() const {
    return iCub::rightarmPosMin;
}

Utils::Vector _iCubRightArm::getMaxPos() const {
    return iCub::rightarmPosMax;
}

Utils::Vector _iCubRightArm::getMinVel() const {
    return iCub::rightarmVelMin;
}

Utils::Vector _iCubRightArm::getMaxVel() const {
    return iCub::rightarmVelMax;
}

void _iCubRightArm::movePos(const Utils::Vector& refs, bool wait) {

    if (refs.size() == 4) {
        _RightArm::movePos(Utils::joinVectors(refs, Utils::subvector(iCub::rightarmInitial, 4, 15)), wait);
    } else {
        _RightArm::movePos(refs, wait);
    }

}

void _iCubRightArm::moveVel(const Utils::Vector& refs, bool wait) {

    if (refs.size() == 4) {
        _RightArm::moveVel(Utils::joinVectors(refs, Utils::subvector(iCub::rightarmInitial, 4, 15)), wait);
    } else {
        _RightArm::moveVel(refs, wait);
    }

}


_iCubLeftArm::_iCubLeftArm() {
    initpos = iCub::leftarmInitial;
}

void _iCubLeftArm::activate(const std::string& robotname, const std::string& localname) {

    _LeftArm::activate(robotname, localname);

    // is it needed?
//    obs = new VelocityObserver("joint/Vel", "encspeeds", 2.0, 10);

    setRefSpeeds(0.25);

}

std::string _iCubLeftArm::name() const {
    return "iCubLeftArm";
}

Utils::Vector _iCubLeftArm::getMinPos() const {
    return iCub::leftarmPosMin;
}

Utils::Vector _iCubLeftArm::getMaxPos() const {
    return iCub::leftarmPosMax;
}

Utils::Vector _iCubLeftArm::getMinVel() const {
    return iCub::leftarmVelMin;
}

Utils::Vector _iCubLeftArm::getMaxVel() const {
    return iCub::leftarmVelMax;
}


iCubHead::iCubHead() {
    head = new _iCubHead();
}

iCubLeftArm::iCubLeftArm() {
    leftarm = new _iCubLeftArm();
}
