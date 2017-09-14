#include "sabian.h"

namespace Sabian {

const CameraParameters cameraParameters320 = CameraParameters{320, 240, 258.245, 257.123, 160, 120};//122.547, 113.737};
const CameraParameters cameraParameters640 = CameraParameters{640, 480, 443.60, 444.75, 320, 240};

const Utils::Vector headPosMin = {-40, -70, -55, -35, -50, 0};
const Utils::Vector headPosMax = {30, 60, 55, 15, 52, 90};
const Utils::Vector headVelMin = {-400, -400, -400, -400, -400, -400};
const Utils::Vector headVelMax = {+400, +400, +400, +400, +400, +400};

const Utils::Vector headMask = {-1, -1, -1, -1, -1,  1};

}



void _SabianHead::activate(const std::string& robotname, const std::string& localname) {

    _iCubHead::activate(robotname, localname);

    yarp::os::Network yarp;

    // motor drivers
    yarp::os::Property options;
    options.put("device", "remote_controlboard");
    options.put("local", "/"+localname+"/" + name());
    options.put("remote", "/"+robotname+"/head");
    if (!driver.open(options)) {
        throw iCubException("iCubRobot: unable to create " + name() + " device.");
    }

    obs = new VelocityObserver("joint/Vel", "encspeeds", 2.0, 10);

    yarp::dev::IPositionControl* p;
    driver.view(p);
    double d[] = {400.0, 400.0, 400.0, 400.0, 400.0, 400.0};
    p->setRefSpeeds(d);
    p->setRefAccelerations(d);

}

void _SabianHead::deactivate() {

    delete obs;

    _iCubHead::deactivate();

}

void _SabianHead::refresh() {

    yarp::dev::IEncoders* encs = nullptr;
    driver.view(encs);
    if (encs == nullptr)
        throw iCubException("iCubRobot(" + name() + "): unable to use driver.");
    double p[dof()];
    encs->getEncoders(p);
    Utils::Vector _pos(dof());
    for (unsigned int i = 0; i < dof(); i++) {
        _pos[i] = p[i];
    }
    _pos = applyMask(_pos);

    auto _vel = obs->derive(_pos);

    mtx.lock();
    pos = _pos;
    vel = _vel;
    mtx.unlock();

}

void _SabianHead::movePos(const Utils::Vector& refs, bool wait) {
    _iCubHead::movePos(applyMask(refs), wait);
}

void _SabianHead::moveVel(const Utils::Vector& refs, bool wait) {
    _iCubHead::moveVel(applyMask(refs), wait);
}

void _SabianHead::movePosJoint(unsigned int joint, double ref, bool wait) {
    _iCubHead::movePosJoint(joint, ref*Sabian::headMask[joint], wait);
}

void _SabianHead::moveVelJoint(unsigned int joint, double ref, bool wait) {
    _iCubHead::moveVelJoint(joint, ref*Sabian::headMask[joint], wait);
}

std::string _SabianHead::name() {
    return "SabianHead";
}

Utils::Vector _SabianHead::getMinPos() const {
    return Sabian::headPosMin;
}

Utils::Vector _SabianHead::getMaxPos() const {
    return Sabian::headPosMax;
}

Utils::Vector _SabianHead::getMinVel() const {
    return Sabian::headVelMin;
}

Utils::Vector _SabianHead::getMaxVel() const {
    return Sabian::headVelMax;
}

Utils::Vector _SabianHead::applyMask(const Utils::Vector& v) {

    Utils::Vector ret(dof());
    for (unsigned int i = 0; i < dof(); i++) {
        ret[i] = v[i] * Sabian::headMask[i];
    }
    return ret;

}


std::string _SabianInertial::name() {
    return "SabianInertial";
}


SabianHead::SabianHead() {
    head = new _SabianHead();
}


SabianInertial::SabianInertial() {
    inertial = new _SabianInertial();
}
