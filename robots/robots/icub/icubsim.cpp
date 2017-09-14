#include "icubsim.h"

#include <yarp/os/Network.h>

#include "commons.h"


namespace iCubSim {

const CameraParameters cameraParameters = CameraParameters{320, 240, 257.34, 257.34, 160, 120};

const Utils::Vector headPosMin = {-40, -70, -55, -35, -50, 0};
const Utils::Vector headPosMax = {30, 60, 55, 15, 52, 90};
const Utils::Vector headVelMin = {-400, -400, -400, -400, -400, -400};
const Utils::Vector headVelMax = {+400, +400, +400, +400, +400, +400};
const Utils::Vector torsoPosMin = {-50, -30, -10};
const Utils::Vector torsoPosMax = {50, 30, 70};
const Utils::Vector torsoVelMin = {-400, -400, -400};
const Utils::Vector torsoVelMax = {+400, +400, +400};

}

void _iCubSimHead::activate(const std::string& robotname, const std::string& localname) {

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

    yarp::dev::IPositionControl* p;
    driver.view(p);
    double d[] = {400.0, 400.0, 400.0, 400.0, 400.0, 400.0};
    p->setRefSpeeds(d);
    p->setRefAccelerations(d);

}

std::string _iCubSimHead::name() {
    return "iCubSimHead";
}

Utils::Vector _iCubSimHead::getMinPos() const {
    return iCubSim::headPosMin;
}

Utils::Vector _iCubSimHead::getMaxPos() const {
    return iCubSim::headPosMax;
}

Utils::Vector _iCubSimHead::getMinVel() const {
    return iCubSim::headVelMin;
}

Utils::Vector _iCubSimHead::getMaxVel() const {
    return iCubSim::headVelMax;
}


void _iCubSimTorso::activate(const std::string& robotname, const std::string& localname) {

    _iCubTorso::activate(robotname, localname);

    yarp::os::Network yarp;

    // motor drivers
    yarp::os::Property options;
    options.put("device", "remote_controlboard");
    options.put("local", "/"+localname+"/" + name());
    options.put("remote", "/"+robotname+"/torso");
    if (!driver.open(options)) {
        throw iCubException("iCubRobot: unable to create " + name() + " device.");
    }

    yarp::dev::IPositionControl* p;
    driver.view(p);
    double d[] = {400.0, 400.0, 400.0, 400.0, 400.0, 400.0};
    p->setRefSpeeds(d);
    p->setRefAccelerations(d);

}

std::string _iCubSimTorso::name() {
    return "iCubSimTorso";
}

Utils::Vector _iCubSimTorso::getMinPos() const {
    return iCubSim::torsoPosMin;
}

Utils::Vector _iCubSimTorso::getMaxPos() const {
    return iCubSim::torsoPosMax;
}

Utils::Vector _iCubSimTorso::getMinVel() const {
    return iCubSim::torsoVelMin;
}

Utils::Vector _iCubSimTorso::getMaxVel() const {
    return iCubSim::torsoVelMax;
}


std::string _iCubSimInertial::name() {
    return "iCubSimInertial";
}


iCubSimHead::iCubSimHead() {
    head = new _iCubSimHead();
}

iCubSimTorso::iCubSimTorso() {
    torso = new _iCubSimTorso();
}

iCubSimInertial::iCubSimInertial() {
    inertial = new _iCubSimInertial();
}
