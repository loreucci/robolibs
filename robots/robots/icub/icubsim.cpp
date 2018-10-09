#include "icubsim.h"

#include <yarp/os/Network.h>

#include "commons.h"


namespace iCubSim {

const CameraParameters cameraParameters = CameraParameters{320, 240, 257.34, 257.34, 160, 120};

const Utils::Vector headPosMin =  { -40,  -70,  -55,  -35,  -50,    0};
const Utils::Vector headPosMax =  {  30,   60,   55,   15,   52,   90};
const Utils::Vector headVelMin =  {-400, -400, -400, -400, -400, -400};
const Utils::Vector headVelMax =  {+400, +400, +400, +400, +400, +400};
const Utils::Vector headInitial = {   0,    0,    0,    0,    0,    0};

const Utils::Vector torsoPosMin =  { -50,  -30,  -10};
const Utils::Vector torsoPosMax =  {  50,   30,   70};
const Utils::Vector torsoVelMin =  {-400, -400, -400};
const Utils::Vector torsoVelMax =  {+400, +400, +400};
const Utils::Vector torsoInitial = {   0,    0,    0};

const Utils::Vector rightarmPosMin =  { -95,     0,  -37, 15.5,  -90,  -90,  -20,    0,    0,    0,    0,    0,    0,    0,    0,    0};
const Utils::Vector rightarmPosMax =  {  10, 160.8,   80,  106,   90,    0,   40,   60,   90,   90,  180,   90,  180,   90,  180,  270};
const Utils::Vector rightarmVelMin =  {-100,  -100, -100, -100, -100, -100, -100, -100, -100, -100, -100, -100, -100, -100, -100, -100};
const Utils::Vector rightarmVelMax =  {+100,  +100, +100, +100, +100, +100, +100, +100, +100, +100, +100, +100, +100, +100, +100, +100};
const Utils::Vector rightarmInitial = { -25,    20,    0,   50,    0,    0,    0,   60,   20,   20,   20,   10,   10,   10,   10,   10};

const Utils::Vector leftarmPosMin =  { -94.5,    0,  -36,   19,  -90,  -90,  -20,    0,    0,    0,    0,    0,    0,    0,    0,    0};
const Utils::Vector leftarmPosMax =  {   9.5,  161,   80,  106,   90,    0,   40,   60,   90,   90,  180,   90,  180,   90,  180,  270};
const Utils::Vector leftarmVelMin =  {  -100, -100, -100, -100, -100, -100, -100, -100, -100, -100, -100, -100, -100, -100, -100, -100};
const Utils::Vector leftarmVelMax =  {  +100, +100, +100, +100, +100, +100, +100, +100, +100, +100, +100, +100, +100, +100, +100, +100};
const Utils::Vector leftarmInitial = {   -25,   20,    0,   50,    0,    0,    0,   60,   20,   20,   20,   10,   10,   10,   10,   10};

}

_iCubSimHead::_iCubSimHead() {
    initpos = iCubSim::headInitial;
}

void _iCubSimHead::activate(const std::string& robotname, const std::string& localname) {

    _Head::activate(robotname, localname);

//    yarp::os::Network yarp;

    // motor drivers
    yarp::os::Property options;
    options.put("device", "remote_controlboard");
    options.put("local", "/"+localname+"/" + name());
    options.put("remote", "/"+robotname+"/head");
    if (!driver.open(options)) {
        throw iCubException("[iCubRobot] Unable to create " + name() + " device.");
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


_iCubSimTorso::_iCubSimTorso() {
    initpos = iCubSim::torsoInitial;
}

void _iCubSimTorso::activate(const std::string& robotname, const std::string& localname) {

    _Torso::activate(robotname, localname);

    yarp::os::Network yarp;

    // motor drivers
    yarp::os::Property options;
    options.put("device", "remote_controlboard");
    options.put("local", "/"+localname+"/" + name());
    options.put("remote", "/"+robotname+"/torso");
    if (!driver.open(options)) {
        throw iCubException("[iCubRobot] Unable to create " + name() + " device.");
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


_iCubSimRightArm::_iCubSimRightArm() {
    initpos = iCubSim::rightarmInitial;
}

void _iCubSimRightArm::activate(const std::string& robotname, const std::string& localname) {

    _RightArm::activate(robotname, localname);

    yarp::os::Network yarp;

    // motor drivers
    yarp::os::Property options;
    options.put("device", "remote_controlboard");
    options.put("local", "/"+localname+"/" + name());
    options.put("remote", "/"+robotname+"/right_arm");
    if (!driver.open(options)) {
        throw iCubException("[iCubRobot] Unable to create " + name() + " device.");
    }

    yarp::dev::IPositionControl* p;
    driver.view(p);
    double d[] = {400.0, 400.0, 400.0, 400.0, 400.0, 400.0, 400.0, 400.0, 400.0, 400.0, 400.0, 400.0, 400.0, 400.0, 400.0, 400.0};
    p->setRefSpeeds(d);
    p->setRefAccelerations(d);

}

std::string _iCubSimRightArm::name() {
    return "iCubSimRightArm";
}

Utils::Vector _iCubSimRightArm::getMinPos() const {
    return iCubSim::rightarmPosMin;
}

Utils::Vector _iCubSimRightArm::getMaxPos() const {
    return iCubSim::rightarmPosMax;
}

Utils::Vector _iCubSimRightArm::getMinVel() const {
    return iCubSim::rightarmVelMin;
}

Utils::Vector _iCubSimRightArm::getMaxVel() const {
    return iCubSim::rightarmVelMax;
}

_iCubSimLeftArm::_iCubSimLeftArm() {
    initpos = iCubSim::leftarmInitial;
}

void _iCubSimLeftArm::activate(const std::string& robotname, const std::string& localname) {

    _LeftArm::activate(robotname, localname);

    yarp::os::Network yarp;

    // motor drivers
    yarp::os::Property options;
    options.put("device", "remote_controlboard");
    options.put("local", "/"+localname+"/" + name());
    options.put("remote", "/"+robotname+"/left_arm");
    if (!driver.open(options)) {
        throw iCubException("[iCubRobot] Unable to create " + name() + " device.");
    }

    yarp::dev::IPositionControl* p;
    driver.view(p);
    double d[] = {400.0, 400.0, 400.0, 400.0, 400.0, 400.0, 400.0, 400.0, 400.0, 400.0, 400.0, 400.0, 400.0, 400.0, 400.0, 400.0};
    p->setRefSpeeds(d);
    p->setRefAccelerations(d);

}

std::string _iCubSimLeftArm::name() {
    return "iCubSimLeftArm";
}

Utils::Vector _iCubSimLeftArm::getMinPos() const {
    return iCubSim::leftarmPosMin;
}

Utils::Vector _iCubSimLeftArm::getMaxPos() const {
    return iCubSim::leftarmPosMax;
}

Utils::Vector _iCubSimLeftArm::getMinVel() const {
    return iCubSim::leftarmVelMin;
}

Utils::Vector _iCubSimLeftArm::getMaxVel() const {
    return iCubSim::leftarmVelMax;
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

iCubSimRightArm::iCubSimRightArm() {
    rightarm = new _iCubSimRightArm();
}

iCubSimLeftArm::iCubSimLeftArm() {
    leftarm = new _iCubSimLeftArm();
}

iCubSimInertial::iCubSimInertial() {
    inertial = new _iCubSimInertial();
}

