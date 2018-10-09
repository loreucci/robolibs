#include "icubpart.h"

#include <yarp/dev/IEncoders.h>
#include <yarp/dev/IPositionControl.h>
#include <yarp/dev/IVelocityControl.h>
#include <yarp/dev/IControlMode2.h>
#include <cmath>

#include "commons.h"


void DriverPart::activate(const std::string&, const std::string&) {
    pos.resize(dof());
    vel.resize(dof());
}

void DriverPart::deactivate() {
    driver.close();
}

void DriverPart::refresh() {

    yarp::dev::IEncoders* encs = nullptr;
    driver.view(encs);
    if (encs == nullptr)
        throw iCubException("[iCubRobot(" + name() + ")] Unable to use driver.");
    double p[dof()];
    double v[dof()];
    encs->getEncoders(p);
    encs->getEncoderSpeeds(v);
    mtx.lock();
    for (unsigned int i = 0; i < dof(); i++) {
        pos[i] = p[i];
        vel[i] = v[i];
    }
    mtx.unlock();

}

Utils::Vector DriverPart::encodersPos() const {

    Utils::Vector ret;
    mtx.lock();
    ret = pos;
    mtx.unlock();
    return ret;

}

Utils::Vector DriverPart::encodersVel() const {

    Utils::Vector ret;
    mtx.lock();
    ret = vel;
    mtx.unlock();
    return ret;

}

void DriverPart::movePos(const Utils::Vector& refs, bool wait) {

    if (refs.size() != dof()) {
        throw iCubException("[iCubRobot(" + name() + ")] Wrong size of reference vector.");
    }

    yarp::dev::IPositionControl* posctrl = nullptr;
    driver.view(posctrl);
    if (posctrl == nullptr)
        throw iCubException("[iCubRobot(" + name() + ")] Unable to use driver.");

    auto _refs = trimToLimitsPos(refs);

    if (!wait) {
        posctrl->positionMove(_refs.data());
    } else {
        Utils::Vector encs;
        do {
            posctrl->positionMove(_refs.data());
            refresh();
            encs = encodersPos();
        } while (Utils::distance(encs, _refs) > 0.05);

    }

}

void DriverPart::moveVel(const Utils::Vector& refs, bool wait) {

    if (refs.size() != dof()) {
        throw iCubException("[iCubRobot(" + name() + ")] Wrong size of reference vector.");
    }

    yarp::dev::IVelocityControl* velctrl = nullptr;
    driver.view(velctrl);
    if (velctrl == nullptr)
        throw iCubException("[iCubRobot(" + name() + ")] Unable to use driver.");

    auto _refs = trimToLimitsVel(refs);

    if (!wait) {
        velctrl->velocityMove(_refs.data());
    } else {
        Utils::Vector encs;
        do {
            velctrl->velocityMove(_refs.data());
            refresh();
            encs = encodersVel();
        } while (Utils::distance(encs, _refs) > 0.05);

    }

}

void DriverPart::movePosJoint(unsigned int joint, double ref, bool wait) {

    if (joint >= dof()) {
        throw iCubException("[iCubRobot(" + name() + ")] Wrong joint index.");
    }

    yarp::dev::IPositionControl* posctrl = nullptr;
    driver.view(posctrl);
    if (posctrl == nullptr)
        throw iCubException("[iCubRobot(" + name() + ")] Unable to use driver.");

    double _ref = trimToLimitsPosJoint(joint, ref);

    if (!wait) {
        posctrl->positionMove(joint, _ref);
    } else {
        double enc;
        do {
            posctrl->positionMove(joint, _ref);
            refresh();
            enc = encodersPos()[joint];
        } while (std::abs(_ref-enc) > 0.05);

    }

}

void DriverPart::moveVelJoint(unsigned int joint, double ref, bool wait) {

    if (joint >= dof()) {
        throw iCubException("[iCubRobot(" + name() + ")] Wrong joint index.");
    }

    yarp::dev::IVelocityControl* velctrl = nullptr;
    driver.view(velctrl);
    if (velctrl == nullptr)
        throw iCubException("[iCubRobot(" + name() + ")] Unable to use driver.");

    double _ref = trimToLimitsVelJoint(joint, ref);

    if (!wait) {
        velctrl->velocityMove(joint, _ref);
    } else {
        double enc;
        do {
            velctrl->velocityMove(joint, _ref);
            refresh();
            enc = encodersVel()[joint];
        } while (std::abs(_ref-enc) > 0.05);

    }

}

void DriverPart::setControlMode(const int mode) {

    yarp::dev::IControlMode2* cm;
    driver.view(cm);
    for (unsigned int i = 0; i < dof(); i++)
        cm->setControlMode(i, mode);

}

void DriverPart::home() {
    movePos(getInitialPosition(), true);
}

Utils::Vector DriverPart::getInitialPosition() const {
    return initpos;
}

void DriverPart::setInitialPosition(const Utils::Vector& initpos) {
    if (initpos.size() != dof()) {
        throw iCubException("[iCubRobot(" + name() + ")] Wrong size of initial position vector.");
    }
    this->initpos = initpos;
}

Utils::Vector DriverPart::trimToLimitsPos(const Utils::Vector& refs) const {
    return trim(refs, getMinPos(), getMaxPos());
}

Utils::Vector DriverPart::trimToLimitsVel(const Utils::Vector& refs) const {
    return trim(refs, getMinVel(), getMaxVel());
}

double DriverPart::trimToLimitsPosJoint(unsigned int j, double ref) const {
    return trimjoint(ref, getMinPos()[j], getMaxPos()[j]);
}

double DriverPart::trimToLimitsVelJoint(unsigned int j, double ref) const {
    return trimjoint(ref, getMinVel()[j], getMaxVel()[j]);
}

Utils::Vector DriverPart::trim(const Utils::Vector& refs, const Utils::Vector& min, const Utils::Vector& max) {

    Utils::Vector ret = refs;
    for (unsigned int i = 0; i < ret.size(); i++) {
        ret[i] = std::max(min[i], ret[i]);
        ret[i] = std::min(max[i], ret[i]);
    }
    return ret;

}

double DriverPart::trimjoint(double ref, double min, double max) {

    double ret = ref;
    ret = std::max(min, ret);
    ret = std::min(max, ret);
    return ret;

}


void _Inertial::activate(const std::string& robotname, const std::string& localname) {

    rotpos.resize(3);
    rotvel.resize(3);

    port.open("/"+localname+"/inertial");
    if (!yarpnetwork.connect("/"+robotname+"/inertial", "/"+localname+"/inertial")) {
        throw iCubException("[iCubRobot(" + name() + ")] Unable to connect to /"+robotname+"/inertial");
    }


}

void _Inertial::deactivate() {
    port.close();
}

void _Inertial::refresh() {

    yarp::sig::Vector* v = port.read(false);
    if (v != nullptr) {
        mtx.lock();
        rotpos[0] = (*v)[0];
        rotpos[1] = (*v)[1];
        rotpos[2] = (*v)[2];
        rotvel[0] = (*v)[6];
        rotvel[1] = (*v)[7];
        rotvel[2] = (*v)[8];
        mtx.unlock();
    }

}

Utils::Vector _Inertial::rotationsPos() const {
    Utils::Vector ret;
    mtx.lock();
    ret = rotpos;
    mtx.unlock();
    return ret;
}

Utils::Vector _Inertial::rotationsVel() const {
    Utils::Vector ret;
    mtx.lock();
    ret = rotvel;
    mtx.unlock();
    return ret;
}


unsigned int _Head::dof() const {
    return 6;
}

unsigned int _Torso::dof() const {
    return 3;
}

unsigned int _RightArm::dof() const {
    return 16;
}

unsigned int _LeftArm::dof() const {
    return 16;
}


void HasHead::activate(const std::string &robotname, const std::string &localname) {
    head->activate(robotname, localname);
}

void HasHead::deactivate() {
    head->deactivate();
    delete head;
    head = nullptr;
}

void HasHead::refresh() {
    head->refresh();
}

std::string HasHead::name() const {
    return head->name();
}

void HasTorso::activate(const std::string &robotname, const std::string &localname) {
    torso->activate(robotname, localname);
}

void HasTorso::deactivate() {
    torso->deactivate();
    delete torso;
    torso = nullptr;
}

void HasTorso::refresh() {
    torso->refresh();
}

std::string HasTorso::name() const {
    return torso->name();
}

void HasRightArm::activate(const std::string& robotname, const std::string& localname) {
    rightarm->activate(robotname, localname);
}

void HasRightArm::deactivate() {
    rightarm->deactivate();
    delete rightarm;
    rightarm = nullptr;
}

void HasRightArm::refresh() {
    rightarm->refresh();
}

std::string HasRightArm::name() const {
    return rightarm->name();
}

void HasLeftArm::activate(const std::string& robotname, const std::string& localname) {
    leftarm->activate(robotname, localname);
}

void HasLeftArm::deactivate() {
    leftarm->deactivate();
}

void HasLeftArm::refresh() {
    leftarm->refresh();
}

std::string HasLeftArm::name() const {
    return leftarm->name();
}

void HasInertial::activate(const std::string& robotname, const std::string& localname) {
    inertial->activate(robotname, localname);
}

void HasInertial::deactivate() {
    inertial->deactivate();
    delete inertial;
    inertial = nullptr;
}

void HasInertial::refresh() {
    inertial->refresh();
}

std::string HasInertial::name() const {
    return inertial->name();
}
