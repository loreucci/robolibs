/*
 * Copyright (C) 2014-2019 Lorenzo Vannucci
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
 */

#include "icubpart.h"

#include <cmath>

#include <sec/sec.h>

#include "commons.h"


void DriverPart::activate(const std::string& robotname, const std::string& localname) {

    pos.resize(dof());
    vel.resize(dof());

    // motor driver
    yarp::os::Property options;
    options.put("device", "remote_controlboard");
    options.put("local", "/" + localname + "/" + yarpname());
    options.put("remote", "/" + robotname + "/" + yarpname());
    if (!driver.open(options)) {
        throw iCubException("[iCubRobot] Unable to create " + name() + " device.");
    }

    setRefSpeeds(0.5);

}

void DriverPart::deactivate() {
    driver.close();
}

void DriverPart::setRefSpeeds(double velperc) {

    yarp::dev::IPositionControl* p;
    driver.view(p);
    p->setRefSpeeds((velperc*getMaxVel()).data());
    p->setRefAccelerations(getMaxVel().data());

}

void DriverPart::setMovementPrecision(double newprec) {
    if (newprec <= 0.0) {
        std::cerr << "[iCubRobot(" + name() + ")] Trying to set non-positive movement precision, ignoring." << std::endl;
        return;
    }
    precision = newprec;
}

void DriverPart::refresh() {

    yarp::dev::IEncoders* encs = getEncs();
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

    yarp::dev::IPositionControl* posctrl = getPosCtrl();

    auto _refs = trimToLimitsPos(refs);

    if (!wait) {
        posctrl->positionMove(_refs.data());
    } else {
        if (sec::isVerbose()) {
            std::cerr << "[iCubRobot(" + name() + ")] Waiting for " << name() << "movement to end..." << std::endl;
        }
        Utils::Vector encs;
        posctrl->positionMove(_refs.data());
        do {
            refresh();
            encs = encodersPos();
            sec::sleep(10.0);
        } while (Utils::distance(encs, _refs) > precision);
        if (sec::isVerbose()) {
            std::cerr << "[iCubRobot(" + name() + ")] " << name() << "movement completed" << std::endl;
        }

    }

}

void DriverPart::moveVel(const Utils::Vector& refs, bool wait) {

    if (refs.size() != dof()) {
        throw iCubException("[iCubRobot(" + name() + ")] Wrong size of reference vector.");
    }

    yarp::dev::IVelocityControl* velctrl = getVelCtrl();

    auto _refs = trimToLimitsVel(refs);

    if (!wait) {
        velctrl->velocityMove(_refs.data());
    } else {
        if (sec::isVerbose()) {
            std::cerr << "[iCubRobot(" + name() + ")] Waiting for " << name() << "movement to end..." << std::endl;
        }
        Utils::Vector encs;
        velctrl->velocityMove(_refs.data());
        do {
            refresh();
            encs = encodersVel();
            sec::sleep(10.0);
        } while (Utils::distance(encs, _refs) > precision);
        if (sec::isVerbose()) {
            std::cerr << "[iCubRobot(" + name() + ")] " << name() << "movement completed" << std::endl;
        }
    }

}

void DriverPart::moveTorque(const Utils::Vector& refs) {

    if (refs.size() != dof()) {
        throw iCubException("[iCubRobot(" + name() + ")] Wrong size of reference vector.");
    }

    yarp::dev::ITorqueControl* torctrl = getTorCtrl();

    auto _refs = trimTolimitsTorque(refs);

    torctrl->setRefTorques(_refs.data());

}

void DriverPart::movePosJoint(unsigned int joint, double ref, bool wait) {

    if (joint >= dof()) {
        throw iCubException("[iCubRobot(" + name() + ")] Wrong joint index.");
    }

    yarp::dev::IPositionControl* posctrl = getPosCtrl();

    double _ref = trimToLimitsPosJoint(joint, ref);

    if (!wait) {
        posctrl->positionMove(joint, _ref);
    } else {
        if (sec::isVerbose()) {
            std::cerr << "[iCubRobot(" + name() + ")] Waiting for " << name() << "movement to end..." << std::endl;
        }
        double enc;
        posctrl->positionMove(joint, _ref);
        do {
            refresh();
            enc = encodersPos()[joint];
            sec::sleep(10.0);
        } while (std::abs(_ref-enc) > precision);
        if (sec::isVerbose()) {
            std::cerr << "[iCubRobot(" + name() + ")] " << name() << "movement completed" << std::endl;
        }
    }

}

void DriverPart::moveVelJoint(unsigned int joint, double ref, bool wait) {

    if (joint >= dof()) {
        throw iCubException("[iCubRobot(" + name() + ")] Wrong joint index.");
    }

    yarp::dev::IVelocityControl* velctrl = getVelCtrl();

    double _ref = trimToLimitsVelJoint(joint, ref);

    if (!wait) {
        velctrl->velocityMove(joint, _ref);
    } else {
        if (sec::isVerbose()) {
            std::cerr << "[iCubRobot(" + name() + ")] Waiting for " << name() << "movement to end..." << std::endl;
        }
        double enc;
        velctrl->velocityMove(joint, _ref);
        do {
            refresh();
            enc = encodersVel()[joint];
            sec::sleep(10.0);
        } while (std::abs(_ref-enc) > precision);
        if (sec::isVerbose()) {
            std::cerr << "[iCubRobot(" + name() + ")] " << name() << "movement completed" << std::endl;
        }
    }

}

void DriverPart::moveTorqueJoint(unsigned int joint, double ref) {

    if (joint >= dof()) {
        throw iCubException("[iCubRobot(" + name() + ")] Wrong joint index.");
    }

    yarp::dev::ITorqueControl* torctrl = getTorCtrl();

    double _ref = trimToLimitsTorqueJoint(joint, ref);

    torctrl->setRefTorque(joint, _ref);

}

void DriverPart::movePosJoints(const std::vector<int>& joints, const Utils::Vector& refs, bool wait) {

    if (refs.size() != joints.size()) {
        throw iCubException("[iCubRobot(" + name() + ")] Wrong size of reference vector.");
    }

    yarp::dev::IPositionControl2* posctrl = getPosCtrl();

    // trimming
    Utils::Vector _refs = refs;
    for (unsigned int i = 0; i < joints.size(); i++) {
        _refs[i] = trimToLimitsPosJoint(joints[i], refs[i]);
    }

    if (!wait) {
        posctrl->positionMove(joints.size(), joints.data(), _refs.data());
    } else {
        // TODO
    }

}

void DriverPart::moveVelJoints(const std::vector<int>& joints, const Utils::Vector& refs, bool wait) {

    if (refs.size() != joints.size()) {
        throw iCubException("[iCubRobot(" + name() + ")] Wrong size of reference vector.");
    }

    yarp::dev::IVelocityControl2* velctrl = getVelCtrl();

    // trimming
    Utils::Vector _refs = refs;
    for (unsigned int i = 0; i < joints.size(); i++) {
        _refs[i] = trimToLimitsVelJoint(joints[i], refs[i]);
    }

    if (!wait) {
        velctrl->velocityMove(joints.size(), joints.data(), _refs.data());
    } else {
        // TODO
    }

}

void DriverPart::moveTorqueJoints(const std::vector<int>& joints, const Utils::Vector& refs) {

    if (refs.size() != joints.size()) {
        throw iCubException("[iCubRobot(" + name() + ")] Wrong size of reference vector.");
    }

    yarp::dev::ITorqueControl* torctrl = getTorCtrl();

    // trimming
    Utils::Vector _refs = refs;
    for (unsigned int i = 0; i < joints.size(); i++) {
        _refs[i] = trimToLimitsTorqueJoint(joints[i], refs[i]);
    }

    torctrl->setRefTorques(joints.size(), joints.data(), _refs.data());

}

void DriverPart::setControlMode(const int mode) {

    yarp::dev::IControlMode2* cm = getCtrlMd();
    for (unsigned int i = 0; i < dof(); i++)
        cm->setControlMode(i, mode);

}

void DriverPart::setControlMode(unsigned int joint, const int mode) {

    yarp::dev::IControlMode2* cm = getCtrlMd();
    cm->setControlMode(joint, mode);

}

void DriverPart::setControlMode(const std::vector<int>& joints, std::vector<int> modes) {

    yarp::dev::IControlMode2* cm = getCtrlMd();
    cm->setControlModes(joints.size(), joints.data(), modes.data());

}

void DriverPart::home() {
    if (sec::isVerbose()) {
        std::cerr << "[iCubRobot(" + name() + ")] Homing " << name() << "..." << std::endl;
    }
    movePos(getInitialPosition(), true);
    if (sec::isVerbose()) {
        std::cerr << "[iCubRobot(" + name() + ")] Homing ended" << std::endl;
    }
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

yarp::dev::IEncoders*DriverPart::getEncs() {

    if (_encs == nullptr) {
        driver.view(_encs);
        if (_encs == nullptr)
            throw iCubException("[iCubRobot(" + name() + ")] Unable to initialize encoders on driver.");
    }
    return _encs;

}

yarp::dev::IPositionControl2* DriverPart::getPosCtrl() {

    if (_posctrl == nullptr) {
        driver.view(_posctrl);
        if (_posctrl == nullptr)
            throw iCubException("[iCubRobot(" + name() + ")] Unable to initialize position control on driver.");
    }
    return _posctrl;

}

yarp::dev::IVelocityControl2*DriverPart::getVelCtrl() {

    if (_velctrl == nullptr) {
        driver.view(_velctrl);
        if (_velctrl == nullptr)
            throw iCubException("[iCubRobot(" + name() + ")] Unable to initialize velocity control on driver.");
    }
    return _velctrl;

}

yarp::dev::ITorqueControl* DriverPart::getTorCtrl() {

    if (_torctrl == nullptr) {
        driver.view(_torctrl);
        if (_torctrl == nullptr)
            throw iCubException("[iCubRobot(" + name() + ")] Unable to initialize torque control on driver.");
    }
    return _torctrl;

}

yarp::dev::IControlMode2* DriverPart::getCtrlMd() {

    if (_ctrlmd == nullptr) {
        driver.view(_ctrlmd);
        if (_ctrlmd == nullptr)
            throw iCubException("[iCubRobot(" + name() + ")] Unable to change control mode on driver.");
    }
    return _ctrlmd;

}

Utils::Vector DriverPart::trimToLimitsPos(const Utils::Vector& refs) const {
    return trim(refs, getMinPos(), getMaxPos());
}

Utils::Vector DriverPart::trimToLimitsVel(const Utils::Vector& refs) const {
    return trim(refs, getMinVel(), getMaxVel());
}

Utils::Vector DriverPart::trimTolimitsTorque(const Utils::Vector& refs) const {
    return trim(refs, getMinTorque(), getMaxTorque());
}

double DriverPart::trimToLimitsPosJoint(unsigned int j, double ref) const {
    return trimjoint(ref, getMinPos()[j], getMaxPos()[j]);
}

double DriverPart::trimToLimitsVelJoint(unsigned int j, double ref) const {
    return trimjoint(ref, getMinVel()[j], getMaxVel()[j]);
}

double DriverPart::trimToLimitsTorqueJoint(unsigned int j, double ref) const {
    return trimjoint(ref, getMinTorque()[j], getMaxTorque()[j]);
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

std::string _Inertial::yarpname() const {
    return "inertial";
}


unsigned int _Head::dof() const {
    return 6;
}

std::string _Head::yarpname() const {
    return "head";
}

unsigned int _Torso::dof() const {
    return 3;
}

std::string _Torso::yarpname() const {
    return "torso";
}

unsigned int _RightArm::dof() const {
    return 16;
}

std::string _RightArm::yarpname() const {
    return "right_arm";
}

unsigned int _LeftArm::dof() const {
    return 16;
}

std::string _LeftArm::yarpname() const {
    return "left_arm";
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
