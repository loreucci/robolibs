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

#ifndef ICUBPART_H
#define ICUBPART_H

#include <string>
#include <mutex>
#include <yarp/dev/PolyDriver.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/sig/Vector.h>

#include <utilities/vector.h>

#include <yarp/dev/IEncoders.h>
#include <yarp/dev/IPositionControl2.h>
#include <yarp/dev/IVelocityControl2.h>
#include <yarp/dev/ITorqueControl.h>
#include <yarp/dev/IControlMode2.h>


class iCubPart {

public:
    virtual void activate(const std::string& robotname, const std::string& localname) = 0;

    virtual void deactivate() = 0;

    virtual void refresh() = 0;

    virtual std::string name() const = 0;
    virtual std::string yarpname() const = 0;

};


class DriverPart : public iCubPart {

public:

    // just sets the encoders size, should be called by subclasses
    virtual void activate(const std::string& robotname, const std::string& localname) override;

    virtual void deactivate() override;

    void setRefSpeeds(double velperc = 1.0);
    void setMovementPrecision(double newprec = 0.05);

    // encoders
    virtual void refresh() override;
    virtual unsigned int dof() const = 0;
    virtual Utils::Vector encodersPos() const;
    virtual Utils::Vector encodersVel() const;

    // move
    virtual void movePos(const Utils::Vector& refs, bool wait = false);
    virtual void moveVel(const Utils::Vector& refs, bool wait = false);
    virtual void moveTorque(const Utils::Vector& refs);
    virtual void movePosJoint(unsigned int joint, double ref, bool wait = false);
    virtual void moveVelJoint(unsigned int joint, double ref, bool wait = false);
    virtual void moveTorqueJoint(unsigned int joint, double ref);
    virtual void movePosJoints(const std::vector<int>& joints, const Utils::Vector& refs, bool wait = false);
    virtual void moveVelJoints(const std::vector<int>& joints, const Utils::Vector& refs, bool wait = false);
    virtual void moveTorqueJoints(const std::vector<int>& joints, const Utils::Vector& refs);
    void setControlMode(const int mode);
    void setControlMode(unsigned int joint, const int mode);
    void setControlMode(const std::vector<int>& joints, std::vector<int> modes);
    void home();

    // limits
    virtual Utils::Vector getMinPos() const = 0;
    virtual Utils::Vector getMaxPos() const = 0;
    virtual Utils::Vector getMinVel() const = 0;
    virtual Utils::Vector getMaxVel() const = 0;
    virtual Utils::Vector getMinTorque() const = 0;
    virtual Utils::Vector getMaxTorque() const = 0;
    Utils::Vector getInitialPosition() const;
    void setInitialPosition(const Utils::Vector& initpos);

protected:

    // stored encoders
    Utils::Vector pos;
    Utils::Vector vel;
    Utils::Vector initpos;
    mutable std::mutex mtx;

    double precision = 0.05;

    yarp::dev::PolyDriver driver;
    yarp::dev::IEncoders* _encs = nullptr;
    yarp::dev::IEncoders* getEncs();
    yarp::dev::IPositionControl2* _posctrl = nullptr;
    yarp::dev::IPositionControl2* getPosCtrl();
    yarp::dev::IVelocityControl2* _velctrl = nullptr;
    yarp::dev::IVelocityControl2* getVelCtrl();
    yarp::dev::ITorqueControl* _torctrl = nullptr;
    yarp::dev::ITorqueControl* getTorCtrl();
    yarp::dev::IControlMode2* _ctrlmd = nullptr;
    yarp::dev::IControlMode2* getCtrlMd();

    Utils::Vector trimToLimitsPos(const Utils::Vector& refs) const;
    Utils::Vector trimToLimitsVel(const Utils::Vector& refs) const;
    Utils::Vector trimTolimitsTorque(const Utils::Vector& refs) const;
    double trimToLimitsPosJoint(unsigned int j, double ref) const;
    double trimToLimitsVelJoint(unsigned int j, double ref) const;
    double trimToLimitsTorqueJoint(unsigned int j, double ref) const;
    static Utils::Vector trim(const Utils::Vector& refs, const Utils::Vector& min, const Utils::Vector& max);
    static double trimjoint(double ref, double min, double max);

};


class _Inertial : public iCubPart {

public:

    virtual void activate(const std::string& robotname, const std::string& localname) override;
    virtual void deactivate() override;

    virtual void refresh() override;

    virtual Utils::Vector rotationsPos() const;
    virtual Utils::Vector rotationsVel() const;

    virtual std::string yarpname() const override;

protected:
    // stored data
    Utils::Vector rotpos;
    Utils::Vector rotvel;
    mutable std::mutex mtx;

    yarp::os::BufferedPort<yarp::sig::Vector> port;

};

//////////////////////
// common driver parts

class _Head : public DriverPart {

public:
    virtual unsigned int dof() const override;

    virtual std::string yarpname() const override;


};

class _Torso : public DriverPart {

public:
    virtual unsigned int dof() const override;

    virtual std::string yarpname() const override;


};

class _RightArm : public DriverPart {

public:
    virtual unsigned int dof() const override;

    virtual std::string yarpname() const override;


};

class _LeftArm : public DriverPart {

public:
    virtual unsigned int dof() const override;

    virtual std::string yarpname() const override;


};


//////////////////////
/// traits
///
class HasHead {

public:
    _Head* head = nullptr;

    void activate(const std::string& robotname, const std::string& localname);
    void deactivate();
    void refresh();
    std::string name() const;

};

class HasTorso {

public:
    _Torso* torso = nullptr;

    void activate(const std::string& robotname, const std::string& localname);
    void deactivate();
    void refresh();
    std::string name() const;

};

class HasRightArm {

public:
    _RightArm* rightarm = nullptr;

    void activate(const std::string& robotname, const std::string& localname);
    void deactivate();
    void refresh();
    std::string name() const;

};

class HasLeftArm {

public:
    _LeftArm* leftarm = nullptr;

    void activate(const std::string& robotname, const std::string& localname);
    void deactivate();
    void refresh();
    std::string name() const;

};

class HasInertial {

public:
    _Inertial* inertial = nullptr;

    void activate(const std::string& robotname, const std::string& localname);
    void deactivate();
    void refresh();
    std::string name() const;

};

#endif // ICUBPART_H
