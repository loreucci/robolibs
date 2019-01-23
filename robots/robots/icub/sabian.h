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
extern const Utils::Vector headInitial;

extern const Utils::Vector headMask;

}


class _SabianHead : public _Head {

public:
    _SabianHead();

    virtual void activate(const std::string& robotname, const std::string& localname) override;
    virtual void deactivate() override;

    virtual void refresh() override;

    virtual void movePos(const Utils::Vector& refs, bool wait = false) override;
    virtual void moveVel(const Utils::Vector& refs, bool wait = false) override;
    virtual void movePosJoint(unsigned int joint, double ref, bool wait = false) override;
    virtual void moveVelJoint(unsigned int joint, double ref, bool wait = false) override;

    virtual std::string name() const override;

    virtual Utils::Vector getMinPos() const override;
    virtual Utils::Vector getMaxPos() const override;
    virtual Utils::Vector getMinVel() const override;
    virtual Utils::Vector getMaxVel() const override;

protected:
    VelocityObserver* obs;

    Utils::Vector applyMask(const Utils::Vector& v);

};

//class _RightArm : public DriverPart {
//public:
//    virtual unsigned int dof() const override;

////    virtual void refresh() override;

////    virtual void movePos(const Utils::Vector& refs, bool wait = false);
////    virtual void moveVel(const Utils::Vector& refs, bool wait = false);

////private:
////    static Utils::Vector handposition;

//};

class _SabianInertial : public _Inertial {

public:
    virtual std::string name() const override;

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
