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
extern const Utils::Vector headTorMin;
extern const Utils::Vector headTorMax;
extern const Utils::Vector headInitial;

extern const Utils::Vector torsoPosMin;
extern const Utils::Vector torsoPosMax;
extern const Utils::Vector torsoVelMin;
extern const Utils::Vector torsoVelMax;
extern const Utils::Vector torsoTorMin;
extern const Utils::Vector torsoTorMax;
extern const Utils::Vector headInitial;

extern const Utils::Vector rightarmPosMin;
extern const Utils::Vector rightarmPosMax;
extern const Utils::Vector rightarmVelMin;
extern const Utils::Vector rightarmVelMax;
extern const Utils::Vector rightarmTorMin;
extern const Utils::Vector rightarmTorMax;
extern const Utils::Vector rightarmInitial;

extern const Utils::Vector leftarmPosMin;
extern const Utils::Vector leftarmPosMax;
extern const Utils::Vector leftarmVelMin;
extern const Utils::Vector leftarmVelMax;
extern const Utils::Vector leftarmTorMin;
extern const Utils::Vector leftarmTorMax;
extern const Utils::Vector leftarmInitial;

}


class _iCubSimHead : public _Head {

public:
    _iCubSimHead();

    virtual std::string name() const override;

    virtual Utils::Vector getMinPos() const override;
    virtual Utils::Vector getMaxPos() const override;
    virtual Utils::Vector getMinVel() const override;
    virtual Utils::Vector getMaxVel() const override;
    virtual Utils::Vector getMinTorque() const override;
    virtual Utils::Vector getMaxTorque() const override;

};


class _iCubSimTorso : public _Torso {

public:
    _iCubSimTorso();

    virtual std::string name() const override;

    virtual Utils::Vector getMinPos() const override;
    virtual Utils::Vector getMaxPos() const override;
    virtual Utils::Vector getMinVel() const override;
    virtual Utils::Vector getMaxVel() const override;
    virtual Utils::Vector getMinTorque() const override;
    virtual Utils::Vector getMaxTorque() const override;

};


class _iCubSimRightArm : public _RightArm {

public:
    _iCubSimRightArm();

    virtual std::string name() const override;

    virtual Utils::Vector getMinPos() const override;
    virtual Utils::Vector getMaxPos() const override;
    virtual Utils::Vector getMinVel() const override;
    virtual Utils::Vector getMaxVel() const override;
    virtual Utils::Vector getMinTorque() const override;
    virtual Utils::Vector getMaxTorque() const override;

};

class _iCubSimLeftArm : public _LeftArm {

public:
    _iCubSimLeftArm();

    virtual std::string name() const override;

    virtual Utils::Vector getMinPos() const override;
    virtual Utils::Vector getMaxPos() const override;
    virtual Utils::Vector getMinVel() const override;
    virtual Utils::Vector getMaxVel() const override;
    virtual Utils::Vector getMinTorque() const override;
    virtual Utils::Vector getMaxTorque() const override;

};

class _iCubSimInertial : public _Inertial {

public:
    virtual std::string name() const override;

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
