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

#include "icubsim.h"

#include <yarp/os/Network.h>

#include "commons.h"


namespace iCubSim {

const CameraParameters cameraParameters = CameraParameters{320, 240, 257.34, 257.34, 160, 120};

const Utils::Vector headPosMin =  { -40,  -70,  -55,  -35,  -50,    0};
const Utils::Vector headPosMax =  {  30,   60,   55,   15,   52,   90};
const Utils::Vector headVelMin =  {-100, -100, -100, -100, -100, -100};
const Utils::Vector headVelMax =  {+100, +100, +100, +100, +100, +100};
const Utils::Vector headTorMin =  {   0,    0,    0,    0,    0,    0};
const Utils::Vector headTorMax =  {   0,    0,    0,    0,    0,    0};
const Utils::Vector headInitial = {   0,    0,    0,    0,    0,    0};

const Utils::Vector torsoPosMin =  { -50,  -30,  -10};
const Utils::Vector torsoPosMax =  {  50,   30,   70};
const Utils::Vector torsoVelMin =  {-400, -400, -400};
const Utils::Vector torsoVelMax =  {+400, +400, +400};
const Utils::Vector torsoTorMin =  { -12,  -12,  -12};
const Utils::Vector torsoTorMax =  { +12,  +12,  +12};
const Utils::Vector torsoInitial = {   0,    0,    0};

const Utils::Vector rightarmPosMin =  { -95,     0,  -37, 15.5,  -90,  -90,  -20,    0,    0,    0,    0,    0,    0,    0,    0,    0};
const Utils::Vector rightarmPosMax =  {  10, 160.8,   80,  106,   90,    0,   40,   60,   90,   90,  180,   90,  180,   90,  180,  270};
const Utils::Vector rightarmVelMin =  {-100,  -100, -100, -100, -100, -100, -100, -100, -100, -100, -100, -100, -100, -100, -100, -100};
const Utils::Vector rightarmVelMax =  {+100,  +100, +100, +100, +100, +100, +100, +100, +100, +100, +100, +100, +100, +100, +100, +100};
const Utils::Vector rightarmTorMin =  {  -8,    -8,   -8,   -8,   -2,   -2,   -2,   -2,    0,    0,    0,    0,    0,    0,    0,    0};
const Utils::Vector rightarmTorMax =  {  +8,    +8,   +8,   +8,   +2,   +2,   +2,   +2,    0,    0,    0,    0,    0,    0,    0,    0};
const Utils::Vector rightarmInitial = { -25,    20,    0,   50,    0,    0,    0,   60,   20,   20,   20,   10,   10,   10,   10,   10};

const Utils::Vector leftarmPosMin =  {-94.5,     0,  -36,   19,  -90,  -90,  -20,    0,    0,    0,    0,    0,    0,    0,    0,    0};
const Utils::Vector leftarmPosMax =  {  9.5,   161,   80,  106,   90,    0,   40,   60,   90,   90,  180,   90,  180,   90,  180,  270};
const Utils::Vector leftarmVelMin =  { -100,  -100, -100, -100, -100, -100, -100, -100, -100, -100, -100, -100, -100, -100, -100, -100};
const Utils::Vector leftarmVelMax =  { +100,  +100, +100, +100, +100, +100, +100, +100, +100, +100, +100, +100, +100, +100, +100, +100};
const Utils::Vector leftarmTorMin =  {   -8,    -8,   -8,   -8,   -2,   -2,   -2,   -2,    0,    0,    0,    0,    0,    0,    0,    0};
const Utils::Vector leftarmTorMax =  {   +8,    +8,   +8,   +8,   +2,   +2,   +2,   +2,    0,    0,    0,    0,    0,    0,    0,    0};
const Utils::Vector leftarmInitial = {  -25,    20,    0,   50,    0,    0,    0,   60,   20,   20,   20,   10,   10,   10,   10,   10};

}

_iCubSimHead::_iCubSimHead() {
    initpos = iCubSim::headInitial;
}

std::string _iCubSimHead::name() const {
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

Utils::Vector _iCubSimHead::getMinTorque() const {
    return iCubSim::headTorMin;
}

Utils::Vector _iCubSimHead::getMaxTorque() const {
    return iCubSim::headTorMax;
}


_iCubSimTorso::_iCubSimTorso() {
    initpos = iCubSim::torsoInitial;
}

std::string _iCubSimTorso::name() const {
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

Utils::Vector _iCubSimTorso::getMinTorque() const{
    return iCubSim::torsoTorMin;
}

Utils::Vector _iCubSimTorso::getMaxTorque() const {
    return iCubSim::torsoTorMax;
}


_iCubSimRightArm::_iCubSimRightArm() {
    initpos = iCubSim::rightarmInitial;
}

std::string _iCubSimRightArm::name() const {
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

Utils::Vector _iCubSimRightArm::getMinTorque() const {
    return iCubSim::rightarmTorMin;
}

Utils::Vector _iCubSimRightArm::getMaxTorque() const {
    return iCubSim::rightarmTorMax;
}


_iCubSimLeftArm::_iCubSimLeftArm() {
    initpos = iCubSim::leftarmInitial;
}

std::string _iCubSimLeftArm::name() const {
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

Utils::Vector _iCubSimLeftArm::getMinTorque() const {
    return iCubSim::leftarmTorMin;
}

Utils::Vector _iCubSimLeftArm::getMaxTorque() const {
    return iCubSim::leftarmTorMax;
}

std::string _iCubSimInertial::name() const {
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

