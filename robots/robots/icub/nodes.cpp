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

#include "nodes.h"

#include "commons.h"

#include <yarp/dev/IControlMode2.h>
#include <numeric>

std::vector<int> jointsFromBitmask(const std::vector<bool>& bitmask) {

    std::vector<int> ret;
    for (unsigned int i = 0; i < bitmask.size(); i++) {
        if (bitmask[i])
            ret.push_back(i);
    }
    return ret;

}


EncodersHead::EncodersHead(const HasHead& robot, double freq)
    :sec::Source(freq), robot(robot) {
    execute();  // <--- just to fill the NodeOuts
}


void EncodersHead::execute() {

    Utils::Vector hd = robot.head->encodersPos();

    head = hd;
    neck = Utils::subvector(hd, 0, 3);
    eyes = Utils::subvector(hd, 3, 6);

    neckroll = hd[1];
    neckpitch = hd[0];
    neckyaw = hd[2];
    eyestilt = hd[3];
    eyesversion = hd[4];
    eyesvergence = hd[5];

}

std::string EncodersHead::parameters() const {
    return "Head encoders of " + robot.name();
}


EncodersHeadVel::EncodersHeadVel(const HasHead& robot, double freq)
    :EncodersHead(robot, freq) {
    execute();
}

void EncodersHeadVel::execute() {

    EncodersHead::execute();

    Utils::Vector hdv = robot.head->encodersVel();

    headvel = hdv;
    neckvel = Utils::subvector(hdv, 0, 3);
    eyesvel = Utils::subvector(hdv, 3, 6);

    neckrollvel = hdv[1];
    neckpitchvel = hdv[0];
    neckyawvel = hdv[2];
    eyestiltvel = hdv[3];
    eyesversionvel = hdv[4];
    eyesvergencevel = hdv[5];

}


HeadControlCommon::HeadControlCommon(HasHead& robot, double freq)
    :sec::Node(freq), robot(robot) {

    full = false;
    sub = false;
    joints = false;

}

void HeadControlCommon::refreshInputs() {

    head.refreshData();

    neck.refreshData();
    eyes.refreshData();

    roll.refreshData();
    pitch.refreshData();
    yaw.refreshData();

    tilt.refreshData();
    version.refreshData();
    vergence.refreshData();

}

bool HeadControlCommon::connected() const {

    if (head.isConnected()) {
        full = true;
        actuated_joints = {0, 1, 2, 3, 4, 5};
    }

    if (neck.isConnected()) {
        sub = true;
        actuated_joints = {0, 1, 2};
    }

    if (eyes.isConnected()) {
        sub = true;
        std::vector<int> tmp{4, 5, 6};
        actuated_joints.insert(actuated_joints.end(), tmp.begin(), tmp.end());
    }

    auto tmp = jointsFromBitmask({pitch.isConnected(), roll.isConnected(), yaw.isConnected(), tilt.isConnected(), version.isConnected(), vergence.isConnected()});

    if (tmp.size() > 0) {
        joints = true;
        actuated_joints = tmp;
    }

    if ((full && sub) || ((full || sub) && joints)) {
        throw iCubException("[HeadPositionControl] Too many connections.");
    }

    return full || sub || joints;

}

Utils::Vector HeadControlCommon::getCmd() {

    Utils::Vector cmd;

    if (full) {
        cmd = head;
    } else if (sub) {
        if (neck.isConnected()) cmd.insert(cmd.end(), neck.getData().begin(), neck.getData().end());
        if (eyes.isConnected()) cmd.insert(cmd.end(), eyes.getData().begin(), eyes.getData().end());
    } else {
        if (pitch.isConnected()) cmd.push_back(pitch);
        if (roll.isConnected()) cmd.push_back(roll);
        if (yaw.isConnected()) cmd.push_back(yaw);
        if (tilt.isConnected()) cmd.push_back(tilt);
        if (version.isConnected()) cmd.push_back(version);
        if (vergence.isConnected()) cmd.push_back(vergence);
    }

    return cmd;

}

void HeadPositionControl::execute() {
    robot.head->movePosJoints(actuated_joints, getCmd());
}

std::string HeadPositionControl::parameters() const {
    return "Head position control of " + robot.name();
}


void HeadVelocityControl::execute() {

    robot.head->moveVelJoints(actuated_joints, getCmd());

}

std::string HeadVelocityControl::parameters() const {
    return "Head velocity control of " + robot.name();
}



EncodersTorso::EncodersTorso(const HasTorso& robot, double freq)
    :sec::Source(freq), robot(robot) {
    execute();
}

void EncodersTorso::execute() {

    Utils::Vector ts = robot.torso->encodersPos();

    torso = ts;

    yaw = ts[0];
    roll = ts[1];
    pitch = ts[2];

}

std::string EncodersTorso::parameters() const {
    return "Torso encoders of " + robot.name();
}


EncodersTorsoVel::EncodersTorsoVel(const HasTorso& robot, double freq)
    :EncodersTorso(robot, freq) {
    execute();
}

void EncodersTorsoVel::execute() {

    EncodersTorso::execute();

    Utils::Vector tsv = robot.torso->encodersVel();

    torsovel = tsv;

    yawvel = tsv[0];
    rollvel = tsv[1];
    pitchvel = tsv[2];

}


TorsoControlCommon::TorsoControlCommon(HasTorso& robot, double freq)
    :sec::Node(freq), robot(robot) {

    full = false;
    joints = false;

}

void TorsoControlCommon::refreshInputs() {

    torso.refreshData();

    roll.refreshData();
    pitch.refreshData();
    yaw.refreshData();

}

bool TorsoControlCommon::connected() const {

    if (torso.isConnected()) {
        full = true;
        actuated_joints = {0, 1, 2};
    }

    auto tmp = jointsFromBitmask({yaw.isConnected(), roll.isConnected(), pitch.isConnected()});

    if (tmp.size() > 0) {
        joints = true;
        actuated_joints = tmp;
    }

    if (full && joints) {
        throw iCubException("[TorsoPositionControl] Too many connections.");
    }

    return full || joints;

}

Utils::Vector TorsoControlCommon::getCmd() {

    Utils::Vector cmd;

    if (full) {
        cmd = torso;
    } else {
        if (yaw.isConnected()) cmd.push_back(yaw);
        if (roll.isConnected()) cmd.push_back(roll);
        if (pitch.isConnected()) cmd.push_back(pitch);
    }

    return cmd;

}

void TorsoPositionControl::execute() {

    robot.torso->movePosJoints(actuated_joints, getCmd());

}

std::string TorsoPositionControl::parameters() const {
    return "Torso position control of " + robot.name();
}


void TorsoVelocityControl::execute() {

    robot.torso->moveVelJoints(actuated_joints, getCmd());

}

std::string TorsoVelocityControl::parameters() const {
    return "Torso velocity control of " + robot.name();
}


EncodersRightArm::EncodersRightArm(const HasRightArm& robot, double freq)
    :sec::Source(freq), robot(robot) {
    execute();
}

void EncodersRightArm::execute() {

    Utils::Vector ra = robot.rightarm->encodersPos();

    arm = ra;

    shoulder_pitch = ra[0];
    shoulder_roll = ra[1];
    shoudler_yaw = ra[2];
    elbow = ra[3];
    wrist_prosup = ra[4];
    wrist_pitch = ra[5];
    wrist_yaw = ra[6];

}

std::string EncodersRightArm::parameters() const {
    return "Right arm encoders of " + robot.name();
}

EncodersRightArmVel::EncodersRightArmVel(const HasRightArm& robot, double freq)
    :EncodersRightArm(robot, freq) {
    execute();
}

void EncodersRightArmVel::execute() {

    EncodersRightArm::execute();

    Utils::Vector rav = robot.rightarm->encodersVel();

    armvel = rav;

    shoulder_pitchvel = rav[0];
    shoulder_rollvel = rav[1];
    shoudler_yawvel = rav[2];
    elbowvel = rav[3];
    wrist_prosupvel = rav[4];
    wrist_pitchvel = rav[5];
    wrist_yawvel = rav[6];

}

RightArmControlCommon::RightArmControlCommon(HasRightArm& robot, double freq)
    :sec::Node(freq), robot(robot) {

    full = false;
    sub = false;
    joints = false;

}

void RightArmControlCommon::refreshInputs() {

    fullarm.refreshData();

    arm.refreshData();
    hand.refreshData();

    shoulder_pitch.refreshData();
    shoulder_roll.refreshData();
    shoulder_yaw.refreshData();
    elbow.refreshData();
    wrist_prosup.refreshData();
    wrist_pitch.refreshData();
    wrist_yaw.refreshData();
    hand_finger.refreshData();
    thumb_oppose.refreshData();
    thumb_proximal.refreshData();
    thumb_distal.refreshData();
    index_proximal.refreshData();
    index_distal.refreshData();
    middle_proximal.refreshData();
    middle_distal.refreshData();
    pinky.refreshData();

}

bool RightArmControlCommon::connected() const {

    if (fullarm.isConnected()) {
        full = true;
        actuated_joints.resize(16);
        std::iota(actuated_joints.begin(), actuated_joints.end(), 0);
    }

    if (arm.isConnected()) {
        sub = true;
        actuated_joints.resize(7);
        std::iota(actuated_joints.begin(), actuated_joints.end(), 0);
    }

    if (hand.isConnected()) {
        sub = true;
        std::vector<int> tmp;
        tmp.resize(9);
        std::iota(tmp.begin(), tmp.end(), 7);
        actuated_joints.insert(actuated_joints.end(), tmp.begin(), tmp.end());
    }

    auto tmp = jointsFromBitmask({shoulder_pitch.isConnected(), shoulder_roll.isConnected(), shoulder_yaw.isConnected(),
                                  elbow.isConnected(), wrist_prosup.isConnected(), wrist_pitch.isConnected(),
                                  wrist_yaw.isConnected(), hand_finger.isConnected(), thumb_oppose.isConnected(),
                                  thumb_proximal.isConnected(), thumb_distal.isConnected(), index_proximal.isConnected(),
                                  index_distal.isConnected(), middle_proximal.isConnected(), middle_distal.isConnected(),
                                  pinky.isConnected()});
    if (tmp.size() > 0) {
        joints = true;
        actuated_joints = tmp;
    }

    if ((full && sub) || ((full || sub) && joints)) {
        throw iCubException("[RightArmPositionControl] Too many connections.");
    }

    return full || sub || joints;

}

Utils::Vector RightArmControlCommon::getCmd() {

    Utils::Vector cmd;

    if (full) {
        cmd = fullarm;
    } else if (sub) {
        if (arm.isConnected()) cmd.insert(cmd.end(), arm.getData().begin(), arm.getData().end());
        if (hand.isConnected()) cmd.insert(cmd.end(), hand.getData().begin(), hand.getData().end());
    } else {
        if (shoulder_pitch.isConnected()) cmd.push_back(shoulder_pitch);
        if (shoulder_roll.isConnected()) cmd.push_back(shoulder_roll);
        if (shoulder_yaw.isConnected()) cmd.push_back(shoulder_yaw);
        if (elbow.isConnected()) cmd.push_back(elbow);
        if (wrist_prosup.isConnected()) cmd.push_back(wrist_prosup);
        if (wrist_pitch.isConnected()) cmd.push_back(wrist_pitch);
        if (wrist_yaw.isConnected()) cmd.push_back(wrist_yaw);
        if (hand_finger.isConnected()) cmd.push_back(hand_finger);
        if (thumb_oppose.isConnected()) cmd.push_back(thumb_oppose);
        if (thumb_proximal.isConnected()) cmd.push_back(thumb_proximal);
        if (thumb_distal.isConnected()) cmd.push_back(thumb_distal);
        if (index_proximal.isConnected()) cmd.push_back(index_proximal);
        if (index_distal.isConnected()) cmd.push_back(index_distal);
        if (middle_proximal.isConnected()) cmd.push_back(middle_proximal);
        if (middle_distal.isConnected()) cmd.push_back(middle_distal);
        if (pinky.isConnected()) cmd.push_back(pinky);
    }

    return cmd;

}

void RightArmPositionControl::execute() {

    robot.rightarm->movePosJoints(actuated_joints, getCmd());

}

std::string RightArmPositionControl::parameters() const {
    return "Right arm position control of " + robot.name();
}

void RightArmVelocityControl::execute() {

    robot.rightarm->moveVelJoints(actuated_joints, getCmd());

}

std::string RightArmVelocityControl::parameters() const {
    return "Right arm velocity control of " + robot.name();
}

void RightArmTorqueControl::execute() {
    robot.rightarm->moveTorqueJoints(actuated_joints, getCmd());
}

std::string RightArmTorqueControl::parameters() const {
    return "Right arm torque control of " + robot.name();
}


EncodersLeftArm::EncodersLeftArm(const HasLeftArm& robot, double freq)
    :sec::Source(freq), robot(robot) {
    execute();
}

void EncodersLeftArm::execute() {

    Utils::Vector la = robot.leftarm->encodersPos();

    fullarm = la;

    arm = Utils::subvector(la, 0, 7);
    hand = Utils::subvector(la, 7, 16);

    shoulder_pitch = la[0];
    shoulder_roll = la[1];
    shoudler_yaw = la[2];
    elbow = la[3];
    wrist_prosup = la[4];
    wrist_pitch = la[5];
    wrist_yaw = la[6];

    hand_finger = la[7];
    thumb_oppose = la[8];
    thumb_proximal = la[9];
    thumb_distal = la[10];
    index_proximal = la[11];
    index_distal = la[12];
    middle_proximal = la[13];
    middle_distal = la[14];
    pinky = la[15];

}

std::string EncodersLeftArm::parameters() const {
    return "Left arm encoders of " + robot.name();
}

EncodersLeftArmVel::EncodersLeftArmVel(const HasLeftArm& robot, double freq)
    :EncodersLeftArm(robot, freq) {
    execute();
}

void EncodersLeftArmVel::execute() {

    EncodersLeftArm::execute();

    Utils::Vector lav = robot.leftarm->encodersVel();

    armvel = lav;

    shoulder_pitchvel = lav[0];
    shoulder_rollvel = lav[1];
    shoudler_yawvel = lav[2];
    elbowvel = lav[3];
    wrist_prosupvel = lav[4];
    wrist_pitchvel = lav[5];
    wrist_yawvel = lav[6];

    hand_fingervel = lav[7];
    thumb_opposevel = lav[8];
    thumb_proximalvel = lav[9];
    thumb_distalvel = lav[10];
    index_proximalvel = lav[11];
    index_distalvel = lav[12];
    middle_proximalvel = lav[13];
    middle_distalvel = lav[14];
    pinkyvel = lav[15];

}


LeftArmControlCommon::LeftArmControlCommon(HasLeftArm& robot, double freq)
    :sec::Node(freq), robot(robot) {

    full = false;
    sub = false;
    joints = false;

}

void LeftArmControlCommon::refreshInputs() {

    fullarm.refreshData();

    arm.refreshData();
    hand.refreshData();

    shoulder_pitch.refreshData();
    shoulder_roll.refreshData();
    shoulder_yaw.refreshData();
    elbow.refreshData();
    wrist_prosup.refreshData();
    wrist_pitch.refreshData();
    wrist_yaw.refreshData();
    hand_finger.refreshData();
    thumb_oppose.refreshData();
    thumb_proximal.refreshData();
    thumb_distal.refreshData();
    index_proximal.refreshData();
    index_distal.refreshData();
    middle_proximal.refreshData();
    middle_distal.refreshData();
    pinky.refreshData();

}

bool LeftArmControlCommon::connected() const {

    if (fullarm.isConnected()) {
        full = true;
        actuated_joints.resize(16);
        std::iota(actuated_joints.begin(), actuated_joints.end(), 0);
    }

    if (arm.isConnected()) {
        sub = true;
        actuated_joints.resize(7);
        std::iota(actuated_joints.begin(), actuated_joints.end(), 0);
    }

    if (hand.isConnected()) {
        sub = true;
        std::vector<int> tmp;
        tmp.resize(9);
        std::iota(tmp.begin(), tmp.end(), 7);
        actuated_joints.insert(actuated_joints.end(), tmp.begin(), tmp.end());
    }

    auto tmp = jointsFromBitmask({shoulder_pitch.isConnected(), shoulder_roll.isConnected(), shoulder_yaw.isConnected(),
                                  elbow.isConnected(), wrist_prosup.isConnected(), wrist_pitch.isConnected(),
                                  wrist_yaw.isConnected(), hand_finger.isConnected(), thumb_oppose.isConnected(),
                                  thumb_proximal.isConnected(), thumb_distal.isConnected(), index_proximal.isConnected(),
                                  index_distal.isConnected(), middle_proximal.isConnected(), middle_distal.isConnected(),
                                  pinky.isConnected()});
    if (tmp.size() > 0) {
        joints = true;
        actuated_joints = tmp;
    }

    if ((full && sub) || ((full || sub) && joints)) {
        throw iCubException("[RightArmPositionControl] Too many connections.");
    }

    return full || sub || joints;

}

Utils::Vector LeftArmControlCommon::getCmd() {

    Utils::Vector cmd;

    if (full) {
        cmd = fullarm;
    } else if (sub) {
        if (arm.isConnected()) cmd.insert(cmd.end(), arm.getData().begin(), arm.getData().end());
        if (hand.isConnected()) cmd.insert(cmd.end(), hand.getData().begin(), hand.getData().end());
    } else {
        if (shoulder_pitch.isConnected()) cmd.push_back(shoulder_pitch);
        if (shoulder_roll.isConnected()) cmd.push_back(shoulder_roll);
        if (shoulder_yaw.isConnected()) cmd.push_back(shoulder_yaw);
        if (elbow.isConnected()) cmd.push_back(elbow);
        if (wrist_prosup.isConnected()) cmd.push_back(wrist_prosup);
        if (wrist_pitch.isConnected()) cmd.push_back(wrist_pitch);
        if (wrist_yaw.isConnected()) cmd.push_back(wrist_yaw);
        if (hand_finger.isConnected()) cmd.push_back(hand_finger);
        if (thumb_oppose.isConnected()) cmd.push_back(thumb_oppose);
        if (thumb_proximal.isConnected()) cmd.push_back(thumb_proximal);
        if (thumb_distal.isConnected()) cmd.push_back(thumb_distal);
        if (index_proximal.isConnected()) cmd.push_back(index_proximal);
        if (index_distal.isConnected()) cmd.push_back(index_distal);
        if (middle_proximal.isConnected()) cmd.push_back(middle_proximal);
        if (middle_distal.isConnected()) cmd.push_back(middle_distal);
        if (pinky.isConnected()) cmd.push_back(pinky);
    }

    return cmd;

}

void LeftArmPositionControl::execute() {

    robot.leftarm->movePosJoints(actuated_joints, getCmd());

}

std::string LeftArmPositionControl::parameters() const {
    return "Left arm position control of " + robot.name();
}

void LeftArmVelocityControl::execute() {

    robot.leftarm->moveVelJoints(actuated_joints, getCmd());

}

std::string LeftArmVelocityControl::parameters() const {
    return "Left arm velocity control of " + robot.name();
}


InertialSensor::InertialSensor(const HasInertial& robot, double freq)
    :sec::Source(freq), robot(robot) {
    execute();
}

void InertialSensor::execute() {

    Utils::Vector in = robot.inertial->rotationsPos();
    Utils::Vector inv = robot.inertial->rotationsVel();

    rotations = in;
    velocities = inv;

    roll = in[0];
    pitch = in[1];
    yaw = in[2];

    rollvel = inv[0];
    pitchvel = inv[1];
    yawvel = inv[2];

}

std::string InertialSensor::parameters() const {
    return "Inertial sensor of " + robot.name();
}
