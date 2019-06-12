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
    }

    if (neck.isConnected() || eyes.isConnected()) {
        sub = true;
    }

    if (roll.isConnected() || pitch.isConnected() || yaw.isConnected() ||
        tilt.isConnected() || version.isConnected() || vergence.isConnected()) {
        joints = true;
    }

    if ((full && sub) || ((full || sub) && joints)) {
        throw iCubException("[HeadPositionControl] Too many connections.");
    }

    return full || sub || joints;

}

Utils::Vector HeadControlCommon::getCmd() {

    Utils::Vector cmd = robot.head->encodersPos();

    if (full) {
        cmd = head;
    } else if (sub) {
        cmd = Utils::joinVectors(neck, eyes);
    } else {
        cmd[0] = pitch.isConnected() ? pitch : cmd[0];
        cmd[1] = roll.isConnected() ? roll : cmd[1];
        cmd[2] = yaw.isConnected() ? yaw : cmd[2];
        cmd[3] = tilt.isConnected() ? tilt : cmd[3];
        cmd[4] = version.isConnected() ? version : cmd[4];
        cmd[5] = vergence.isConnected() ? vergence : cmd[5];
    }

    return cmd;

}

void HeadPositionControl::execute() {
    robot.head->movePos(getCmd());
}

std::string HeadPositionControl::parameters() const {
    return "Head position control of " + robot.name();
}


void HeadVelocityControl::execute() {

    robot.head->moveVel(getCmd());

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
    }

    if (roll.isConnected() || pitch.isConnected() || yaw.isConnected()) {
        joints = true;
    }

    if (full && joints) {
        throw iCubException("[TorsoPositionControl] Too many connections.");
    }

    return full || joints;

}

Utils::Vector TorsoControlCommon::getCmd() {

    Utils::Vector cmd = robot.torso->encodersPos();

    if (full) {
        cmd = torso;
    } else {
        cmd[2] = pitch.isConnected() ? pitch : cmd[2];
        cmd[1] = roll.isConnected() ? roll : cmd[1];
        cmd[0] = yaw.isConnected() ? yaw : cmd[0];
    }

    return cmd;

}

void TorsoPositionControl::execute() {

    robot.torso->movePos(getCmd());

}

std::string TorsoPositionControl::parameters() const {
    return "Torso position control of " + robot.name();
}


void TorsoVelocityControl::execute() {

    robot.torso->moveVel(getCmd());

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
    shoudler_yaw.refreshData();
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
    }

    if (arm.isConnected() || hand.isConnected()) {
        sub = true;
    }

    if (shoulder_pitch.isConnected() || shoulder_roll.isConnected() || shoudler_yaw.isConnected() ||
        elbow.isConnected() || wrist_prosup.isConnected() || wrist_pitch.isConnected() ||
        wrist_yaw.isConnected() || hand_finger.isConnected() || thumb_oppose.isConnected() ||
        thumb_proximal.isConnected() || thumb_distal.isConnected() || index_proximal.isConnected() ||
        index_distal.isConnected() || middle_proximal.isConnected() || middle_distal.isConnected() ||
        pinky.isConnected()) {
        joints = true;
    }

    if ((full && sub) || ((full || sub) && joints)) {
        throw iCubException("[RightArmPositionControl] Too many connections.");
    }

    return full || sub || joints;

}

Utils::Vector RightArmControlCommon::getCmd() {

    Utils::Vector cmd = robot.rightarm->encodersPos();

    if (full) {
        cmd = fullarm;
    } else if (sub) {
        cmd = Utils::joinVectors(arm, hand);
    } else {
        cmd[0] = shoulder_pitch.isConnected() ? shoulder_pitch : cmd[0];
        cmd[1] = shoulder_roll.isConnected() ? shoulder_roll : cmd[1];
        cmd[2] = shoudler_yaw.isConnected() ? shoudler_yaw : cmd[2];
        cmd[3] = elbow.isConnected() ? elbow : cmd[3];
        cmd[4] = wrist_prosup.isConnected() ? wrist_prosup : cmd[4];
        cmd[5] = wrist_pitch.isConnected() ? wrist_pitch : cmd[5];
        cmd[6] = wrist_yaw.isConnected() ? wrist_yaw : cmd[6];
        cmd[7] = hand_finger.isConnected() ? hand_finger : cmd[7];
        cmd[8] = thumb_oppose.isConnected() ? thumb_oppose : cmd[8];
        cmd[9] = thumb_proximal.isConnected() ? thumb_proximal : cmd[9];
        cmd[10] = thumb_distal.isConnected() ? thumb_distal : cmd[10];
        cmd[11] = index_proximal.isConnected() ? index_proximal : cmd[11];
        cmd[12] = index_distal.isConnected() ? index_distal : cmd[12];
        cmd[13] = middle_proximal.isConnected() ? middle_proximal : cmd[13];
        cmd[14] = middle_distal.isConnected() ? middle_distal : cmd[14];
        cmd[15] = pinky.isConnected() ? pinky : cmd[15];
    }

    return cmd;

}

void RightArmPositionControl::execute() {

    robot.rightarm->movePos(getCmd());

}

std::string RightArmPositionControl::parameters() const {
    return "Right arm position control of " + robot.name();
}

void RightArmVelocityControl::execute() {

    robot.rightarm->moveVel(getCmd());

}

std::string RightArmVelocityControl::parameters() const {
    return "Right arm velocity control of " + robot.name();
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
    }

    if (arm.isConnected() || hand.isConnected()) {
        sub = true;
    }

    if (shoulder_pitch.isConnected() || shoulder_roll.isConnected() || shoulder_yaw.isConnected() ||
        elbow.isConnected() || wrist_prosup.isConnected() || wrist_pitch.isConnected() ||
        wrist_yaw.isConnected() || hand_finger.isConnected() || thumb_oppose.isConnected() ||
        thumb_proximal.isConnected() || thumb_distal.isConnected() || index_proximal.isConnected() ||
        index_distal.isConnected() || middle_proximal.isConnected() || middle_distal.isConnected() ||
        pinky.isConnected()) {
        joints = true;
    }

    if ((full && sub) || ((full || sub) && joints)) {
        throw iCubException("[LeftArmPositionControl] Too many connections.");
    }

    return full || sub || joints;

}

Utils::Vector LeftArmControlCommon::getCmd() {

    Utils::Vector cmd = robot.leftarm->encodersPos();

    if (full) {
        cmd = fullarm;
    } else if (sub) {
        cmd = Utils::joinVectors(arm, hand);
    } else {
        cmd[0] = shoulder_pitch.isConnected() ? shoulder_pitch : cmd[0];
        cmd[1] = shoulder_roll.isConnected() ? shoulder_roll : cmd[1];
        cmd[2] = shoulder_yaw.isConnected() ? shoulder_yaw : cmd[2];
        cmd[3] = elbow.isConnected() ? elbow : cmd[3];
        cmd[4] = wrist_prosup.isConnected() ? wrist_prosup : cmd[4];
        cmd[5] = wrist_pitch.isConnected() ? wrist_pitch : cmd[5];
        cmd[6] = wrist_yaw.isConnected() ? wrist_yaw : cmd[6];
        cmd[7] = hand_finger.isConnected() ? hand_finger : cmd[7];
        cmd[8] = thumb_oppose.isConnected() ? thumb_oppose : cmd[8];
        cmd[9] = thumb_proximal.isConnected() ? thumb_proximal : cmd[9];
        cmd[10] = thumb_distal.isConnected() ? thumb_distal : cmd[10];
        cmd[11] = index_proximal.isConnected() ? index_proximal : cmd[11];
        cmd[12] = index_distal.isConnected() ? index_distal : cmd[12];
        cmd[13] = middle_proximal.isConnected() ? middle_proximal : cmd[13];
        cmd[14] = middle_distal.isConnected() ? middle_distal : cmd[14];
        cmd[15] = pinky.isConnected() ? pinky : cmd[15];
    }

    return cmd;

}

void LeftArmPositionControl::execute() {

    robot.leftarm->movePos(getCmd());

}

std::string LeftArmPositionControl::parameters() const {
    return "Left arm position control of " + robot.name();
}

void LeftArmVelocityControl::execute() {

    robot.leftarm->moveVel(getCmd());

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
