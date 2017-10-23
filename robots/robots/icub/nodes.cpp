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


HeadPositionControl::HeadPositionControl(HasHead& robot, double freq)
    :sec::Node(freq), robot(robot) {

    robot.head->setControlMode(VOCAB_CM_MIXED);

    full = false;
    sub = false;
    joints = false;

}

void HeadPositionControl::refreshInputs() {

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

bool HeadPositionControl::connected() const {

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
        throw iCubException("HeadPositionControl: too many connections.");
    }

    return full || sub || joints;

}

void HeadPositionControl::execute() {

    Utils::Vector cmd;

    if (full) {
        cmd = head;
    } else if (sub) {
        cmd.resize(6);
        cmd[0] = neck.getData()[0];
        cmd[1] = neck.getData()[1];
        cmd[2] = neck.getData()[2];
        cmd[3] = eyes.getData()[0];
        cmd[4] = eyes.getData()[1];
        cmd[5] = eyes.getData()[2];
    } else {
        cmd.resize(6);
        cmd[0] = pitch;
        cmd[1] = roll;
        cmd[2] = yaw;
        cmd[3] = tilt;
        cmd[4] = version;
        cmd[5] = vergence;
    }

    robot.head->movePos(cmd);

}

std::string HeadPositionControl::parameters() const {
    return "Head position control of " + robot.name();
}


HeadVelocityControl::HeadVelocityControl(HasHead& robot, double freq)
    :sec::Node(freq), robot(robot) {

    robot.head->setControlMode(VOCAB_CM_MIXED);

    full = false;
    sub = false;
    joints = false;

}

void HeadVelocityControl::refreshInputs() {

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

bool HeadVelocityControl::connected() const {

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
        throw iCubException("HeadPositionControl: too many connections.");
    }

    return full || sub || joints;

}

void HeadVelocityControl::execute() {

    Utils::Vector cmd;

    if (full) {
        cmd = head;
    } else if (sub) {
        cmd.resize(6);
        cmd[0] = neck.getData()[0];
        cmd[1] = neck.getData()[1];
        cmd[2] = neck.getData()[2];
        cmd[3] = eyes.getData()[0];
        cmd[4] = eyes.getData()[1];
        cmd[5] = eyes.getData()[2];
    } else {
        cmd.resize(6);
        cmd[0] = pitch;
        cmd[1] = roll;
        cmd[2] = yaw;
        cmd[3] = tilt;
        cmd[4] = version;
        cmd[5] = vergence;
    }

    robot.head->moveVel(cmd);

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


TorsoPositionControl::TorsoPositionControl(HasTorso& robot, double freq)
    :sec::Node(freq), robot(robot) {

    robot.torso->setControlMode(VOCAB_CM_MIXED);

    full = false;
    joints = false;

}

void TorsoPositionControl::refreshInputs() {

    torso.refreshData();

    roll.refreshData();
    pitch.refreshData();
    yaw.refreshData();

}

bool TorsoPositionControl::connected() const {

    if (torso.isConnected()) {
        full = true;
    }

    if (roll.isConnected() || pitch.isConnected() || yaw.isConnected()) {
        joints = true;
    }

    if (full && joints) {
        throw iCubException("TorsoPositionControl: too many connections.");
    }

    return full || joints;

}

void TorsoPositionControl::execute() {

    Utils::Vector cmd;

    if (full) {
        cmd = torso;
    } else {
        cmd.resize(3);
        cmd[2] = pitch;
        cmd[1] = roll;
        cmd[0] = yaw;
    }

    robot.torso->movePos(cmd);

}

std::string TorsoPositionControl::parameters() const {
    return "Torso position control of " + robot.name();
}


TorsoVelocityControl::TorsoVelocityControl(HasTorso& robot, double freq)
    :sec::Node(freq), robot(robot){

    robot.torso->setControlMode(VOCAB_CM_MIXED);

    full = false;
    joints = false;

}

void TorsoVelocityControl::refreshInputs() {

    torso.refreshData();

    roll.refreshData();
    pitch.refreshData();
    yaw.refreshData();

}

bool TorsoVelocityControl::connected() const {

    if (torso.isConnected()) {
        full = true;
    }

    if (roll.isConnected() || pitch.isConnected() || yaw.isConnected()) {
        joints = true;
    }

    if (full && joints) {
        throw iCubException("TorsoPositionControl: too many connections.");
    }

    return full || joints;

}

void TorsoVelocityControl::execute() {

    Utils::Vector cmd;

    if (full) {
        cmd = torso;
    } else {
        cmd.resize(3);
        cmd[2] = pitch;
        cmd[1] = roll;
        cmd[0] = yaw;
    }

    robot.torso->moveVel(cmd);

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
    wrist_yawvel = rav[6];;

}

RightArmPositionControl::RightArmPositionControl(HasRightArm& robot, double freq)
    :sec::Node(freq), robot(robot) {

    robot.rightarm->setControlMode(VOCAB_CM_MIXED);

    full = false;
    joints = false;

    cmd = robot.rightarm->getInitialPosition();

}

void RightArmPositionControl::refreshInputs() {

    arm.refreshData();

    shoulder_pitch.refreshData();
    shoulder_roll.refreshData();
    shoudler_yaw.refreshData();
    elbow.refreshData();
    wrist_prosup.refreshData();
    wrist_pitch.refreshData();
    wrist_yaw.refreshData();

}

bool RightArmPositionControl::connected() const {

    if (arm.isConnected()) {
        full = true;
    }

    if (shoulder_pitch.isConnected() || shoulder_roll.isConnected() || shoudler_yaw.isConnected() ||
            elbow.isConnected() ||
            wrist_prosup.isConnected() || wrist_pitch.isConnected() || wrist_yaw.isConnected()) {
        joints = true;
    }

    if (full && joints) {
        throw iCubException("TorsoPositionControl: too many connections.");
    }

    return full || joints;

}

void RightArmPositionControl::execute() {

    if (full) {
        cmd = arm;
    } else {
        cmd.resize(7);
        cmd[0] = shoulder_pitch.isConnected() ? shoulder_pitch : cmd[0];
        cmd[1] = shoulder_roll.isConnected() ? shoulder_roll : cmd[1];
        cmd[2] = shoudler_yaw.isConnected() ? shoudler_yaw : cmd[2];
        cmd[3] = elbow.isConnected() ? elbow : cmd[3];
        cmd[4] = wrist_prosup.isConnected() ? wrist_prosup : cmd[4];
        cmd[5] = wrist_pitch.isConnected() ? wrist_pitch : cmd[5];
        cmd[6] = wrist_yaw.isConnected() ? wrist_yaw : cmd[6];
    }

    robot.rightarm->movePos(cmd);

}

std::string RightArmPositionControl::parameters() const {
    return "Right arm position control of " + robot.name();
}

RightArmVelocityControl::RightArmVelocityControl(HasRightArm& robot, double freq)
    :sec::Node(freq), robot(robot) {

    robot.rightarm->setControlMode(VOCAB_CM_MIXED);

    full = false;
    joints = false;

}

void RightArmVelocityControl::refreshInputs() {

    arm.refreshData();

    shoulder_pitch.refreshData();
    shoulder_roll.refreshData();
    shoudler_yaw.refreshData();
    elbow.refreshData();
    wrist_prosup.refreshData();
    wrist_pitch.refreshData();
    wrist_yaw.refreshData();

}

bool RightArmVelocityControl::connected() const {

    if (arm.isConnected()) {
        full = true;
    }

    if (shoulder_pitch.isConnected() || shoulder_roll.isConnected() || shoudler_yaw.isConnected() ||
            elbow.isConnected() ||
            wrist_prosup.isConnected() || wrist_pitch.isConnected() || wrist_yaw.isConnected()) {
        joints = true;
    }

    if (full && joints) {
        throw iCubException("TorsoPositionControl: too many connections.");
    }

    return full || joints;

}

void RightArmVelocityControl::execute() {

    Utils::Vector cmd;

    if (full) {
        cmd = arm;
    } else {
        cmd.resize(7);
        cmd[0] = shoulder_pitch;
        cmd[1] = shoulder_roll;
        cmd[2] = shoudler_yaw;
        cmd[3] = elbow;
        cmd[4] = wrist_prosup;
        cmd[5] = wrist_pitch;
        cmd[6] = wrist_yaw;
    }

    robot.rightarm->moveVel(cmd);

}

std::string RightArmVelocityControl::parameters() const {
    return "Right arm velocity control of " + robot.name();
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
