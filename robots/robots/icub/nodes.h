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

#ifndef ICUB_NODES_H
#define ICUB_NODES_H

#include <utilities/vector.h>
#include <sec/node.h>
#include <sec/source.h>
#include <sec/nodelink.h>

#include "icubpart.h"

/////////////
/// Head
/////////////
class EncodersHead : public sec::Source {

public:
    EncodersHead(const HasHead& robot, double freq = 0.0);

    virtual void execute() override;

    virtual std::string parameters() const override;

    sec::NodeOut<Utils::Vector> head;
    sec::NodeOut<Utils::Vector> neck;
    sec::NodeOut<Utils::Vector> eyes;

    sec::NodeOut<double> neckroll;
    sec::NodeOut<double> neckpitch;
    sec::NodeOut<double> neckyaw;
    sec::NodeOut<double> eyestilt;
    sec::NodeOut<double> eyesversion;
    sec::NodeOut<double> eyesvergence;


protected:
    const HasHead& robot;

};


class EncodersHeadVel : public EncodersHead {

public:
    EncodersHeadVel(const HasHead& robot, double freq = 0.0);

    virtual void execute() override;

    sec::NodeOut<Utils::Vector> headvel;
    sec::NodeOut<Utils::Vector> neckvel;
    sec::NodeOut<Utils::Vector> eyesvel;

    sec::NodeOut<double> neckrollvel;
    sec::NodeOut<double> neckpitchvel;
    sec::NodeOut<double> neckyawvel;
    sec::NodeOut<double> eyestiltvel;
    sec::NodeOut<double> eyesversionvel;
    sec::NodeOut<double> eyesvergencevel;

};


class HeadControlCommon : public sec::Node {

public:
    HeadControlCommon(HasHead& robot, double freq = 0.0);

    virtual void refreshInputs() override;

    virtual bool connected() const override;

    sec::NodeIn<Utils::Vector> head;

    sec::NodeIn<Utils::Vector> neck;
    sec::NodeIn<Utils::Vector> eyes;

    sec::NodeIn<double> roll;
    sec::NodeIn<double> pitch;
    sec::NodeIn<double> yaw;
    sec::NodeIn<double> tilt;
    sec::NodeIn<double> version;
    sec::NodeIn<double> vergence;

protected:
    HasHead& robot;
    mutable bool full, sub, joints;

    Utils::Vector getCmd();

};

class HeadPositionControl : public HeadControlCommon {

public:
    using HeadControlCommon::HeadControlCommon;

    virtual void execute() override;

    virtual std::string parameters() const override;

};


class HeadVelocityControl : public HeadControlCommon {

public:
    using HeadControlCommon::HeadControlCommon;

    virtual void execute() override;

    virtual std::string parameters() const override;

};

/////////////
/// Torso
/////////////
class EncodersTorso : public sec::Source {

public:
    EncodersTorso(const HasTorso& robot, double freq = 0.0);

    virtual void execute() override;

    virtual std::string parameters() const override;

    sec::NodeOut<Utils::Vector> torso;

    sec::NodeOut<double> roll;
    sec::NodeOut<double> pitch;
    sec::NodeOut<double> yaw;


protected:
    const HasTorso& robot;

};

class EncodersTorsoVel : public EncodersTorso {

public:
    EncodersTorsoVel(const HasTorso& robot, double freq = 0.0);

    virtual void execute() override;

    sec::NodeOut<Utils::Vector> torsovel;

    sec::NodeOut<double> rollvel;
    sec::NodeOut<double> pitchvel;
    sec::NodeOut<double> yawvel;

};


class TorsoControlCommon : public sec::Node {

public:
    TorsoControlCommon(HasTorso& robot, double freq = 0.0);

    virtual void refreshInputs() override;

    virtual bool connected() const override;

    sec::NodeIn<Utils::Vector> torso;

    sec::NodeIn<double> roll;
    sec::NodeIn<double> pitch;
    sec::NodeIn<double> yaw;

protected:
    HasTorso& robot;
    mutable bool full, joints;

    Utils::Vector getCmd();

};

class TorsoPositionControl : public TorsoControlCommon {

public:
    using TorsoControlCommon::TorsoControlCommon;

    virtual void execute() override;

    virtual std::string parameters() const override;

};

class TorsoVelocityControl : public TorsoControlCommon {

public:
    using TorsoControlCommon::TorsoControlCommon;

    virtual void execute() override;

    virtual std::string parameters() const override;

};

/////////////
/// RightArm
/////////////
class EncodersRightArm : public sec::Source {

public:
    EncodersRightArm(const HasRightArm& robot, double freq = 0.0);

    virtual void execute() override;

    virtual std::string parameters() const override;

    sec::NodeOut<Utils::Vector> arm;

    sec::NodeOut<double> shoulder_pitch;
    sec::NodeOut<double> shoulder_roll;
    sec::NodeOut<double> shoudler_yaw;
    sec::NodeOut<double> elbow;
    sec::NodeOut<double> wrist_prosup;
    sec::NodeOut<double> wrist_pitch;
    sec::NodeOut<double> wrist_yaw;

    sec::NodeOut<double> hand_finger;
    sec::NodeOut<double> thumb_oppose;
    sec::NodeOut<double> thumb_proximal;
    sec::NodeOut<double> thumb_distal;
    sec::NodeOut<double> index_proximal;
    sec::NodeOut<double> index_distal;
    sec::NodeOut<double> middle_proximal;
    sec::NodeOut<double> middle_distal;
    sec::NodeOut<double> pinky;

protected:
    const HasRightArm& robot;

};

class EncodersRightArmVel : public EncodersRightArm {

public:
    EncodersRightArmVel(const HasRightArm& robot, double freq = 0.0);

    virtual void execute() override;

    sec::NodeOut<Utils::Vector> armvel;

    sec::NodeOut<double> shoulder_pitchvel;
    sec::NodeOut<double> shoulder_rollvel;
    sec::NodeOut<double> shoudler_yawvel;
    sec::NodeOut<double> elbowvel;
    sec::NodeOut<double> wrist_prosupvel;
    sec::NodeOut<double> wrist_pitchvel;
    sec::NodeOut<double> wrist_yawvel;

    sec::NodeOut<double> hand_fingervel;
    sec::NodeOut<double> thumb_opposevel;
    sec::NodeOut<double> thumb_proximalvel;
    sec::NodeOut<double> thumb_distalvel;
    sec::NodeOut<double> index_proximalvel;
    sec::NodeOut<double> index_distalvel;
    sec::NodeOut<double> middle_proximalvel;
    sec::NodeOut<double> middle_distalvel;
    sec::NodeOut<double> pinkyvel;

};


class RightArmControlCommon : public sec::Node {

public:
    RightArmControlCommon(HasRightArm& robot, double freq = 0.0);

    virtual void refreshInputs() override;

    virtual bool connected() const override;

    sec::NodeIn<Utils::Vector> fullarm;

    sec::NodeIn<Utils::Vector> arm;
    sec::NodeIn<Utils::Vector> hand;

    sec::NodeIn<double> shoulder_pitch;
    sec::NodeIn<double> shoulder_roll;
    sec::NodeIn<double> shoudler_yaw;
    sec::NodeIn<double> elbow;
    sec::NodeIn<double> wrist_prosup;
    sec::NodeIn<double> wrist_pitch;
    sec::NodeIn<double> wrist_yaw;

    sec::NodeIn<double> hand_finger;
    sec::NodeIn<double> thumb_oppose;
    sec::NodeIn<double> thumb_proximal;
    sec::NodeIn<double> thumb_distal;
    sec::NodeIn<double> index_proximal;
    sec::NodeIn<double> index_distal;
    sec::NodeIn<double> middle_proximal;
    sec::NodeIn<double> middle_distal;
    sec::NodeIn<double> pinky;

protected:
    HasRightArm& robot;
    mutable bool full, sub, joints;

    Utils::Vector getCmd();

};

class RightArmPositionControl : public RightArmControlCommon {

public:
    using RightArmControlCommon::RightArmControlCommon;

    virtual void execute() override;

    virtual std::string parameters() const override;

};

class RightArmVelocityControl : public RightArmControlCommon {

public:
    using RightArmControlCommon::RightArmControlCommon;

    virtual void execute() override;

    virtual std::string parameters() const override;

};

/////////////
/// LeftArm
/////////////
class EncodersLeftArm : public sec::Source {

public:
    EncodersLeftArm(const HasLeftArm& robot, double freq = 0.0);

    virtual void execute() override;

    virtual std::string parameters() const override;

    sec::NodeOut<Utils::Vector> fullarm;

    sec::NodeOut<Utils::Vector> arm;
    sec::NodeOut<Utils::Vector> hand;

    // arm
    sec::NodeOut<double> shoulder_pitch;
    sec::NodeOut<double> shoulder_roll;
    sec::NodeOut<double> shoudler_yaw;
    sec::NodeOut<double> elbow;
    sec::NodeOut<double> wrist_prosup;
    sec::NodeOut<double> wrist_pitch;
    sec::NodeOut<double> wrist_yaw;

    // hand
    sec::NodeOut<double> hand_finger;
    sec::NodeOut<double> thumb_oppose;
    sec::NodeOut<double> thumb_proximal;
    sec::NodeOut<double> thumb_distal;
    sec::NodeOut<double> index_proximal;
    sec::NodeOut<double> index_distal;
    sec::NodeOut<double> middle_proximal;
    sec::NodeOut<double> middle_distal;
    sec::NodeOut<double> pinky;


protected:
    const HasLeftArm& robot;

};

class EncodersLeftArmVel : public EncodersLeftArm {

public:
    EncodersLeftArmVel(const HasLeftArm& robot, double freq = 0.0);

    virtual void execute() override;

    sec::NodeOut<Utils::Vector> fullarmvel;

    sec::NodeOut<Utils::Vector> armvel;
    sec::NodeOut<Utils::Vector> handvel;

    sec::NodeOut<double> shoulder_pitchvel;
    sec::NodeOut<double> shoulder_rollvel;
    sec::NodeOut<double> shoudler_yawvel;
    sec::NodeOut<double> elbowvel;
    sec::NodeOut<double> wrist_prosupvel;
    sec::NodeOut<double> wrist_pitchvel;
    sec::NodeOut<double> wrist_yawvel;

    sec::NodeOut<double> hand_fingervel;
    sec::NodeOut<double> thumb_opposevel;
    sec::NodeOut<double> thumb_proximalvel;
    sec::NodeOut<double> thumb_distalvel;
    sec::NodeOut<double> index_proximalvel;
    sec::NodeOut<double> index_distalvel;
    sec::NodeOut<double> middle_proximalvel;
    sec::NodeOut<double> middle_distalvel;
    sec::NodeOut<double> pinkyvel;

};


class LeftArmControlCommon : public sec::Node {

public:
    LeftArmControlCommon(HasLeftArm& robot, double freq = 0.0);

    virtual void refreshInputs() override;

    virtual bool connected() const override;

    sec::NodeIn<Utils::Vector> fullarm;

    sec::NodeIn<Utils::Vector> arm;
    sec::NodeIn<Utils::Vector> hand;

    sec::NodeIn<double> shoulder_pitch;
    sec::NodeIn<double> shoulder_roll;
    sec::NodeIn<double> shoulder_yaw;
    sec::NodeIn<double> elbow;
    sec::NodeIn<double> wrist_prosup;
    sec::NodeIn<double> wrist_pitch;
    sec::NodeIn<double> wrist_yaw;

    sec::NodeIn<double> hand_finger;
    sec::NodeIn<double> thumb_oppose;
    sec::NodeIn<double> thumb_proximal;
    sec::NodeIn<double> thumb_distal;
    sec::NodeIn<double> index_proximal;
    sec::NodeIn<double> index_distal;
    sec::NodeIn<double> middle_proximal;
    sec::NodeIn<double> middle_distal;
    sec::NodeIn<double> pinky;

protected:
    HasLeftArm& robot;
    mutable bool full, sub, joints;

    Utils::Vector getCmd();

};

class LeftArmPositionControl : public LeftArmControlCommon {

public:
    using LeftArmControlCommon::LeftArmControlCommon;

    virtual void execute() override;

    virtual std::string parameters() const override;

};

class LeftArmVelocityControl : public LeftArmControlCommon {

public:
    using LeftArmControlCommon::LeftArmControlCommon;

    virtual void execute() override;

    virtual std::string parameters() const override;

};



/////////////
/// Inertial
/////////////
class InertialSensor : public sec::Source {

public:
    InertialSensor(const HasInertial& robot, double freq = 0.0);

    virtual void execute() override;

    virtual std::string parameters() const override;

    sec::NodeOut<Utils::Vector> rotations;
    sec::NodeOut<Utils::Vector> velocities;

    sec::NodeOut<double> roll;
    sec::NodeOut<double> pitch;
    sec::NodeOut<double> yaw;

    sec::NodeOut<double> rollvel;
    sec::NodeOut<double> pitchvel;
    sec::NodeOut<double> yawvel;

protected:
    const HasInertial& robot;

};

#endif // ICUB_NODES_H
