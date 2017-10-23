#ifndef ICUB_NODES_H
#define ICUB_NODES_H

#include <utilities/vector.h>
#include <sec/node.h>
#include <sec/source.h>
#include <sec/nodelink.h>

#include "icubpart.h"


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


class HeadPositionControl : public sec::Node {

public:
    HeadPositionControl(HasHead& robot, double freq = 0.0);

    virtual void refreshInputs() override;

    virtual bool connected() const override;

    virtual void execute() override;

    virtual std::string parameters() const override;

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

};


class HeadVelocityControl : public sec::Node {

public:
    HeadVelocityControl(HasHead& robot, double freq = 0.0);

    virtual void refreshInputs() override;

    virtual bool connected() const override;

    virtual void execute() override;

    virtual std::string parameters() const override;

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

};


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

class TorsoPositionControl : public sec::Node {

public:
    TorsoPositionControl(HasTorso& robot, double freq = 0.0);

    virtual void refreshInputs() override;

    virtual bool connected() const override;

    virtual void execute() override;

    virtual std::string parameters() const override;

    sec::NodeIn<Utils::Vector> torso;

    sec::NodeIn<double> roll;
    sec::NodeIn<double> pitch;
    sec::NodeIn<double> yaw;

protected:
    HasTorso& robot;
    mutable bool full, joints;

};

class TorsoVelocityControl : public sec::Node {

public:
    TorsoVelocityControl(HasTorso& robot, double freq = 0.0);

    virtual void refreshInputs() override;

    virtual bool connected() const override;

    virtual void execute() override;

    virtual std::string parameters() const override;

    sec::NodeIn<Utils::Vector> torso;

    sec::NodeIn<double> roll;
    sec::NodeIn<double> pitch;
    sec::NodeIn<double> yaw;

protected:
    HasTorso& robot;
    mutable bool full, joints;

};


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

};

class RightArmPositionControl : public sec::Node {

public:
    RightArmPositionControl(HasRightArm& robot, double freq = 0.0);

    virtual void refreshInputs() override;

    virtual bool connected() const override;

    virtual void execute() override;

    virtual std::string parameters() const override;

    sec::NodeIn<Utils::Vector> arm;

    sec::NodeIn<double> shoulder_pitch;
    sec::NodeIn<double> shoulder_roll;
    sec::NodeIn<double> shoudler_yaw;
    sec::NodeIn<double> elbow;
    sec::NodeIn<double> wrist_prosup;
    sec::NodeIn<double> wrist_pitch;
    sec::NodeIn<double> wrist_yaw;

protected:
    HasRightArm& robot;
    Utils::Vector cmd;
    mutable bool full, joints;

};

class RightArmVelocityControl : public sec::Node {

public:
    RightArmVelocityControl(HasRightArm& robot, double freq = 0.0);

    virtual void refreshInputs() override;

    virtual bool connected() const override;

    virtual void execute() override;

    virtual std::string parameters() const override;

    sec::NodeIn<Utils::Vector> arm;

    sec::NodeIn<double> shoulder_pitch;
    sec::NodeIn<double> shoulder_roll;
    sec::NodeIn<double> shoudler_yaw;
    sec::NodeIn<double> elbow;
    sec::NodeIn<double> wrist_prosup;
    sec::NodeIn<double> wrist_pitch;
    sec::NodeIn<double> wrist_yaw;

protected:
    HasRightArm& robot;
    mutable bool full, joints;

};

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
