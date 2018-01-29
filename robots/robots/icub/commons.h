#ifndef ICUB_COMMONS_H
#define ICUB_COMMONS_H

#include <vector>
#include <unistd.h>

#include <yarp/sig/Vector.h>
#include <iCub/iKin/iKinFwd.h>

#include <utilities/vector.h>
#include <sec/node.h>


////////////////////
/// Legacy stuff, deprecated
///
namespace iCub {

using Head = Utils::Vector;
using NormalizedHead = Utils::Vector;
using HeadSpeed = Utils::Vector;
using Eyes = Utils::Vector;
using EyesSpeed = Utils::Vector;
using NormalizedEyes = Utils::Vector;
using Neck = Utils::Vector;
using NeckSpeed = Utils::Vector;
using NormalizedNeck = Utils::Vector;
using GFP = Utils::Vector;
using Inertial = Utils::Vector;
using InertialSpeed = Utils::Vector;

}


///////////////////////
/// \brief The iCubException class
///
class iCubException : public std::exception {

public:

    iCubException(const std::string& msg) noexcept
        :std::exception(), msg(msg) {}

    virtual const char* what() const noexcept override {
        return msg.c_str();
    }

protected:
    std::string msg;

};


/////////////////////
///
///
namespace iCubUtils {

    // yarp::Vector to std::vector conversion
    yarp::sig::Vector convert(const Utils::Vector& v);
    Utils::Vector convert(const yarp::sig::Vector& v);

}





class iCubKin {

public:

    iCubKin();

    Utils::Vector getGFP(const Utils::Vector& head_encoders) const;
    Utils::Vector getGFP(const Utils::Vector& neck_encoders, const Utils::Vector& eyes_encoders) const;
    Utils::Vector eyeToRoot(const Utils::Vector& head_encoders, const Utils::Vector& point, bool left = true) const;
    Utils::Vector rootToEye(const Utils::Vector& head_encoders, const Utils::Vector& point, bool left = true) const;

    Utils::Vector headToInertialRot(const Utils::Vector& head_encoders, const Utils::Vector& torso_encoders) const;
    Utils::Vector neckToInertialRot(const Utils::Vector& neck_encoders, const Utils::Vector& torso_encoders) const;

    std::pair<Utils::Vector, Utils::Vector> headToInertialRot(const Utils::Vector& hpos, const Utils::Vector& hvel,
                                                              const Utils::Vector& tpos, const Utils::Vector& tvel) const;
    std::pair<Utils::Vector, Utils::Vector> neckToInertialRot(const Utils::Vector& npos, const Utils::Vector& nvel,
                                                              const Utils::Vector& tpos, const Utils::Vector& tvel) const;

    Utils::Vector getLeftHandPosition(const Utils::Vector& arm_encoders) const;
    Utils::Vector getRightHandPosition(const Utils::Vector& arm_encoders) const;

protected:
    mutable iCub::iKin::iCubEye leftEye, rightEye;
    mutable iCub::iKin::iCubInertialSensor inertial;
    mutable iCub::iKin::iCubArm leftArm, rightArm;
    yarp::sig::Matrix baseMatrix;

};



////////////////////////////
///// \brief The PortSource class
/////

static unsigned int portSourceInstanceCount = 0;

template <typename PortType = yarp::sig::Vector>
class PortSource : public sec::Node {

public:
    PortSource(const std::string& portname, double freq = 30.0)
        :Node(freq), portname(portname) {
        yarp::os::Network yarp;
        std::string local = "/local/portsource" + std::to_string(portSourceInstanceCount);
        portSourceInstanceCount++;
        port.open(local);
        if (!yarp.connect(portname, local)) {
            throw iCubException("[PortSource] Unable to connect to " + portname + ".");
        }
        newvalue = false;
    }

    virtual ~PortSource() {
        port.close();
    }

    virtual void refreshInputs() {
        auto v = port.read(false);
        if (v != nullptr) {
            last_reading = *v;
            newvalue = true;
        } else {
            newvalue = false;
        }
    }

    virtual bool connected() const {
        return true;
    }

    virtual std::string parameters() const {
        return "PortSource connected to " + portname + ".";
    }

protected:
    bool newvalue;
    yarp::os::BufferedPort<PortType> port;
    PortType last_reading;
    std::string portname;

};



class VelocityObserver {

public:
    VelocityObserver(const std::string& basename, const std::string& local, double thrVel, unsigned int lenVel);
    virtual ~VelocityObserver();

    Utils::Vector derive(const Utils::Vector& x);

protected:
    Utils::Vector last;
    yarp::os::BufferedPort<yarp::os::Bottle> send_port, receive_port;
    pid_t obspid;

};


//class DelayAndObserve {

//public:
//    DelayAndObserve(const std::string& basename, const std::string& local, unsigned int steps);

//    std::pair<Utils::Vector, Utils::Vector> derive(const Utils::Vector& x);

//protected:
//    std::deque<Utils::Vector> delay;
//    VelocityObserver obs;

//};


#endif // ICUB_COMMONS_H
