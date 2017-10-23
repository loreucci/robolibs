#include "commons.h"

#include <cmath>
#include <sys/wait.h>
#include <signal.h>

#include <utilities/utilities.h>

// operation with vectors
using namespace yarp::math;

////////////////////
///
///
namespace iCubUtils {


yarp::sig::Vector convert(const Utils::Vector& v) {
    yarp::sig::Vector ret;
    for (auto& d : v)
        ret.push_back(d);
    return ret;
}

Utils::Vector convert(const yarp::sig::Vector& v) {
    std::vector<double> ret;
    for (unsigned int i = 0; i < v.size(); i++) {
        ret.push_back(v[i]);
    }
    return ret;
}

}




/////////////////
/// \brief iCubKin
///
iCubKin::iCubKin() {

    // eyes kinematics
    leftEye = iCub::iKin::iCubEye("left_v1");
    rightEye = iCub::iKin::iCubEye("right_v1");
    leftEye.releaseLink(0);
    leftEye.releaseLink(1);
    leftEye.releaseLink(2);
    rightEye.releaseLink(0);
    rightEye.releaseLink(1);
    rightEye.releaseLink(2);

    // inertial
    inertial = iCub::iKin::iCubInertialSensor("v1");

    // arms
    leftArm = iCub::iKin::iCubArm("left");
    rightArm = iCub::iKin::iCubArm("right");
    leftArm.releaseLink(0);
    leftArm.releaseLink(1);
    leftArm.releaseLink(2);
    rightArm.releaseLink(0);
    rightArm.releaseLink(1);
    rightArm.releaseLink(2);

    // set base Matrix
    baseMatrix = yarp::sig::Matrix(4, 4);
    baseMatrix.zero();
    baseMatrix(0, 1) = -1;
    baseMatrix(1, 2) = 1;
    baseMatrix(1, 3) = 0.5976;
    baseMatrix(2, 0) = -1;
    baseMatrix(2, 3) = -0.026;
    baseMatrix(3, 3) = 1;

}


Utils::Vector iCubKin::getGFP(const Utils::Vector& head_encoders) const {

    if (head_encoders.size() != 6) {
        throw iCubException("iCubKin: wrong input size.");
    }

    //    iCub::iKin::iCubEye leftEye("left_v1"), rightEye("right_v1");
    //    leftEye = iCub::iKin::iCubEye("left_v1");
    //    rightEye = iCub::iKin::iCubEye("right_v1");
    //    leftEye.releaseLink(0);
    //    leftEye.releaseLink(1);
    //    leftEye.releaseLink(2);
    //    rightEye.releaseLink(0);
    //    rightEye.releaseLink(1);
    //    rightEye.releaseLink(2);

    yarp::sig::Vector gfp(4);
    gfp[3] = 1;
    std::vector<double> ret(3);

    yarp::sig::Vector ang(8);
    ang.zero();

    ang[3] = head_encoders[0];
    ang[4] = head_encoders[1];
    ang[5] = head_encoders[2];
    ang[6] = head_encoders[3];
    ang[7] = (head_encoders[4] + head_encoders[5])/2.0;
    ang = Utils::PI*ang/180.0;
    leftEye.setAng(ang);
    //        ang[7] = Utils::PI*((joints[4] - joints[5])/2.0)/180.0;
    ang[7] = Utils::PI*((head_encoders[4] - head_encoders[5])/2.0)/180.0;
    rightEye.setAng(ang);

    yarp::sig::Matrix HL=leftEye.getH();
    yarp::sig::Matrix HR=rightEye.getH();

    HL(3,3)=HR(3,3)=0.0;

    double qty1=iCub::ctrl::dot(HR,2,HL,2);
    yarp::sig::Matrix H1=HL-HR;
    yarp::sig::Matrix H2L=HL-qty1*HR;
    yarp::sig::Matrix H2R=qty1*HL-HR;
    double qty2L=iCub::ctrl::dot(H2L,2,H1,3);
    double qty2R=iCub::ctrl::dot(H2R,2,H1,3);
    double qty3=qty1*qty1-1.0;

    if (std::fabs(qty3)<1e-6) {
        throw iCubException("getGFP: unable to find gfp.");
        return ret;
    }

    double tL=qty2L/qty3;
    double tR=qty2R/qty3;

    for (int i=0; i<3; i++)
        gfp[i]=0.5*(HL(i,3)+tL*HL(i,2)+HR(i,3)+tR*HR(i,2));

    gfp = baseMatrix*gfp;

    for (int i=0; i<3; i++)
        ret[i] = gfp[i];

    return ret; // remove for sabian

    // with tracker only
    //    auto gfp1 = eyeToRoot({0, 0, ret[2]}, joints, torso);

    //    return gfp1;

}

Utils::Vector iCubKin::getGFP(const Utils::Vector& neck_encoders, const Utils::Vector& eyes_encoders) const {
    Utils::Vector head_encoders = neck_encoders;
    head_encoders.insert(head_encoders.end(), eyes_encoders.begin(), eyes_encoders.end());
    return getGFP(head_encoders);
}

Utils::Vector iCubKin::eyeToRoot(const Utils::Vector& head_encoders, const Utils::Vector& point, bool left) const {

    if (head_encoders.size() != 6 || point.size() != 3) {
        throw iCubException("iCubKin: wrong input size.");
    }

    yarp::sig::Vector p = iCubUtils::convert(point);
    p.push_back(1);

    yarp::sig::Vector ang(8);
    ang.zero();

    ang[3] = head_encoders[0];
    ang[4] = head_encoders[1];
    ang[5] = head_encoders[2];
    ang[6] = head_encoders[3];

    yarp::sig::Matrix m;
    if (left) {
        //        ang[7] = (joints[4] + joints[5])/2.0;
        ang[7] = (head_encoders[4] + head_encoders[5])/2.0;
        ang = Utils::PI*ang/180.0;
        leftEye.setAng(ang);
        m = leftEye.getH();
    } else {
        //        ang[7] = (joints[4] - joints[5])/2.0;
        ang[7] = (head_encoders[4] - head_encoders[5])/2.0;
        ang = Utils::PI*ang/180.0;
        rightEye.setAng(ang);
        m = rightEye.getH();
    }

    p = m*p;

    p = baseMatrix*p;

    p.pop_back();

    return iCubUtils::convert(p);

}

Utils::Vector iCubKin::rootToEye(const Utils::Vector& head_encoders, const Utils::Vector& point, bool left) const {

    if (head_encoders.size() != 6 || point.size() != 3) {
        throw iCubException("iCubKin: wrong input size.");
    }

    yarp::sig::Vector p = iCubUtils::convert(point);
    p.push_back(1);

    yarp::sig::Vector ang(8);
    ang.zero();

    ang[3] = head_encoders[0];
    ang[4] = head_encoders[1];
    ang[5] = head_encoders[2];
    ang[6] = head_encoders[3];

    yarp::sig::Matrix m;
    if (left) {
        ang[7] = (head_encoders[4] + head_encoders[5])/2.0;
        ang = Utils::PI*ang/180.0;
        leftEye.setAng(ang);
        m = leftEye.getH();
    } else {
        ang[7] = (head_encoders[4] - head_encoders[5])/2.0;
        ang = Utils::PI*ang/180.0;
        rightEye.setAng(ang);
        m = rightEye.getH();
    }

    p = yarp::math::luinv(baseMatrix)*p;

    p = yarp::math::luinv(m)*p;

    p.pop_back();

    return iCubUtils::convert(p);

}

Utils::Vector iCubKin::headToInertialRot(const Utils::Vector& head_encoders, const Utils::Vector& torso_encoders) const {

    if (head_encoders.size() != 6) {
        throw iCubException("iCubKin: wrong input size.");
    }

    Utils::Vector neck_encoders(6);
    neck_encoders[0] = head_encoders[0];
    neck_encoders[1] = head_encoders[1];
    neck_encoders[2] = head_encoders[2];
    return neckToInertialRot(neck_encoders, torso_encoders);
}

Utils::Vector iCubKin::neckToInertialRot(const Utils::Vector& neck_encoders, const Utils::Vector& torso_encoders) const {

    if (neck_encoders.size() != 3) {
        throw iCubException("iCubKin: wrong input size.");
    }


    yarp::sig::Vector vec(6);

    vec[0] = torso_encoders[0];
    vec[1] = torso_encoders[1];
    vec[2] = torso_encoders[2];
    vec[3] = neck_encoders[0];
    vec[4] = neck_encoders[1];
    vec[5] = neck_encoders[2];

    vec = Utils::PI*vec/180.0;
    inertial.setAng(vec);
    yarp::sig::Vector x = inertial.EndEffPose(false);
    x = -1*x/Utils::PI*180.0;

    Utils::Vector res(3);
    res[0] = x[3];
    res[1] = x[4];
    res[2] = x[5];
    return res;

}

std::pair<Utils::Vector, Utils::Vector> iCubKin::headToInertialRot(const Utils::Vector& hpos, const Utils::Vector& hvel, const Utils::Vector& tpos, const Utils::Vector& tvel) const {

    if (hpos.size() != 6 || hvel.size() != 6) {
        throw iCubException("iCubKin: wrong input size.");
    }

    Utils::Vector npos(3);
    npos[0] = hpos[0];
    npos[1] = hpos[1];
    npos[2] = hpos[2];

    Utils::Vector nvel(3);
    nvel[0] = hvel[0];
    nvel[1] = hvel[1];
    nvel[2] = hvel[2];
    return neckToInertialRot(npos, nvel, tpos, tvel);

}

std::pair<Utils::Vector, Utils::Vector> iCubKin::neckToInertialRot(const Utils::Vector& npos, const Utils::Vector& nvel, const Utils::Vector& tpos, const Utils::Vector& tvel) const {

    if (npos.size() != 3 || nvel.size() != 3) {
        throw iCubException("iCubKin: wrong input size.");
    }

    yarp::sig::Vector vec(6);

    vec[0] = tpos[0];
    vec[1] = tpos[1];
    vec[2] = tpos[0];
    vec[3] = npos[0];
    vec[4] = npos[1];
    vec[5] = npos[2];
    vec = Utils::PI*vec/180.0;
    inertial.setAng(vec);

    // orientation
    yarp::sig::Vector x = inertial.EndEffPose(false);
    x = -1*x/Utils::PI*180.0;

    Utils::Vector res1(3);
    res1[0] = x[3];
    res1[1] = x[4];
    res1[2] = x[5];

    // orientation speed
    yarp::sig::Vector q(6);
    q[0] = tvel[0];
    q[1] = tvel[1];
    q[2] = tvel[2];
    q[3] = nvel[0];
    q[4] = nvel[1];
    q[5] = nvel[2];
    auto eeffvel = inertial.GeoJacobian() * q;
    Utils::Vector res2(3);
    res2[0] = eeffvel[3];
    res2[1] = eeffvel[4];
    res2[2] = eeffvel[5];

    return std::make_pair(res1, res2);

}

Utils::Vector iCubKin::getLeftHandPosition(const Utils::Vector& arm_encoders) const {

    if (arm_encoders.size() != 7) {
        throw iCubException("iCubKin: wrong input size.");
    }

    yarp::sig::Vector ang(10);
    ang.zero();

    ang[3] = arm_encoders[0];
    ang[4] = arm_encoders[1];
    ang[5] = arm_encoders[2];
    ang[6] = arm_encoders[3];
    ang[7] = arm_encoders[4];
    ang[8] = arm_encoders[5];
    ang[9] = arm_encoders[6];
    ang = Utils::PI*ang/180.0;

    leftArm.setAng(ang);

    yarp::sig::Vector p = rightArm.EndEffPose();
    p.pop_back();
    p.pop_back();
    p.pop_back();
    p.pop_back();
    p.push_back(1);

    p = baseMatrix*p;

    p.pop_back();

    return iCubUtils::convert(p);

}

using namespace Utils;

Utils::Vector iCubKin::getRightHandPosition(const Utils::Vector& arm_encoders) const {

    if (arm_encoders.size() != 7) {
        throw iCubException("iCubKin: wrong input size.");
    }

    yarp::sig::Vector ang(10);
    ang.zero();

    ang[3] = arm_encoders[0];
    ang[4] = arm_encoders[1];
    ang[5] = arm_encoders[2];
    ang[6] = arm_encoders[3];
    ang[7] = arm_encoders[4];
    ang[8] = arm_encoders[5];
    ang[9] = arm_encoders[6];
    ang = Utils::PI*ang/180.0;
    rightArm.setAng(ang);

    yarp::sig::Vector p = rightArm.EndEffPose();
    p.pop_back();
    p.pop_back();
    p.pop_back();
    p.pop_back();
    p.push_back(1);

    p = baseMatrix*p;

    p.pop_back();

    return iCubUtils::convert(p);

}




////////////////////////
/// \brief VelocityObserver::VelocityObserver
/// \param basename
/// \param local
///
VelocityObserver::VelocityObserver(const std::string& basename, const std::string& local, double thrVel, unsigned int lenVel) {

    // spawn observer process
    obspid = fork();
    if (obspid == 0) {
        // exec velocityObserver
        execlp("velocityObserver", "velocityObserver",
               "--thrVel", std::to_string(thrVel).c_str(),
               "--lenVel", std::to_string(lenVel).c_str(),
               "--name", ("/"+basename).c_str(),
               nullptr);

    } else if (obspid < 0) {
        throw iCubException("VelocityObserver: failed to fork.");
    }


    yarp::os::Network yarp;

    yarp.sync("/"+basename+"/pos:i", false);
    send_port.open("/" + local + "/out");
    if (!yarp.connect("/" + local + "/out","/"+basename+"/pos:i"))
        throw iCubException("VelocityObserver: unable to connect to /"+basename+"/pos:i");

    receive_port.open("/" + local + "/in");
    if (!yarp.connect("/"+basename+"/vel:o","/" + local + "/in"))
        throw iCubException("VelocityObserver: unable to connect to /" + local + "/in");

    last.clear();

}

VelocityObserver::~VelocityObserver() {
    send_port.close();
    receive_port.close();

    // kill observer process
    kill(obspid, 2); // SIGINT
    int obsstatus;
    do {
        waitpid(obspid, &obsstatus, 0);
    } while (!WIFEXITED(obsstatus) && !WIFSIGNALED(obsstatus));

}

Utils::Vector VelocityObserver::derive(const Utils::Vector& x) {

    yarp::os::Bottle& send_bottle = send_port.prepare();
    send_bottle.clear();
    for (auto v : x)
        send_bottle.addDouble(v);
    send_port.write();

    yarp::os::Bottle *receive_bottle = receive_port.read(false);
    Utils::Vector ret(x.size());
    if (receive_bottle != nullptr) {
        for (unsigned int i = 0; i < x.size(); i++) {
            ret[i] = receive_bottle->get(i).asDouble();
        }
        last = ret;
    } else if (last.size() == 0)
        last.resize(x.size());
    return last;

}



//DelayAndObserve::DelayAndObserve(const std::string& basename, const std::string& local, unsigned int steps)
//    :obs(basename, local) {

//    delay.resize(steps);

//}

//std::pair<Utils::Vector, Utils::Vector> DelayAndObserve::derive(const Utils::Vector& x) {

//    if (delay[0].empty()) {
//        for (unsigned int i = 0; i < delay.size(); i++)
//            delay[i] = x;
//    }

//    delay.push_back(x);
//    auto der = obs.derive(x);
//    Utils::Vector in = delay.front();
//    delay.pop_front();

//    return std::make_pair(in, der);

//}
