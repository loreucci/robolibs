#include "icubsim_utilities.h"

#include <yarp/os/Network.h>

iCubSimSleeper::iCubSimSleeper(const std::string& simulatorName, double step_size)
    :step_size(step_size) {

    if (step_size <= 0) {
        throw iCubException("iCubSimSleeper: step_size must be positive.");
    }

    yarp::os::Network yarp;

    step_port.open("/local/step");
    if (!yarp.connect("/"+simulatorName+"/step", "/local/step")) {
        throw iCubException("Unable to connect to /"+simulatorName+"/step.");
    }

}

iCubSimSleeper::~iCubSimSleeper() {
    step_port.close();
}

void iCubSimSleeper::sleep(double d) {
    int steps = std::round(d / step_size);
    for (int i = 0; i < steps; i++) {
        step_port.read(true);
    }
}

bool iCubSimSleeper::isSynchronous() const {
    return false;
}


iCubSimWorld::iCubSimWorld(const std::string& simulatorName) {

    yarp::os::Network yarp;
    world_port.open("/local/world");
    if (!yarp.connect("/local/world", "/"+simulatorName+"/world")) {
        throw iCubException("iCubSimBalls: unable to connect to /"+simulatorName+"/world.");
    }

    counts = {
        {"sph", 1},
        {"ssph", 1},
        {"box", 1},
        {"sbox", 1},
        {"cyl", 1},
        {"scyl", 1},
    };

}

iCubSimWorld::~iCubSimWorld() {
    world_port.close();
}

iCubSimObject iCubSimWorld::createSphere(double ra, double x, double y, double z, double r, double g, double b, bool stat) {

    iCubSimObject obj;
    obj.type = stat ? "ssph" : "sph";
    obj.id = counts[obj.type]++;
    obj.x = x;
    obj.y = y;
    obj.z = z;

    std::string str = "world mk " + obj.type + " ";
    str += std::to_string(ra);
    str += " ";
    str += std::to_string(x) + " " + std::to_string(y) + " " + std::to_string(z);
    str += " ";
    str += std::to_string(r) + " " + std::to_string(g) + " " + std::to_string(b);

    yarp::os::Bottle& btl = world_port.prepare();
    btl.clear();
    btl.fromString(str.c_str());
    world_port.write();

    return obj;

}

iCubSimObject iCubSimWorld::createBox(double h, double w, double d, double x, double y, double z, double r, double g, double b, bool stat) {

    iCubSimObject obj;
    obj.type = stat ? "sbox" : "box";
    obj.id = counts[obj.type]++;
    obj.x = x;
    obj.y = y;
    obj.z = z;

    std::string str = "world mk " + obj.type + " ";
    str += std::to_string(h) + " " + std::to_string(w) + " " + std::to_string(d);
    str += " ";
    str += std::to_string(x) + " " + std::to_string(y) + " " + std::to_string(z);
    str += " ";
    str += std::to_string(r) + " " + std::to_string(g) + " " + std::to_string(b);

    yarp::os::Bottle& btl = world_port.prepare();
    btl.clear();
    btl.fromString(str.c_str());
    world_port.write();

    return obj;

}

iCubSimObject iCubSimWorld::createCylinder(double ra, double l, double x, double y, double z, double r, double g, double b, bool stat) {

    iCubSimObject obj;
    obj.type = stat ? "scyl" : "cyl";
    obj.id = counts[obj.type]++;
    obj.x = x;
    obj.y = y;
    obj.z = z;

    std::string str = "world mk " + obj.type + " ";
    str += std::to_string(ra) + " " + std::to_string(l);
    str += " ";
    str += std::to_string(x) + " " + std::to_string(y) + " " + std::to_string(z);
    str += " ";
    str += std::to_string(r) + " " + std::to_string(g) + " " + std::to_string(b);

    yarp::os::Bottle& btl = world_port.prepare();
    btl.clear();
    btl.fromString(str.c_str());
    world_port.write();

    return obj;

}

void iCubSimWorld::moveObject(const iCubSimObject& obj, double x, double y, double z) {

    if (obj.id > counts[obj.type] || obj.id == 0){
        throw iCubException("iCubSimWorld: wrong object index.");
    }

    std::string str = "world set " + obj.type + " ";
    str += std::to_string(obj.id);
    str += " ";
    str += std::to_string(x);
    str += " ";
    str += std::to_string(y);
    str += " ";
    str += std::to_string(z);

    yarp::os::Bottle& bBall = world_port.prepare();
    bBall.clear();
    bBall.fromString(str.c_str());
    world_port.write();

}

void iCubSimWorld::resetObject(const iCubSimObject& obj) {
    moveObject(obj, obj.x, obj.y, obj.z);
}

void iCubSimWorld::clearAll() {

    yarp::os::Bottle& btl = world_port.prepare();
    btl.clear();
    btl.fromString("world del all");
    world_port.write();

}



iCubSimObjectMover::iCubSimObjectMover(iCubSimWorld& world, iCubSimObject obj, bool relative, double freq)
    :sec::Node(freq), world(world), obj(obj), relative(relative) {}

void iCubSimObjectMover::refreshInputs() {

    if (point.isConnected()) {
        point.refreshData();
    } else {
        if (x.isConnected())
            x.refreshData();
        if (y.isConnected())
            y.refreshData();
        if (z.isConnected())
            z.refreshData();
    }

}

bool iCubSimObjectMover::connected() const {

    bool pointconn = point.isConnected();

    bool coordconn = x.isConnected() || y.isConnected() || z.isConnected();

    if (pointconn && coordconn)
        throw iCubException("iCubSimObjectMover: too many connections.");

    return pointconn || coordconn;

}

void iCubSimObjectMover::execute() {

    double _x = 0.0, _y = 0.0, _z = 0.0;

    if (relative) {
        _x = obj.x;
        _y = obj.y;
        _z = obj.z;
    }

    if (point.isConnected()) {
        auto p = point.getData();
        if (p.size() != 3)
            throw iCubException("iCubSimObjectMover: wrong input point.");
        _x += p[0];
        _y += p[1];
        _z += p[2];
    } else {
        if (x.isConnected())
            _x += x;
        if (y.isConnected())
            _y += y;
        if (z.isConnected())
            _z += z;
    }

    world.moveObject(obj, _x, _y, _z);

}

std::string iCubSimObjectMover::parameters() const {
    return "Object mover node (type=" + obj.type + ",id=" + std::to_string(obj.id) + ").";
}
