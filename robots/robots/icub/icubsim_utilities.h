#ifndef ICUBSIM_UTILITIES_H
#define ICUBSIM_UTILITIES_H

#include <string>
#include <cmath>
#include <unordered_map>

#include <yarp/os/BufferedPort.h>

#include <sec/sleeper.h>
#include <sec/nodelink.h>

#include "commons.h"


class iCubSimSleeper : public sec::Sleeper {

public:

    iCubSimSleeper(const std::string& simulatorName = "icubSim", double step_size = 10);
    ~iCubSimSleeper();

    virtual void sleep(double d) override;

    virtual bool isSynchronous() const override;

protected:
    double step_size;
    yarp::os::BufferedPort<yarp::os::Bottle> step_port;

};



struct iCubSimObject {

    std::string type;
    unsigned int id;
    double x, y, z;

};


class iCubSimWorld {

public:
    iCubSimWorld(const std::string& simulatorName = "icubSim");
    ~iCubSimWorld();

    iCubSimObject createSphere(double ra,                     // radius
                               double x, double y, double z,  // position
                               double r, double g, double b,  // color
                               bool stat = true);

    iCubSimObject createBox(double h, double w, double d,  // size
                            double x, double y, double z,  // position
                            double r, double g, double b,  // color
                            bool stat = true);

    iCubSimObject createCylinder(double ra, double l,           // radius, length
                                 double x, double y, double z,  // position
                                 double r, double g, double b,  // color
                                 bool stat = true);

    void moveObject(const iCubSimObject& obj, double x, double y, double z);
    void resetObject(const iCubSimObject& obj);

    void clearAll();

protected:
    yarp::os::BufferedPort<yarp::os::Bottle> world_port;
    std::unordered_map<std::string, unsigned int> counts;

};


class iCubSimObjectMover : public sec::Node {

public:
    iCubSimObjectMover(iCubSimWorld& world, iCubSimObject obj, bool relative, double freq = 0.0);

    virtual void refreshInputs() override;

    virtual bool connected() const override;

    virtual void execute() override;

    virtual std::string parameters() const override;

    void reset();

    sec::NodeIn<Utils::Vector> point;

    sec::NodeIn<double> x, y, z;


protected:
    iCubSimWorld& world;
    iCubSimObject obj;
    bool relative;

};

#endif // ICUBSIM_UTILITIES_H
