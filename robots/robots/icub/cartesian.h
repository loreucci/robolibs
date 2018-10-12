#ifndef CARTESIAN_H
#define CARTESIAN_H

#include "utilities/vector.h"

#include "sec/node.h"
#include "sec/nodelink.h"

#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/CartesianControl.h>


class CartesianController : public sec::Node {

public:
    CartesianController(const std::string& robotName, const std::string& part, double freq = 0.0);
    ~CartesianController();

    virtual void refreshInputs() override;

    virtual bool connected() const override;

    virtual void execute() override;

    virtual std::string parameters() const override;

    void goToPose(const Utils::Vector& newpos, const Utils::Vector& newori, bool wait = false);

    std::pair<Utils::Vector, Utils::Vector> getPose() const;

    sec::NodeIn<Utils::Vector> inpos, inori;
    sec::NodeOut<Utils::Vector> currpos, currori;


protected:
    yarp::dev::PolyDriver clientCartCtrl;
    yarp::dev::ICartesianControl* icart;

    std::string part;

};

#endif // CARTESIAN_H
