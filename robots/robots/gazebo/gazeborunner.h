#ifndef GAZEBORUNNER_H
#define GAZEBORUNNER_H

#include <sec/node.h>

#include <ros/ros.h>

// This class needs the custom Gazebo plugins that can be found at:
// https://bitbucket.org/hbpneurorobotics/gazeborospackages
class GazeboRunner : public sec::Node {

public:
    GazeboRunner(double freq = 100.0);
    virtual ~GazeboRunner();

    virtual void refreshInputs() override;

    virtual bool connected() const override;

    virtual void execute() override;

    virtual std::string parameters() const override;

protected:
    ros::NodeHandle n;
    ros::ServiceClient advancesim;
    double timestep;

};

#endif // GAZEBORUNNER_H
