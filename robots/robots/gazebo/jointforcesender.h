#ifndef JOINTFORCESENDER_H
#define JOINTFORCESENDER_H

#include <string>

#include <sec/node.h>
#include <sec/nodelink.h>

#include <ros/ros.h>


class JointForceSender : public sec::Node {

public:
    JointForceSender(const std::string& jointname, double freq = 100.0);

    virtual void refreshInputs() override;

    virtual bool connected() const override;

    virtual void execute() override;

    virtual std::string parameters() const override;

    sec::NodeIn<double> input;

protected:
    std::string jointname;
    ros::NodeHandle n;
    ros::ServiceClient client;

};

#endif // JOINTFORCESENDER_H
