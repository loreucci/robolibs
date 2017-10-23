#ifndef JOINTSCONTROLLER_H
#define JOINTSCONTROLLER_H

#include <vector>
#include <string>

#include <ros/ros.h>

#include <utilities/vector.h>
#include <sec/node.h>
#include <sec/nodelink.h>


class JointsController : public sec::Node {

public:
    JointsController(const std::vector<std::string>& names, double freq = 100.0);

    virtual void refreshInputs() override;

    virtual bool connected() const override;

    virtual void execute() override;

    virtual std::string parameters() const override;

    sec::NodeIn<Utils::Vector> commands;

protected:
    ros::NodeHandle n;
    std::vector<ros::Publisher> publishers;
    std::vector<std::string> names;
};

#endif // JOINTSCONTROLLER_H
