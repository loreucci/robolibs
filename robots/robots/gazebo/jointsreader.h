#ifndef JOINTSREADER_H
#define JOINTSREADER_H

#include <unordered_map>

#include <utilities/vector.h>

#include <sec/simplesources.h>
#include <sec/nodelink.h>

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>


class JointsReader : public sec::Source {

public:
    JointsReader(const std::string& jointtopic, const std::vector<std::string>& joints, double freq = 100.0);

    virtual void execute() override;

    virtual std::string parameters() const override;

    std::unordered_map<std::string, sec::NodeOut<double>> posrad, velrad;
    std::unordered_map<std::string, sec::NodeOut<double>> posdeg, veldeg;

    sec::NodeOut<Utils::Vector> allposrad, allvelrad;
    sec::NodeOut<Utils::Vector> allposdeg, allveldeg;

protected:
    ros::NodeHandle n;
    ros::Subscriber sub;
    std::string topic;
    std::vector<std::string> joints;

    void jointCallback(const sensor_msgs::JointState::ConstPtr& msg);

};

#endif // JOINTSREADER_H
