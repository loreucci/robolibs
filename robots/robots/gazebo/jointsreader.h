#ifndef JOINTSREADER_H
#define JOINTSREADER_H

#include <sec/simplesources.h>
#include <sec/nodelink.h>

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

class JointsReader : public sec::Source {

public:
    JointsReader(double freq = 100.0);

    virtual void execute() override;

    virtual std::string parameters() const override;

    sec::NodeOut<double> posrad, velrad;
    sec::NodeOut<double> posdeg, veldeg;

protected:
    ros::NodeHandle n;
    ros::Subscriber sub;

    void jointCallback(const sensor_msgs::JointState::ConstPtr& msg);

};

#endif // JOINTSREADER_H



//class CommandSender {

//public:
//    CommandSender(address) {

//        joints.initialize(address);
//        ros::NodeHandle n;
//        sub = n.subscribe("", 1000, callback, this);

//    }


//private:
//    Joint joints;
//    ros::Subscriber sub;

//    void callback(whatever) {
//        joints.send_angle();
//    }

//};

//int main() {
//    CommandSender sender(address);

//}
