#include <robots/gazebo/gazeborunner.h>
#include <robots/gazebo/jointscontroller.h>
#include <ros/init.h>
#include <sec/node.h>
#include <sec/sec.h>

class ResetNode : public sec::Node
{
public:
    ResetNode(int len, double freq = 100.0) : sec::Node(freq), len(len) {}
    ~ResetNode() {}

    virtual void execute() override
    {
        Utils::Vector vec;
        for (int i = 0; i < len; i++) {
            vec.push_back(0.0);
        }

        joints.addData(vec);
    }

    virtual void refreshInputs() override {}

    virtual bool connected() const { return true; }

    virtual std::string parameters() const { return ""; }

    sec::NodeOut<Utils::Vector> joints;

private:
    int len;
};

class BrokenExperiment
{
public:
    BrokenExperiment() {
        gr.runOnSingleThread();
    }

    void run()
    {
        ResetNode rn(6);
        std::vector<std::string> joints = {
            "/iCub/r_hip_pitch/pos",   "/iCub/r_knee/pos",
            "/iCub/r_ankle_pitch/pos", "/iCub/l_hip_pitch/pos",
            "/iCub/l_knee/pos",        "/iCub/l_ankle_pitch/pos"};
        JointsController jc(joints);

        sec::connect(rn.joints, jc.commands);
        sec::run(1.0);
    }

private:
    GazeboRunner gr;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ros_cose", ros::init_options::NoRosout);
    sec::setVerbose();
    sec::setSleeper(new sec::Barrier());

    BrokenExperiment be;
    while (true) {
        be.run();
    }
}
