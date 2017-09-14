#ifndef ICUBROBOT_H
#define ICUBROBOT_H

#include <string>
#include <vector>
#include <sec/source.h>

#include <utilities/utilities.h>

extern unsigned int iCubRobotInstanceCount;

template <class ... Parts>
class iCubRobot : public sec::Source, public Parts... {

public:
    iCubRobot(const std::string& robotname, double encoderFreq = 100.0)
        :sec::Source(encoderFreq), robotname(robotname) {

        localname = "robot" + std::to_string(iCubRobotInstanceCount);
        iCubRobotInstanceCount++;

        activateAll();

    }

    ~iCubRobot() {
        using expander = int[];
        expander{0, (Parts::deactivate(), 0)... };
    }

    void activateAll() {
        // some black magic from
        // https://stackoverflow.com/questions/30563254/how-can-i-expand-call-to-variadic-template-base-classes
        // in C++17 should be:
        // (Types::activate(), ...);
        // http://en.cppreference.com/w/cpp/language/fold

        using expander = int[];
        expander{0, (Parts::activate(robotname, localname), 0)... };
    }

    void refreshEncoders() {
        using expander = int[];
        expander{0, (Parts::refresh(), 0)... };
    }

    virtual void execute() override {
        refreshEncoders();
    }

    virtual std::string parameters() const override {
        std::vector<std::string> list{Parts::name()...};
        return "iCubRobot with: " + Utils::make_string(list, ", ");
    }

protected:
    std::string robotname, localname;

};

#endif // ICUBROBOT_H
