#include "pid.h"

#include <iostream>

PID::PID(double kp, double ki, double kd, double freq)
    :sec::Node(freq), kp(kp), ki(ki), kd(kd) {

    preverr = 0.0;
    erri = 0.0;

}

void PID::refreshInputs() {
    ref.refreshData();
    enc.refreshData();
}

bool PID::connected() const {
    return ref.isConnected() && enc.isConnected();
}

void PID::execute() {

    // P
    double err = ref - enc;

    // I
    erri += (preverr+err)/freq/2.0;


    // D
    double errd = (err-preverr)/freq;

    output = kp * err + ki * erri + kd * errd;

}

std::string PID::parameters() const {

    std::string ret = "PID controller with:";
    ret += "\tKp = " + std::to_string(kp);
    ret += "\tKi = " + std::to_string(ki);
    ret += "\tKd = " + std::to_string(kd);

    return ret;

}
