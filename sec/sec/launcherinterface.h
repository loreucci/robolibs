#ifndef LAUNCHERINTERFACE_H
#define LAUNCHERINTERFACE_H

#include "socketdataexchange.h"


namespace sec {


bool isLauncherMode();

void setLauncherMode(bool mode = true);


extern const std::string launchersocketarg;


class LauncherSocketDataIn : public SocketDataIn {

    Q_OBJECT

public:
    LauncherSocketDataIn(const std::string& socketname, const std::vector<std::string>& expected, const std::vector<double> values, double freq = 0.0);

    virtual void waitForStart() override;

    virtual void execute() override;

};


void addLauncherSocketArgument(const std::string& socketname);


}

#endif // LAUNCHERINTERFACE_H
