#include "launcherinterface.h"

#include "argumentparser.h"


namespace sec {


bool launchermode = false;

bool isLauncherMode() {
    return launchermode;
}

void setLauncherMode(bool mode) {
    launchermode = mode;
}



const std::string launchersocketarg("defaultdatasocket");


LauncherSocketDataIn::LauncherSocketDataIn(const std::string& socketname, const std::vector<std::string>& expected, const std::vector<double> values, double freq)
    :SocketDataIn(socketname, expected, values, freq) {

    if (!isLauncherMode()) {
        server->close();
        server->deleteLater();
        server = nullptr;
        loop->processEvents();
        delete loop;
        loop = nullptr;
    }

}

void LauncherSocketDataIn::waitForStart() {
    if (!isLauncherMode()) {
        return;
    }
    SocketDataIn::waitForStart();
}

void LauncherSocketDataIn::execute() {
    if (!isLauncherMode()) {
        return;
    }
    SocketDataIn::execute();
}

void addLauncherSocketArgument(const std::string& socketname) {
    addExpectedArgument(launchersocketarg, socketname);
}

}
