/*
 * Copyright (C) 2014-2019 Lorenzo Vannucci
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
 */

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
