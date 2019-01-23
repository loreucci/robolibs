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

#ifndef SEC_H
#define SEC_H

#include "commons.h"
#include "controller.h"
#include "connections.h"
#include "synchronization.h"
#include "flags.h"

namespace sec {

void run(double time = 0.0, std::vector<std::function<bool(void)>> endconditions = {});

void setSleeper(Sleeper* sleeper);

void sleep(double ms);

void runTrials(unsigned int N,
               std::vector<std::function<void(void)>> pre = {},
               std::vector<std::function<void(void)>> post = {},
               double time = 0.0,
               std::vector<std::function<bool(void)>> endconditions = {});

}

#endif // SEC_H
