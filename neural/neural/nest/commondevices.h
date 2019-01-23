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

#ifndef NESTCOMMONDEVICES_H
#define NESTCOMMONDEVICES_H

#include <sec/nodelink.h>

#include "../spikes.h"
#include "datainterfaces.h"


namespace nest {

class PoissonGeneratorSetter : public StatusSetter {

public:
    using StatusSetter::StatusSetter;

    virtual bool connected() const;

    virtual void refreshInputs();

    sec::NodeIn<double> rate;

protected:
    virtual void generateParams() override;

};


class SpikeDetectorGetter : public StatusGetter {

public:
    using StatusGetter::StatusGetter;

    void generatePyplot();

    neural::SpikeNodeOut spikes;

protected:
    unsigned lastlen;
    virtual void consumeParams() override;

};

SpikeDetectorGetter createSpikeDetectorGetter(const boost::python::tuple& gids);

}

#endif // NESTCOMMONDEVICES_H
