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

//!  \file nodelink.h
/*!
  This file contains the classes that link nodes together.
*/

#ifndef NODELINK_H
#define NODELINK_H

#include <mutex>
#include <memory>
#include <utility>
#include <stdexcept>
#include <vector>
#include <functional>

// TODO: consider using a condition_variable

namespace sec {

template <typename T>
class LinkSource {

public:
    virtual std::pair<T, unsigned int> getData() const = 0;

};


template <typename T>
class NodeOut : public LinkSource<T> {

public:
    NodeOut()
        :mtx(new std::mutex()), dataid(0) {
    }

    NodeOut<T>& operator=(const T& data) {
        addData(data);
        return *this;
    }

    void addData(const T& data) {
        mtx->lock();
        this->data = data;
        dataid++;
        mtx->unlock();
    }

    virtual std::pair<T, unsigned int> getData() const override {
        mtx->lock();
        auto ret = std::make_pair(data, dataid);
        mtx->unlock();
        return ret;
    }

    using type = T;

protected:
    mutable std::shared_ptr<std::mutex> mtx;
    T data;
    unsigned int dataid;

};


template <typename T>
class NodeIn {

public:
    NodeIn(const LinkSource<T>* nodeout = nullptr)
        :nodeout(nodeout),
         data(T()),
         lastid(0),
         isnew(false) {
    }

    void connect(const LinkSource<T>* nodeout) {
        this->nodeout = nodeout;
    }

    bool isConnected() const {
        return nodeout != nullptr;
    }

    void refreshData() {
        if (!isConnected())
            return;
        auto d = nodeout->getData();
        if (d.second != lastid) {
            data = d.first;
            lastid = d.second;
            isnew = true;
        } else {
            isnew = false;
        }
    }

    operator T() const {
        return data;
    }

    T getData() const {
        return data;
    }

    bool isNew() const {
        return isnew;
    }

    using type = T;

protected:
    const LinkSource<T>* nodeout;
    T data;
    unsigned int lastid;
    bool isnew;

};


template <typename A, typename B>
B convert(const A& a) {
    return B(a);
}

std::vector<double> convert(const double& a);
double convert(const std::vector<double>& a);


template <typename A, typename B>
class LinkConverter : public LinkSource<B> {

public:

    LinkConverter(const NodeOut<A>* nodeout) {
        this->nodeout = nodeout;
    }

    virtual std::pair<B, unsigned int> getData() const override {
        auto r = nodeout->getData();
        return std::make_pair(convert(r.first), r.second);
    }

protected:
    const NodeOut<A>* nodeout;

};


template <typename A, typename B>
class LinkFunction : public LinkSource<B> {

public:

    LinkFunction(const NodeOut<A>* nodeout, std::function<B(A)> fun)
        :nodeout(nodeout), fun(fun) {}

    virtual std::pair<B, unsigned int> getData() const override {
        auto r = nodeout->getData();
        return std::make_pair(fun(r.first), r.second);
    }

protected:
    const NodeOut<A>* nodeout;
    std::function<B(A)> fun;

};

}

#endif // NODELINK_H

