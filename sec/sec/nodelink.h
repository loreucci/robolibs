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

// TODO: consider using a condition_variable

namespace sec {

template <typename T>
class NodeOut {

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

    std::pair<T, unsigned int> getData() const {
        mtx->lock();
        auto ret = std::make_pair(data, dataid);
        mtx->unlock();
        return ret;
    }

protected:
    mutable std::shared_ptr<std::mutex> mtx;
    T data;
    unsigned int dataid;

};


template <typename T>
class NodeIn {

public:
    NodeIn(NodeOut<T>* nodeout = nullptr)
        :nodeout(nodeout),
         data(T()),
         lastid(0),
         isnew(false) {
    }

    void connect(const NodeOut<T>* nodeout) {
        this->nodeout = nodeout;
    }

    bool isConnected() const {
        return nodeout != nullptr;
    }

    void refreshData() {
        if (!isConnected())
            throw std::runtime_error("NodeIn: NodeLink is not connected.");
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


protected:
    const NodeOut<T>* nodeout;
    T data;
    unsigned int lastid;
    bool isnew;

};

}

#endif // NODELINK_H

