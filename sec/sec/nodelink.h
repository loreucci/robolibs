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

#include <utilities/message.h>

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

template <typename T, unsigned int N, unsigned int ID>
std::vector<T> convert(const Message<T, N, ID>& a) {
    return a.to_vector();
}

//template <typename T, unsigned int N, unsigned int ID>
//Message<T, N, ID> convert(const std::vector<T>& a) {
//    return Message<T, N, ID>(a);
//}


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

}

#endif // NODELINK_H

