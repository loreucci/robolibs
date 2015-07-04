#ifndef THREADSLINK_H
#define THREADSLINK_H

#include <mutex>
#include <memory>

// TODO: consider using a condition_variable
template <typename T>
class ThreadsLink {

public:
    ThreadsLink()
        :mtx(new std::mutex()){
    }

    void addData(const T& data) {
        mtx->lock();
        this->data = data;
        mtx->unlock();
    }

    T getData() const {
        mtx->lock();
        T ret = data;
        mtx->unlock();
        return ret;
    }


protected:
    mutable std::shared_ptr<std::mutex> mtx;
    T data;
};

#endif // THREADSLINK_H
