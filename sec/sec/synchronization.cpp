#include "synchronization.h"

#include <thread>

#include <iostream> //debug only


namespace sec {

Semaphore::Semaphore()
    :mtx(new std::mutex()),
     cv(new std::condition_variable()),
     end(new std::atomic_bool(false)) { }

void Semaphore::wait() {
    std::unique_lock<std::mutex> lk(*mtx);
    cv->wait(lk);
    lk.unlock();
}

void Semaphore::wakeup() {
    cv->notify_one();
}

bool Semaphore::shouldquit() {

    return *end;

}

void Semaphore::sendquit() {

    *end = true;
    wakeup();

}



SemaphoreQueueItem::SemaphoreQueueItem()
    :time(0.0){}

SemaphoreQueueItem::SemaphoreQueueItem(Semaphore semaphore, double time)
    :semaphore(semaphore), time(time){
    reset();
}

void SemaphoreQueueItem::reset() {
    remaining = time;
}

double SemaphoreQueueItem::getRemaining() const {
    return remaining;
}

Semaphore& SemaphoreQueueItem::getSemaphore() {
    return semaphore;
}

void SemaphoreQueueItem::decreaseTime(double t) {
    remaining -= t;
}


bool operator<(const SemaphoreQueueItem& sqi1, const SemaphoreQueueItem& sqi2) {
    return sqi1.getRemaining() < sqi2.getRemaining();
}


SemaphoreQueue::SemaphoreQueue() {}

Semaphore SemaphoreQueue::addItem(double time) {

    Semaphore s;
    SemaphoreQueueItem sqi(s, time);

    insert(sqi);

    return s;

}


SemaphoreQueueItem SemaphoreQueue::advance() {

    if (queue.empty()) {
        throw std::runtime_error("SemaphoreQueue is empty.");
    }

    SemaphoreQueueItem top = queue.front();
    queue.pop_front();

    for (SemaphoreQueueItem& sqi : queue) {
        sqi.decreaseTime(top.getRemaining());
    }

    SemaphoreQueueItem ret = top;

    top.reset();
    insert(top);

    return ret;

}

void SemaphoreQueue::wakeAll() {
    for (SemaphoreQueueItem& sqi : queue) {
        sqi.getSemaphore().wakeup();
    }
}

void SemaphoreQueue::quitAll() {
    for (SemaphoreQueueItem& sqi : queue) {
        sqi.getSemaphore().sendquit();
    }
}

void SemaphoreQueue::print() {
    std::cout << "( ";
    for (SemaphoreQueueItem& sqi : queue) {
        std::cout << sqi.getRemaining() << " ";
    }
    std::cout << ")\n";
}

void SemaphoreQueue::insert(SemaphoreQueueItem& sqi) {

    auto it = queue.begin();
    while (it != queue.end()) {
        if (sqi < *it)
            break;
        it++;
    }

    queue.insert(it, sqi);

}


Synchronizer::Synchronizer(Sleeper* sleeper)
    :sleeper(sleeper) {

    if (sleeper == nullptr)
        throw std::runtime_error("Sleeper can't be null.");

    stop_flag = false;
    started = false;
}

void Synchronizer::setSleeper(Sleeper* sleeper) {

    if (sleeper == nullptr)
        throw std::runtime_error("Sleeper can't be null.");

    this->sleeper = std::shared_ptr<Sleeper>(sleeper);
}

Semaphore Synchronizer::registerSignal(double frequency) {

    double time = 1.0/frequency*1000.0; // time is in milliseconds

    mtx.lock();
    auto s = sq.addItem(time);
    mtx.unlock();

    return s;

}

void Synchronizer::wakeAll() {
    sq.wakeAll();
}

void Synchronizer::quitAll(){
    sq.quitAll();
}

void Synchronizer::start() {

    if (started)
        return;

    std::thread t(&Synchronizer::run, this);
    t.detach();
    started = true;


}

void Synchronizer::stop() {
    stop_flag = true;
}

void Synchronizer::run() {

    while (!stop_flag) {
        auto sqi = sq.advance();
        sleeper->sleep(sqi.getRemaining());
        sqi.getSemaphore().wakeup();
    }

}

void Synchronizer::sleep(double ms) {
    sleeper->sleep(ms);
}

void Synchronizer::print() {
    sq.print();
}

Synchronizer synchronizer;

}
