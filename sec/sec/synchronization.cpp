#include "synchronization.h"

#include <thread>


namespace sec {

Semaphore::Semaphore()
    :mtx(new std::mutex()),
      completion_mtx(new std::mutex()),
      cv(new std::condition_variable()),
      completion_cv(new std::condition_variable()),
      end(new std::atomic_bool(false)),
      wakeup_flag(new std::atomic_bool(false)),
      completion_flag(new std::atomic_bool(false)) {

}

void Semaphore::wait() {
    std::unique_lock<std::mutex> lk(*mtx);
    while(!*wakeup_flag)
        cv->wait(lk);
    *wakeup_flag = false;
    lk.unlock();
}

void Semaphore::wakeup() {
    std::unique_lock<std::mutex> lk(*mtx);
    *wakeup_flag = true;
    lk.unlock();
    cv->notify_one();
}

bool Semaphore::shouldquit() {

    return *end;

}

void Semaphore::sendquit() {

    *end = true;
    wakeup();
    completion_notify();

}

void Semaphore::completion_wait() {
    std::unique_lock<std::mutex> lk(*completion_mtx);
    while (!*completion_flag)
        completion_cv->wait(lk);
    *completion_flag = false;
    lk.unlock();
}

void Semaphore::completion_notify() {
    std::unique_lock<std::mutex> lk(*completion_mtx);
    *completion_flag = true;
    lk.unlock();
    completion_cv->notify_one();
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

double SemaphoreQueue::getTotalTime() {
    double time = 0.0;
    for (const auto& sqi : queue) {
        time += sqi.getRemaining() - time;
    }
    return time;
}

void SemaphoreQueue::waitForAllCompletion() {
    for (auto& sqi : queue) {
        sqi.getSemaphore().completion_wait();
    }
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
    time = 0.0;

    t = nullptr;

}

Synchronizer::~Synchronizer() {
    if (t != nullptr)
        t->join();
}

void Synchronizer::setSleeper(Sleeper* sleeper) {

    if (sleeper == nullptr)
        throw std::runtime_error("Sleeper can't be null.");

    this->sleeper = std::shared_ptr<Sleeper>(sleeper);
}

bool Synchronizer::isSynchronous() {
    return sleeper->isSynchronous();
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

    t = new std::thread(&Synchronizer::run, this);
    started = true;


}

void Synchronizer::stop() {
    stop_flag = true;
}

void Synchronizer::run() {

    if (sleeper->isSynchronous()) {
        // SYNCHRONOUS MODE
        while (!stop_flag) {
            sq.wakeAll();
            time += sq.getTotalTime();
            sq.waitForAllCompletion();
        }

    } else {
        // REALTIME MODE
        while (!stop_flag) {
            auto sqi = sq.advance();
            double remtime = sqi.getRemaining();
            sleeper->sleep(remtime);
            time += remtime;
            sqi.getSemaphore().wakeup();
        }
    }

}

void Synchronizer::sleep(double ms) {
    sleeper->sleep(ms);
}

double Synchronizer::getTime() {
    return time/1000.0;
}

Synchronizer synchronizer;

}
