#ifndef SYNCHRONIZATION_H
#define SYNCHRONIZATION_H

#include <memory>
#include <condition_variable>
#include <mutex>
#include <queue>
#include <atomic>

#include "sleeper.h"


namespace sec {

class Semaphore {

public:
    Semaphore();

    void wait();
    void wakeup();
    bool shouldquit();
    void sendquit();

protected:
    std::shared_ptr<std::mutex> mtx;
    std::shared_ptr<std::condition_variable> cv;
    std::shared_ptr<std::atomic_bool> end;

};


class SemaphoreQueueItem {

public:
    SemaphoreQueueItem();
    SemaphoreQueueItem(Semaphore semaphore, double time);

    void reset();

    double getRemaining() const;
    Semaphore& getSemaphore();

    void decreaseTime(double t);

//    friend bool operator<(const SemaphoreQueueItem& sqi1, const SemaphoreQueueItem& sqi2);

protected:

    Semaphore semaphore;
    double time, remaining;

};

bool operator<(const SemaphoreQueueItem& sqi1, const SemaphoreQueueItem& sqi2);


class SemaphoreQueue {

public:
    SemaphoreQueue();

    Semaphore addItem(double time);

    SemaphoreQueueItem advance();

    void wakeAll();
    void quitAll();

    void print(); //debug only

protected:
    std::deque<SemaphoreQueueItem> queue;

    void insert(SemaphoreQueueItem& sqi);

};


class Synchronizer {

public:
    Synchronizer(Sleeper* sleeper = new BasicSleeper());

    void setSleeper(Sleeper* sleeper);

    Semaphore registerSignal(double frequency);

    void wakeAll();
    void quitAll();

    void start();
    void stop();

    void run();

    void sleep(double ms);

    double getTime();

    void print(); //debug only

protected:
    bool started;
    SemaphoreQueue sq;
    std::mutex mtx;
    std::shared_ptr<Sleeper> sleeper;
    std::atomic_bool stop_flag;
    double time;

};

extern Synchronizer synchronizer;

}

#endif // SYNCHRONIZATION_H
