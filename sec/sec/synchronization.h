#ifndef SYNCHRONIZATION_H
#define SYNCHRONIZATION_H

#include <memory>
#include <condition_variable>
#include <mutex>
#include <queue>
#include <atomic>
#include <thread>

#include "sleeper.h"


namespace sec {

class Semaphore {

public:
    Semaphore();

    void wait();
    void wakeup();
    bool shouldquit();
    void sendquit();

    bool completion_wait();
    void completion_notify(bool finalize);

protected:
    std::shared_ptr<std::mutex> mtx, completion_mtx;
    std::shared_ptr<std::condition_variable> cv, completion_cv;
    std::shared_ptr<std::atomic_bool> end, wakeup_flag, completion_flag, finalized;

};


class SemaphoreQueueItem {

public:
    SemaphoreQueueItem();
    SemaphoreQueueItem(Semaphore semaphore, double time);

    void reset();

    double getRemaining() const;
    Semaphore& getSemaphore();

    void decreaseTime(double t);

protected:

    Semaphore semaphore;
    double time, remaining;

};

bool operator<(const SemaphoreQueueItem& sqi1, const SemaphoreQueueItem& sqi2);


class SemaphoreQueue {

public:
    SemaphoreQueue();

    Semaphore addItem(double time);

    void removeAllItems();

    SemaphoreQueueItem advance();

    void wakeAll();
    void quitAll();

    double getTotalTime();
    bool waitForAllCompletion();

protected:
    std::deque<SemaphoreQueueItem> queue;

    void insert(SemaphoreQueueItem& sqi);

};


class Synchronizer {

public:
    Synchronizer(Sleeper* sleeper = new BasicSleeper());
    ~Synchronizer();

    void setSleeper(Sleeper* sleeper);
    bool isSynchronous();

    Semaphore registerSignal(double frequency);

    void unregisterAll();

    void wakeAll();
    void quitAll();

    void start();
    void stop();

    void sleep(double ms);

    double getTime();

protected:
    bool started;
    SemaphoreQueue sq;
    std::mutex mtx;
    std::shared_ptr<Sleeper> sleeper;
    std::atomic_bool stop_flag;
    std::thread* t;
    double time;

    void run();

};

extern Synchronizer synchronizer;

}

#endif // SYNCHRONIZATION_H
