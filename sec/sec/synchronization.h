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

/*!
 * \file synchronization.h
 * \brief Classes at the core of the synchronization mechanism.
 */

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

/*!
 * \brief The waiting mechanism for Controller threads.
 *
 * This class implements a stop and wait mechanism between the Controller threads and the Synchronizer thread.
 * The ones in the Controller should stop and wait for the Synchronizer to wake them up.
 * The Synchronizer can also signal the fact that the Controller thread should end.
 *
 * The Controller threads can signal to the Synchronizer that they finished the execution of a step or its complete execution (full-synch mode).
 *
 * A Semaphore is implemented in such a way that it can be copied safely without having to resort to pointers.
 */
class Semaphore {

public:

    /*!
     * \brief Creates the Semaphore.
     *
     * At creation, all mutexes and condition variables are initialized.
     */
    Semaphore();

    /*!
     * \brief Blocks the calling thread until a wake up is signaled.
     *
     * This should be called by a Controller thread.
     */
    void wait();

    /*!
     * \brief Sends a wake up signal to any thread waiting.
     *
     * This is called by the Synchronizer.
     */
    void wakeup();

    /*!
     * \brief Checks whether the calling thread should end.
     *
     * This should be called by a Controller thread.
     *
     * \return true if the calling thread should terminate
     */
    bool shouldquit();

    /*!
     * \brief Sends a termination signal to the Controller thread.
     *
     * This is called by the Synchronizer.
     */
    void sendquit();

    /*!
     * \brief Waits for the Controller thread to signal the end of the execution step.
     *
     * This is called by the Synchronizer (full-synch mode).
     *
     * \return true if the Controller thread has signaled its complete execution
     */
    bool completion_wait();

    /*!
     * \brief Notifies the Synchronizer thread that an execution step has been completed.
     *
     * This should be called by a Controller thread (full-synch mode).
     *
     * \param true if the thread should signal its complete execution
     */
    void completion_notify(bool finalize);

private:
    std::shared_ptr<std::mutex> mtx, completion_mtx;
    std::shared_ptr<std::condition_variable> cv, completion_cv;
    std::shared_ptr<std::atomic_bool> end, wakeup_flag, completion_flag, finalized;

};


/*!
 * \brief An item of SemaphoreQueue.
 *
 * This class represents an element of the priority queue of Semaphores. It includes a Semaphore and a waiting time.
 */
class SemaphoreQueueItem {

public:
    /*!
     * \brief Creates an empty item.
     *
     * The default constructor is needed for usage with std containers.
     */
    SemaphoreQueueItem();

    /*!
     * \brief Creates an item with a Semaphore and a waiting time.
     *
     * Waiting time is reset at construction.
     */
    SemaphoreQueueItem(Semaphore semaphore, double time);

    /*!
     * \brief Resets the waiting time.
     *
     * The waiting time is reset to the original value passed at construction.
     * Used when the Semaphore is moved back inside the queue.
     */
    void reset();

    /*!
     * \brief Gets the waiting time for this item.
     *
     * \return the remaining waiting time
     */
    double getRemaining() const;

    /*!
     * \brief Gets the Semaphore for this item.
     *
     * \return the Semaphore of this item
     */
    Semaphore& getSemaphore();

    /*!
     * \brief Decreases the waiting time by a certain amount.
     *
     * \param t the amount of time to remove from the remaining waiting time
     */
    void decreaseTime(double t);

private:

    Semaphore semaphore;
    double time, remaining;

};


/*!
 * \brief An ordered queue of Semaphores and waiting times.
 *
 * This class implements a priority queue in which semaphores with less waiting times are put on front.
 */
class SemaphoreQueue {

public:
    /*!
     * \brief Creates an empty queue.
     */
    SemaphoreQueue();

    /*!
     * \brief Adds a new item to the queue.
     *
     * This methods adds a new SemaphoreQueueItem to the queue, effectively creating a new Semaphore with the associated waiting time.
     *
     * \param time the waiting time for the Semaphore
     * \return the new Semaphore added to the queue
     */
    Semaphore addItem(double time);

    /*!
     * \brief Clears the queue.
     */
    void removeAllItems();

    /*!
     * \brief Advances the queue.
     *
     * This method moves the first item in the queue back inside (with priority) and updates the remaining waiting times of other items.
     *
     * \return the SemaphoreQueueItem that was moved from the top of the queue
     */
    SemaphoreQueueItem advance();

    /*!
     * \brief Sends a wake up signal from all Semaphores.
     */
    void wakeAll();

    /*!
     * \brief Sends a quit signal from all Semaphores.
     */
    void quitAll();

    /*!
     * \brief Gets the total waiting time for all Semaphores in the queue.
     *
     * \return the total waiting time
     */
    double getTotalTime();

    /*!
     * \brief Wait for all Semaphores to be signaled the completion of the step.
     *
     * This is used in full-synch mode, to wait for all Node steps to be executed.
     * If a Node signals the end of its complete execution this method will return true.
     *
     * \return true if a Node signaled the end of its complete execution
     */
    bool waitForAllCompletion();

private:
    std::deque<SemaphoreQueueItem> queue;

    // inserts an element in the queue, with priority
    void insert(SemaphoreQueueItem& sqi);

};


/*!
* \brief Core synchronization mechanism of the library.
*
* This class in the the core of the synchronization mechanism of the sec library.
* It has two responsibilities:
*   - maintaining and updating the global time
*   - waking up the different thread at the right frequencies
*
* Global time is not necessarily wall time, it depends on the Sleeper that is being used.
* For instance, in case of synchronization with a simulator, the global time is the simulated time.
* The time is reset every time the synchronizer is started. This happens every time sec::run is called.
*
* The threads are woken up at their execution frequency thanks to a queue of semaphores ordered by remaining time (see SemaphoreQueue).
* The class implements the two synchronization mechanisms (real-time and full-synch), which are specified by the Sleeper.
*/
class Synchronizer {

public:

    /*!
     * \brief Creates and initializes the Synchronizer.
     *
     * \param sleeper the sleeper that will be used to wait for waking up threads
     */
    Synchronizer(Sleeper* sleeper = new BasicSleeper());

    /*!
     * \brief Simply joins the synchronization thread.
     */
    ~Synchronizer();

    /*!
     * \brief Set a new Sleeper.
     * \param the new Sleeper that should be used
     */
    void setSleeper(Sleeper* sleeper);

    /*!
     * \brief Checks whether we are in full-synch mode.
     * \return true if full-synch, false if real-time
     */
    bool isSynchronous();

    /*!
     * \brief Register a new signal to wake up a thread.
     *
     * This method adds another Semaphore to the queue that will be signaled at the specified frequency.
     *
     * \param frequency the frequency at which the thread should be woke up
     * \return the new Semaphore created, which should be used by the thread for the synchronization
     */
    Semaphore registerSignal(double frequency);

    /*!
     * \brief Deletes all semaphores from the queue.
     */
    void unregisterAll();

    /*!
     * \brief Sends a wake up signal to all threads.
     */
    void wakeAll();

    /*!
     * \brief Sends a quit signal to all threads.
     */
    void quitAll();

    /*!
     * \brief Starts the synchronization thread.
     *
     * This will start another thread with the timer, which will signal the wake ups. This effectively starts the whole controller.
     */
    void start();

    /*!
     * \brief Stops the synchronization
     *
     * This will stop the thread with the timer.
     */
    void stop();

    /*!
     * \brief Wait a certain amount of time.
     *
     * The time of the sleep is in global time, thus it depends on the Sleeper that is being used.
     *
     * \param ms duration of sleep (in ms)
     */
    void sleep(double ms);

    /*!
     * \brief Returns the current time.
     *
     * The time returned is in global time, thus it depends on the Sleeper that is being used.
     *
     * \return the current time of the run
     */
    double getTime();

private:
    bool started;
    SemaphoreQueue sq;
    std::mutex mtx;
    std::shared_ptr<Sleeper> sleeper;
    std::atomic_bool stop_flag;
    std::thread* t;
    double time;

    void run();

};


/*!
 * \brief The default Synchronizer that is used by all classes of sec.
 */
extern Synchronizer synchronizer;

}

#endif // SYNCHRONIZATION_H
