/**
 * @file RtThread.h
 *
 * Small C++ wrapper class to create a realtime scheduled pthread
 *
 * @author Jan Sommer
 *  Created on: Apr 10, 2013
 *
 */


#ifndef RTTHREAD_H
#define RTTHREAD_H

#include<pthread.h>

///TODO: Make some proper exceptions

namespace USU {

/*!
 \brief Abstract wrapper class for the pthread library with RT-priority

 This class is a thin wrapper for the pthread library. Inherited classes need
 to implement the run function with the tasks for the thread. The thread will
 run with the SCHED_FIFO-scheduler at the set priority. Therefore root rights
 are necessary for changing the scheduling policy.

 \ingroup threading
 \ingroup 3dm
*/
class RtThread
{

private:
    pthread_attr_t mAttr; /*!< Attributes of the thread, used to set the scheduler and priority */
    int mPriority; /*!< Priority of the thread*/

    RtThread(const RtThread& thread); /*!< Copy constructor made inaccessible by declaring it private */
    RtThread& operator=(const RtThread& rhs); /*!< Assignment constructor made inaccessible by declaring it private */

protected:
    pthread_t mId; /*!< The thread handle */
    bool mStarted; /*!< Keeps the status of the thread TODO: Useful?? */
    void *mArgs; /*!< Arguments which can be passed to a certain thread thread */
    /*!
     \brief Function passed to pthread_create,  do not call manually!

     This function builds the interface to the pthread library. Only purpose is to
     be compatible to pthread_create, as it will immediately call run of this class.

     \param thr pointer to this instance of the class.
    */
    static void *exec(void * thr);

public:
    /*!
     \brief Creates the RtThread object

      Prepares the Attribute object which is passed to pthread_create
      later.

    \param priority the Priority of the Thread (Linux: 1..99)
    */
    RtThread(int priority = 0);

    /*!
     \brief Destructor of the RtThread object

      Waits for the thread to join (if not already)
      and releases the Attributes object.

    */
    virtual ~RtThread();

    /*!
     \brief Return the pthread handle

     \return pthread_t the thread handle of the last started pthread or -1 (if no pthread was started)
    */
    pthread_t getThreadId() const;

    /*!
     \brief Returns the priority of the thread

     \return int priority
    */
    int getPriority() const;

    /*!
     \brief Creates and starts the pthread

      Creates the pthread with the desired attributes.

     \param args optional arguments for the thread
    */
    void start(void * args = NULL);

    /*!
     \brief Waits for the thread to join

     \param timeout_ms timeout in ms (optional). 0 means no timeout
     \return bool returns true if thread joined successfully and false if error occured
    */
    bool join(int timeout_ms = 0);

    /*!
     \brief Actual method of the thread is running

        Every child class has to implement this function
        in order to do some threaded work.
    */
    virtual void run() = 0;
};

}

#endif // RTTHREAD_H
